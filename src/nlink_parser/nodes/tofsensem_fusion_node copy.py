#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TofsenseM 融合节点（Z 深度版）
- 设备 dis 为 Z 深度（mm）时，点云重建采用小孔模型：X=u*Z, Y=v*Z, Z=Z
- GridMap elevation 写入 Z（米）
- 动态分辨率时，以 Z 中位数估算地面覆盖尺寸
"""

import math
import numpy as np
from collections import deque

import rospy
from nlink_parser.msg import TofsenseMFrame0
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Float32MultiArray, MultiArrayDimension
import sensor_msgs.point_cloud2 as pc2

from grid_map_msgs.msg import GridMap, GridMapInfo
from geometry_msgs.msg import Pose, Point, Quaternion


def rad(deg: float) -> float:
    return deg * math.pi / 180.0


class TofsenseMFusionNode:
    def __init__(self):
        # ---------- 基本参数 ----------
        self.frame_id = rospy.get_param("~frame_id", "tofsensem_optical_frame")
        self.topic_in = rospy.get_param("~topic_in", "/nlink_tofsensem_frame0")
        self.topic_cloud = rospy.get_param("~topic_pointcloud", "/tofsensem/points")
        self.topic_grid = rospy.get_param("~topic_gridmap", "/tofsensem/grid_map")

        self.rows = int(rospy.get_param("~rows", 8))
        self.cols = int(rospy.get_param("~cols", 8))

        # 视场角（度）
        self.fov_x_deg = float(rospy.get_param("~fov_x_deg", 27.0))
        self.fov_y_deg = float(rospy.get_param("~fov_y_deg", 27.0))

        # 有效性与滤波
        self.min_range = float(rospy.get_param("~min_range", 0.02))  # m
        self.max_range = float(rospy.get_param("~max_range", 2.0))   # m
        self.drop_invalid = bool(rospy.get_param("~drop_invalid", True))

        self.filter_window = int(rospy.get_param("~filter_window", 3))
        self.use_temporal_median = bool(rospy.get_param("~use_temporal_median", True))

        # 高程图分辨率估计
        self.resolution_mode = rospy.get_param("~resolution_mode", "fixed")  # "fixed" or "dynamic"
        self.nominal_distance = float(rospy.get_param("~nominal_distance", 2.0))  # m
        self.layer_name = rospy.get_param("~layer_name", "elevation")
        self.latch_grid = bool(rospy.get_param("~latch_grid", True))

        # 深度定义：默认 "z"（本次需求）
        self.depth_mode = rospy.get_param("~depth_mode", "z").lower()  # "z" or "los"
        if self.depth_mode not in ("z", "los"):
            rospy.logwarn("~depth_mode 仅支持 'z' 或 'los'，已回退为 'z'")
            self.depth_mode = "z"

        # 可选调试
        debug = bool(rospy.get_param("~debug", False))
        debug_port = int(rospy.get_param("~debug_port", 5678))
        if debug:
            try:
                import debugpy
                debugpy.listen(("0.0.0.0", debug_port))
                rospy.loginfo("Waiting for VS Code debugger on port %d...", debug_port)
                debugpy.wait_for_client()
            except Exception as e:
                rospy.logwarn("debugpy 初始化失败：%s", e)

        # ---------- 预计算像素 LUT ----------
        # 小孔模型：u = (c-cx)/fx, v = (r-cy)/fy
        self._build_camera_lut()

        # ---------- 为时间中值滤波建立 buffer ----------
        N = self.rows * self.cols
        self.buffers = [deque(maxlen=self.filter_window) for _ in range(N)]

        # ---------- ROS pub/sub ----------
        self.pub_cloud = rospy.Publisher(self.topic_cloud, PointCloud2, queue_size=1)
        self.pub_grid = rospy.Publisher(self.topic_grid, GridMap, queue_size=1, latch=self.latch_grid)
        self.sub = rospy.Subscriber(self.topic_in, TofsenseMFrame0, self.cb, queue_size=10)

        rospy.loginfo(
            "tofsensem_fusion (depth_mode=%s): rows=%d cols=%d FOVx=%.1f FOVy=%.1f max_range=%.2f mode=%s",
            self.depth_mode, self.rows, self.cols, self.fov_x_deg, self.fov_y_deg,
            self.max_range, self.resolution_mode
        )

    # === LUT 构建（小孔模型） ===
    def _build_camera_lut(self):
        """
        构建：
          - self.u_lut, self.v_lut：像平面归一化坐标（单位：以 z=1 的比例，等于 x/z, y/z）
          - self.dir_lut：单位射线方向（x,y,z），与 (u,v,1) 归一化一致
          - self.dz_lut：dir 的 z 分量
        """
        fov_x = rad(self.fov_x_deg)
        fov_y = rad(self.fov_y_deg)
        cx = (self.cols - 1) / 2.0
        cy = (self.rows - 1) / 2.0

        # 等效焦距（以“像素”为单位）
        fx = (self.cols / 2.0) / math.tan(fov_x / 2.0)
        fy = (self.rows / 2.0) / math.tan(fov_y / 2.0)

        self.u_lut = np.zeros((self.rows, self.cols), dtype=np.float32)
        self.v_lut = np.zeros((self.rows, self.cols), dtype=np.float32)
        self.dir_lut = np.zeros((self.rows, self.cols, 3), dtype=np.float32)
        self.dz_lut = np.zeros((self.rows, self.cols), dtype=np.float32)

        for r in range(self.rows):
            for c in range(self.cols):
                u = (c - cx) / fx  # = x/z
                v = (r - cy) / fy  # = y/z
                n = math.sqrt(u*u + v*v + 1.0)
                self.u_lut[r, c] = u
                self.v_lut[r, c] = v
                self.dir_lut[r, c, :] = [u/n, v/n, 1.0/n]
                self.dz_lut[r, c] = 1.0 / n

    # === 工具函数 ===
    def _pc2_from_points(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        fields = [
            PointField('x', 0,  PointField.FLOAT32, 1),
            PointField('y', 4,  PointField.FLOAT32, 1),
            PointField('z', 8,  PointField.FLOAT32, 1),
        ]
        return pc2.create_cloud(header, fields, points)

    def _gridmap_info(self, resolution, len_x, len_y):
        info = GridMapInfo()
        info.header.frame_id = self.frame_id
        info.header.stamp = rospy.Time.now()
        info.resolution = float(resolution)
        info.length_x = float(len_x)
        info.length_y = float(len_y)
        pose = Pose()
        pose.position = Point(0.0, 0.0, 0.0)
        pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        info.pose = pose
        return info

    def _pack_layer(self, grid_2d):
        arr = Float32MultiArray()
        dim_col = MultiArrayDimension(label="column_index", size=self.cols, stride=self.cols*self.rows)
        dim_row = MultiArrayDimension(label="row_index",  size=self.rows, stride=self.rows)
        arr.layout.dim = [dim_col, dim_row]
        arr.layout.data_offset = 0
        arr.data = grid_2d.astype(np.float32).ravel(order='C').tolist()
        return arr

    # === 回调 ===
    def cb(self, msg: TofsenseMFrame0):
        # 读取并按 idx -> (r, c) 排列
        N = min(len(msg.pixels), self.rows*self.cols)

        # 1) 数据有效性 + 时间中值滤波
        # 保存“主深度”：
        # - 若 depth_mode == 'z'：保存 Z（米）
        # - 若 depth_mode == 'los'：保存斜距 R（米）
        main_depth = np.full((self.rows, self.cols), np.nan, dtype=np.float32)
        valid_vals = []  # 用于动态分辨率的统计（最后会转为 Z 中位数）

        for idx in range(N):
            p = msg.pixels[idx]
            r = idx // self.cols
            c = idx % self.cols

            if getattr(p, "dis_status", 0) == 0 and math.isfinite(p.dis):
                d_m = float(p.dis) / 1000.0  # mm -> m
                if self.min_range <= d_m <= self.max_range:
                    # 时间中值滤波
                    if self.use_temporal_median:
                        self.buffers[idx].append(d_m)
                        d_use = float(np.median(self.buffers[idx])) if len(self.buffers[idx]) > 0 else d_m
                    else:
                        d_use = d_m

                    main_depth[r, c] = d_use

                    # 用于动态分辨率：我们要的是 Z 中位数
                    if self.depth_mode == "z":
                        valid_vals.append(d_use)
                    else:  # 'los'
                        valid_vals.append(d_use * self.dz_lut[r, c])

        # 2) 计算高程图分辨率与物理尺寸（按 Z 中位数）
        if self.resolution_mode == "dynamic" and len(valid_vals) > 0:
            D_z = float(np.median(valid_vals))  # 当前帧的名义 Z 距离
        else:
            D_z = float(self.nominal_distance)

        # 覆盖宽高：W = 2*Z*tan(FOV/2)
        W = 2.0 * D_z * math.tan(rad(self.fov_x_deg)/2.0)
        H = 2.0 * D_z * math.tan(rad(self.fov_y_deg)/2.0)
        # 单元分辨率（米/格），取均值
        res_x = W / max(self.cols, 1)
        res_y = H / max(self.rows, 1)
        resolution = float((res_x + res_y) / 2.0)

        # 3) 生成点云
        points = []
        for r in range(self.rows):
            for c in range(self.cols):
                d = main_depth[r, c]
                if not math.isfinite(d):
                    if self.drop_invalid:
                        continue
                    else:
                        points.append([float('nan'), float('nan'), float('nan')])
                        continue

                if self.depth_mode == "z":
                    # Z 深度：X=u*Z, Y=v*Z, Z=Z
                    x = self.u_lut[r, c] * d
                    y = self.v_lut[r, c] * d
                    z = d
                    points.append([x, y, z])
                else:
                    # LOS 深度：沿射线乘斜距
                    dir_xyz = self.dir_lut[r, c, :]
                    xyz = (dir_xyz * d).tolist()
                    points.append(xyz)

        cloud = self._pc2_from_points(points)
        self.pub_cloud.publish(cloud)

        # 4) 生成 GridMap（elevation = Z）
        z_grid = np.full_like(main_depth, np.nan, dtype=np.float32)
        if self.depth_mode == "z":
            z_grid = main_depth.astype(np.float32, copy=True)
        else:
            # 将 LOS 转为 Z：Z = R * dz
            z_grid = (main_depth * self.dz_lut).astype(np.float32)

        gm = GridMap()
        gm.info = self._gridmap_info(resolution=resolution, len_x=W, len_y=H)
        gm.layers = [self.layer_name]
        gm.basic_layers = [self.layer_name]
        gm.data = [self._pack_layer(z_grid)]
        self.pub_grid.publish(gm)


def main():
    rospy.init_node("tofsensem_fusion_node")
    TofsenseMFusionNode()
    rospy.loginfo("tofsensem_fusion_node started.")
    rospy.spin()


if __name__ == "__main__":
    main()
