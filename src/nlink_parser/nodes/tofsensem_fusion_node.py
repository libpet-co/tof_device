#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TofsenseM 融合节点（Z 深度版，PointXYZIRCAEDT 输出）
- 输入 dis 为 Z 深度（mm），小孔模型重建：X=u*Z, Y=v*Z, Z=Z
- GridMap elevation 写入 Z（米）
- 动态分辨率：以 Z 中位数估算覆盖尺寸
- PointCloud2 字段按以下顺序与类型构造（满足你的 C++ same_layout 校验）：
  x (FLOAT32)          offset=0
  y (FLOAT32)          offset=4
  z (FLOAT32)          offset=8
  intensity (UINT8)    offset=12
  return_type (UINT8)  offset=13
  channel (UINT16)     offset=14
  azimuth (FLOAT32)    offset=16   = atan2(y, x)
  elevation (FLOAT32)  offset=20   = atan2(z, distance) 其中 distance=hypot(x,y,z)
  distance (FLOAT32)   offset=24   = hypot(x, y, z)
  time_stamp (UINT32)  offset=28   = 相对 header 时刻的纳秒偏移（无逐点时间戳则置 0）
point_step = 32
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

        # 深度定义：默认 "z"
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
        self._build_camera_lut()

        # ---------- 时间中值滤波 buffer ----------
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
          - self.u_lut, self.v_lut：像平面归一化坐标（以 z=1 的比例，= x/z, y/z）
          - self.dir_lut：归一化射线方向 (x,y,z) ~ (u,v,1)/||.||
          - self.dz_lut：dir 的 z 分量（= 1/n）
        """
        fov_x = rad(self.fov_x_deg)
        fov_y = rad(self.fov_y_deg)
        cx = (self.cols - 1) / 2.0
        cy = (self.rows - 1) / 2.0

        # 等效焦距（像素）
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
                n = math.sqrt(u * u + v * v + 1.0)
                self.u_lut[r, c] = u
                self.v_lut[r, c] = v
                self.dir_lut[r, c, :] = [u / n, v / n, 1.0 / n]
                self.dz_lut[r, c] = 1.0 / n

    # === 工具函数 ===
    @staticmethod
    def _hypot3(x: float, y: float, z: float) -> float:
        return math.sqrt(x * x + y * y + z * z)

    @staticmethod
    def _saturate_uint8(val) -> int:
        try:
            v = int(val)
        except Exception:
            v = 0
        return max(0, min(255, v))

    def _extract_intensity(self, p) -> int:
        """
        从像素中提取强度（若无相关字段则返回 0）：
        优先顺序：amp/intensity/strength/conf
        """
        cand = None
        for name in ("amp", "intensity", "strength", "conf"):
            if hasattr(p, name):
                cand = getattr(p, name)
                break
        return self._saturate_uint8(cand if cand is not None else 0)

    def _pc2_from_points(self, points):
        """
        将点列表（每点 10 元素：x,y,z,intensity,return_type,channel,azimuth,elevation,distance,time_stamp）
        打包为 PointCloud2，字段偏移严格对应 C++ offsetof。
        """
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id

        # 偏移布局：
        # x:0, y:4, z:8, intensity:12(u8), return_type:13(u8), channel:14(u16),
        # azimuth:16, elevation:20, distance:24, time_stamp:28(u32)
        fields = [
            PointField('x', 0,  PointField.FLOAT32, 1),
            PointField('y', 4,  PointField.FLOAT32, 1),
            PointField('z', 8,  PointField.FLOAT32, 1),
            PointField('intensity',   12, PointField.UINT8,   1),
            PointField('return_type', 13, PointField.UINT8,   1),
            PointField('channel',     14, PointField.UINT16,  1),
            PointField('azimuth',     16, PointField.FLOAT32, 1),
            PointField('elevation',   20, PointField.FLOAT32, 1),
            PointField('distance',    24, PointField.FLOAT32, 1),
            PointField('time_stamp',  28, PointField.UINT32,  1),
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
        dim_col = MultiArrayDimension(label="column_index", size=self.cols, stride=self.cols * self.rows)
        dim_row = MultiArrayDimension(label="row_index",  size=self.rows, stride=self.rows)
        arr.layout.dim = [dim_col, dim_row]
        arr.layout.data_offset = 0
        arr.data = grid_2d.astype(np.float32).ravel(order='C').tolist()
        return arr

    # === 回调 ===
    def cb(self, msg: TofsenseMFrame0):
        # 读取并按 idx -> (r, c) 排列
        N = min(len(msg.pixels), self.rows * self.cols)

        # 1) 数据有效性 + 时间中值滤波
        # main_depth：
        # - depth_mode == 'z'：保存 Z（米）
        # - depth_mode == 'los'：保存斜距 R（米）
        main_depth = np.full((self.rows, self.cols), np.nan, dtype=np.float32)
        valid_vals = []  # 用于动态分辨率统计（Z 中位数）

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

                    # 用于动态分辨率：取 Z 值
                    if self.depth_mode == "z":
                        valid_vals.append(d_use)
                    else:  # 'los'
                        valid_vals.append(d_use * self.dz_lut[r, c])

        # 2) 高程图分辨率与覆盖尺寸（按 Z 中位数）
        if self.resolution_mode == "dynamic" and len(valid_vals) > 0:
            D_z = float(np.median(valid_vals))
        else:
            D_z = float(self.nominal_distance)

        W = 2.0 * D_z * math.tan(rad(self.fov_x_deg) / 2.0)
        H = 2.0 * D_z * math.tan(rad(self.fov_y_deg) / 2.0)
        res_x = W / max(self.cols, 1)
        res_y = H / max(self.rows, 1)
        resolution = float((res_x + res_y) / 2.0)

        # 3) 生成点云（PointXYZIRCAEDT）
        points = []
        for r in range(self.rows):
            for c in range(self.cols):
                idx = r * self.cols + c
                d = main_depth[r, c]

                # 提取强度、回波类型、通道
                if math.isfinite(d):
                    I = self._extract_intensity(msg.pixels[idx])
                    Rtype = int(getattr(msg.pixels[idx], "return_type", 0))
                    Rtype = max(0, min(255, Rtype))  # UINT8
                    Chan = int(idx)                  # UINT16: 用像素线性索引作为通道
                    Chan = max(0, min(65535, Chan))
                else:
                    I = 0
                    Rtype = 0
                    Chan = 0

                if not math.isfinite(d):
                    if self.drop_invalid:
                        continue
                    else:
                        # 无效点：浮点 NaN，整型 0
                        points.append([
                            float('nan'), float('nan'), float('nan'),
                            0, 0, 0,
                            float('nan'), float('nan'), float('nan'),
                            0
                        ])
                        continue

                # 坐标
                if self.depth_mode == "z":
                    x = self.u_lut[r, c] * d
                    y = self.v_lut[r, c] * d
                    z = d
                else:
                    dir_xyz = self.dir_lut[r, c, :]
                    x, y, z = (dir_xyz * d).tolist()

                # 衍生量
                dist = self._hypot3(x, y, z)
                az = math.atan2(y, x)
                el = math.atan2(z, dist) if dist > 0.0 else 0.0
                t_ns = 0  # 无逐点时间戳 -> 0

                points.append([x, y, z, I, Rtype, Chan, az, el, dist, t_ns])

        cloud = self._pc2_from_points(points)
        self.pub_cloud.publish(cloud)

        # 4) 生成 GridMap（elevation = Z）
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
