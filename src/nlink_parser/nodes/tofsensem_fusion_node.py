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
from collections import deque
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from geometry_msgs.msg import Point, Pose, Quaternion
from grid_map_msgs.msg import GridMap, GridMapInfo
from nlink_parser.msg import TofsenseMFrame0
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray, Header, MultiArrayDimension


def rad(deg: float) -> float:
    return deg * math.pi / 180.0


class TofsenseMFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("tofsensem_fusion_node")
        # ---------- 基本参数 ----------
        self.frame_id = self.declare_parameter("frame_id", "tofsensem_optical_frame").value
        self.topic_in = self.declare_parameter("topic_in", "/nlink_tofsensem_frame0").value
        self.topic_cloud = self.declare_parameter("topic_pointcloud", "/tofsensem/points").value
        self.topic_grid = self.declare_parameter("topic_gridmap", "/tofsensem/grid_map").value

        self.rows = int(self.declare_parameter("rows", 8).value)
        self.cols = int(self.declare_parameter("cols", 8).value)

        # 视场角（度）
        self.fov_x_deg = float(self.declare_parameter("fov_x_deg", 27.0).value)
        self.fov_y_deg = float(self.declare_parameter("fov_y_deg", 27.0).value)

        # 有效性与滤波
        self.min_range = float(self.declare_parameter("min_range", 0.02).value)  # m
        self.max_range = float(self.declare_parameter("max_range", 2.0).value)   # m
        self.drop_invalid = bool(self.declare_parameter("drop_invalid", True).value)

        self.filter_window = int(self.declare_parameter("filter_window", 3).value)
        self.use_temporal_median = bool(self.declare_parameter("use_temporal_median", True).value)

        # 高程图分辨率估计
        self.resolution_mode = str(self.declare_parameter("resolution_mode", "fixed").value)
        self.nominal_distance = float(self.declare_parameter("nominal_distance", 2.0).value)  # m
        self.layer_name = str(self.declare_parameter("layer_name", "elevation").value)
        self.latch_grid = bool(self.declare_parameter("latch_grid", True).value)

        # 深度定义：默认 "z"
        self.depth_mode = str(self.declare_parameter("depth_mode", "z").value).lower()
        if self.depth_mode not in ("z", "los"):
            self.get_logger().warn("~depth_mode 仅支持 'z' 或 'los'，已回退为 'z'")
            self.depth_mode = "z"

        # 可选调试
        debug = bool(self.declare_parameter("debug", False).value)
        debug_port = int(self.declare_parameter("debug_port", 5678).value)
        if debug:
            try:
                import debugpy  # type: ignore

                debugpy.listen(("0.0.0.0", debug_port))
                self.get_logger().info("Waiting for VS Code debugger on port %d...", debug_port)
                debugpy.wait_for_client()
            except Exception as exc:  # pragma: no cover - optional debug
                self.get_logger().warn(f"debugpy 初始化失败：{exc}")

        # ---------- 预计算像素 LUT ----------
        self._build_camera_lut()

        # ---------- 时间中值滤波 buffer ----------
        N = self.rows * self.cols
        self.buffers: List[deque] = [deque(maxlen=self.filter_window) for _ in range(N)]

        # ---------- ROS pub/sub ----------
        self.pub_cloud = self.create_publisher(PointCloud2, self.topic_cloud, QoSProfile(depth=1))
        grid_qos = QoSProfile(depth=1)
        if self.latch_grid:
            grid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_grid = self.create_publisher(GridMap, self.topic_grid, grid_qos)
        self.sub = self.create_subscription(TofsenseMFrame0, self.topic_in, self.cb, 10)

        self.get_logger().info(
            "tofsensem_fusion (depth_mode=%s): rows=%d cols=%d FOVx=%.1f FOVy=%.1f max_range=%.2f mode=%s",
            self.depth_mode,
            self.rows,
            self.cols,
            self.fov_x_deg,
            self.fov_y_deg,
            self.max_range,
            self.resolution_mode,
        )

    # === LUT 构建（小孔模型） ===
    def _build_camera_lut(self) -> None:
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
            value = int(val)
        except Exception:
            value = 0
        return max(0, min(255, value))

    def _extract_intensity(self, pixel) -> int:
        """从像素中提取强度（若无相关字段则返回 0）。"""
        cand = None
        for name in ("amp", "intensity", "strength", "conf"):
            if hasattr(pixel, name):
                cand = getattr(pixel, name)
                break
        return self._saturate_uint8(cand if cand is not None else 0)

    def _pc2_from_points(self, points):
        """将点列表打包为 PointCloud2。"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.UINT8, 1),
            PointField("return_type", 13, PointField.UINT8, 1),
            PointField("channel", 14, PointField.UINT16, 1),
            PointField("azimuth", 16, PointField.FLOAT32, 1),
            PointField("elevation", 20, PointField.FLOAT32, 1),
            PointField("distance", 24, PointField.FLOAT32, 1),
            PointField("time_stamp", 28, PointField.UINT32, 1),
        ]
        return pc2.create_cloud(header, fields, points)

    def _gridmap_info(self, resolution, len_x, len_y):
        info = GridMapInfo()
        info.header.frame_id = self.frame_id
        info.header.stamp = self.get_clock().now().to_msg()
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
        dim_row = MultiArrayDimension(label="row_index", size=self.rows, stride=self.rows)
        arr.layout.dim = [dim_col, dim_row]
        arr.layout.data_offset = 0
        arr.data = grid_2d.astype(np.float32).ravel(order="C").tolist()
        return arr

    # === 回调 ===
    def cb(self, msg: TofsenseMFrame0) -> None:
        N = min(len(msg.pixels), self.rows * self.cols)

        main_depth = np.full((self.rows, self.cols), np.nan, dtype=np.float32)
        valid_vals = []

        for idx in range(N):
            pixel = msg.pixels[idx]
            r = idx // self.cols
            c = idx % self.cols

            if getattr(pixel, "dis_status", 0) == 0 and math.isfinite(pixel.dis):
                d_m = float(pixel.dis) / 1000.0  # mm -> m
                if self.min_range <= d_m <= self.max_range:
                    if self.use_temporal_median:
                        self.buffers[idx].append(d_m)
                        d_use = float(np.median(self.buffers[idx])) if self.buffers[idx] else d_m
                    else:
                        d_use = d_m

                    main_depth[r, c] = d_use

                    if self.depth_mode == "z":
                        valid_vals.append(d_use)
                    else:
                        valid_vals.append(d_use * self.dz_lut[r, c])

        if self.resolution_mode == "dynamic" and valid_vals:
            D_z = float(np.median(valid_vals))
        else:
            D_z = float(self.nominal_distance)

        W = 2.0 * D_z * math.tan(rad(self.fov_x_deg) / 2.0)
        H = 2.0 * D_z * math.tan(rad(self.fov_y_deg) / 2.0)
        res_x = W / max(self.cols, 1)
        res_y = H / max(self.rows, 1)
        resolution = float((res_x + res_y) / 2.0)

        points = []
        for r in range(self.rows):
            for c in range(self.cols):
                idx = r * self.cols + c
                d = main_depth[r, c]

                if math.isfinite(d):
                    intensity = self._extract_intensity(msg.pixels[idx])
                    return_type = int(getattr(msg.pixels[idx], "return_type", 0))
                    return_type = max(0, min(255, return_type))
                    channel = max(0, min(65535, int(idx)))
                else:
                    intensity = 0
                    return_type = 0
                    channel = 0

                if not math.isfinite(d):
                    if self.drop_invalid:
                        continue
                    points.append([
                        float("nan"),
                        float("nan"),
                        float("nan"),
                        0,
                        0,
                        0,
                        float("nan"),
                        float("nan"),
                        float("nan"),
                        0,
                    ])
                    continue

                if self.depth_mode == "z":
                    x = self.u_lut[r, c] * d
                    y = self.v_lut[r, c] * d
                    z = d
                else:
                    dir_xyz = self.dir_lut[r, c, :]
                    x, y, z = (dir_xyz * d).tolist()

                dist = self._hypot3(x, y, z)
                azimuth = math.atan2(y, x)
                elevation = math.atan2(z, dist) if dist > 0.0 else 0.0
                t_ns = 0

                points.append([x, y, z, intensity, return_type, channel, azimuth, elevation, dist, t_ns])

        cloud = self._pc2_from_points(points)
        self.pub_cloud.publish(cloud)

        if self.depth_mode == "z":
            z_grid = main_depth.astype(np.float32, copy=True)
        else:
            z_grid = (main_depth * self.dz_lut).astype(np.float32)

        grid_map = GridMap()
        grid_map.info = self._gridmap_info(resolution=resolution, len_x=W, len_y=H)
        grid_map.layers = [self.layer_name]
        grid_map.basic_layers = [self.layer_name]
        grid_map.data = [self._pack_layer(z_grid)]
        self.pub_grid.publish(grid_map)


def main() -> None:
    rclpy.init()
    node = TofsenseMFusionNode()
    node.get_logger().info("tofsensem_fusion_node started.")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
