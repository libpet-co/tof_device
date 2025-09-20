# tof_device

一个基于 ROS 2 的 ToF 设备包，重点提供 Nooploop TOFSense‑M 的串口驱动与点云/栅格图融合节点（nlink_parser + tofsensem_fusion）。已移除旧版 ROS 1/catkin 相关说明，保留最小且实用的使用指南。

**主要功能**
- 串口解析 TOFSense‑M 的 NLink 协议，发布 `nlink_parser/TofsenseMFrame0`。
- Python 融合节点将深度阵列转为 `sensor_msgs/PointCloud2` 与 `grid_map_msgs/GridMap`。
- 提供可配置的视场角、尺寸、深度定义、滤波与分辨率策略。

**环境要求**
- ROS 2（建议 Humble 及以上）
- 已安装 `rclcpp`、`std_msgs`、`geometry_msgs`、`nav_msgs`、`grid_map_msgs`
- 具备串口读写权限（将当前用户加入 `dialout` 组，或按需设置 udev 规则）

**快速开始**
- 构建
  - 在 ROS 2 工作空间下将本仓库放入 `src/`：
    - `cd <your_ros2_ws>`
    - `colcon build --symlink-install`
    - `source install/setup.bash`
- 运行（驱动 + 融合，推荐）
  - `ros2 launch nlink_parser tofsensem.launch.py`
  - 如需修改串口，直接运行可执行体并传参：
    - `ros2 run nlink_parser tofsensem --ros-args -p port_name:=/dev/ttyUSB0 -p baud_rate:=921600`
    - 另开终端启动融合：`ros2 launch nlink_parser tofsensem_fusion.launch.py`

**话题与消息**
- 驱动节点（C++ 可执行体 `tofsensem`）
  - 发布：`/nlink_tofsensem_frame0`（`nlink_parser/TofsenseMFrame0`）
    - 字段：`id`、`system_time`、`pixel_count`、`pixels[]`（每像素含 `dis`[m]、`dis_status`、`signal_strength`）
- 融合节点（Python `tofsensem_fusion_node.py`）
  - 订阅：`/nlink_tofsensem_frame0`（可通过参数改名）
  - 发布：
    - `/<prefix>/points`（默认 `/tofsensem/points`，`sensor_msgs/PointCloud2`）
      - 字段顺序与类型：`x,y,z`(float32), `intensity`(uint8), `return_type`(uint8), `channel`(uint16), `azimuth,elevation,distance`(float32), `time_stamp`(uint32)，`point_step=32`。
    - `/<prefix>/grid_map`（默认 `/tofsensem/grid_map`，`grid_map_msgs/GridMap`）
      - `layers=[elevation]`，值为深度定义对应的 Z 高程（单位 m）。

**融合节点参数（常用）**
- 数据接口与坐标：`frame_id`(默认 `tofsensem_optical_frame`)、`topic_in`、`topic_pointcloud`、`topic_gridmap`
- 阵列尺寸：`rows`(默认 8)、`cols`(默认 8)
- 视场角：`fov_x_deg`、`fov_y_deg`（`config/tofsensem_fusion.yaml` 默认 45°）
- 深度定义：`depth_mode`=`z`|`los`（Z 深度/沿射线距离）
- 有效范围与滤波：`min_range`(m, 默认 0.02)、`max_range`(m, 默认 2.0)、`drop_invalid`、`use_temporal_median`、`filter_window`
- 高程图分辨率：`resolution_mode`=`fixed`|`dynamic`、`nominal_distance`(m)、`latch_grid`
- 调试：`debug`、`debug_port`

使用配置文件：`ros2 launch nlink_parser tofsensem_fusion.launch.py` 将自动加载 `config/tofsensem_fusion.yaml`，可按需修改上述参数。

**常见问题**
- 串口权限：执行 `sudo usermod -a -G dialout $USER` 后重新登录，或为设备添加 udev 规则。
- 话题名不一致：驱动默认发布 `nlink_tofsensem_frame0`（无前导 `/`），融合默认订阅 `/nlink_tofsensem_frame0`，ROS 2 会解析为同一全局话题；如有自定义命名，请同步修改参数。

**许可证**
- 本包遵循 BSD-3-Clause 许可。详见 `tof_device/src/nlink_parser/LICENSE`。
