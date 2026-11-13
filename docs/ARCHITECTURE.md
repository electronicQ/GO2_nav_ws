# 架构与坐标约定

## 模块边界

- 传感器层只发布 `/imu/data_raw` 和 `/rslidar_points`。
- Voxel-SLAM 负责扫描地图导出、全局重定位、`map→odom` 和高频
  `odom→imu_link`。
- 静态 TF 层根据 `config/calibration/*.yaml` 发布
  `imu_link→base_link` 与 `imu_link→rslidar`。
- Nav2 负责栅格地图、全局规划、局部控制、动态代价地图和恢复行为。
- `go2_twist_bridge` 是唯一允许向宇树运动 API 发布速度请求的节点。

## 标定数据源

`imu_to_lidar` 同时用于：

1. 静态 TF `imu_link→rslidar`；
2. Voxel-SLAM 的 `General.extrinsic_tran`；
3. 由四元数转换得到的 `General.extrinsic_rota`。

`voxel_map_tools_runtime` 的 `process_map.launch.py` 会读取同一
`calibration_file`，并把 `imu_to_lidar.translation` 写入
`dynamic_removal.sensor_origin_imu_m`。

地图包与其建图外参绑定。建图、地图处理和定位统一使用
`current_robot.yaml`；重新标定后必须重新建图。

## 定位门控

`go2_twist_bridge` 同时订阅：

- `/localization/valid`
- `/localization/state`
- `/cmd_vel`

只有以下条件全部满足才转发运动：

- valid 为 true；
- state 为 `IN_MAP_TRACKING` 或 `GLOBAL_TRACKING`；
- valid/state 消息未超时；
- `/cmd_vel` 未超时。

条件不满足时，桥接以节流方式发送平衡站立请求，以停止持续运动。
两个状态都必须同时满足 `/localization/valid=true`，状态消息本身也会做时效检查。

## 运行约束

- `sensor_source=bag` 时禁止启动真实传感器驱动，使用 `/clock` 和模拟时间。
- `control_robot=false` 时不启动宇树控制桥接。
- `control_robot=true` 要求标定文件设置 `robot_motion_authorized: true`。
- 定位、导航同时启动，但应等待定位状态进入 `IN_MAP_TRACKING` 或
  `GLOBAL_TRACKING` 后再下发目标。
