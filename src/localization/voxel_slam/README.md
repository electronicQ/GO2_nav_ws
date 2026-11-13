# voxel_slam（ROS 2 Humble）

本目录是 ROS 2 `voxel_slam` 功能包，使用 `ament_cmake` 构建。

当前分支支持 Voxel-SLAM 建图，以及基于 v4 逐扫描历史地图的持续五帧 SC＋Quatro＋Nano-GICP 定位。定位 TF 为 `map → odom → aft_mapped`。

依赖安装、工作空间编译、传感器配置、地图导出、定位启动、参数和输出接口统一见仓库根目录的 [README](../README.md)。

实现补充说明：

- [双 TF 定位架构](../docs/双TF定位架构.md)
- [定位参数说明](../docs/定位后端配置说明.md)
