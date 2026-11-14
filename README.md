# GO2 Voxel-SLAM Navigation Workspace

单一 ROS 2 Humble 工作空间，使用外置 IMU、RoboSense RS16、Voxel-SLAM 和
Nav2，只通过 `go2_twist_bridge` 控制宇树 GO2。

主要链路：

```text
/imu/data_raw + /rslidar_points
              ↓
          Voxel-SLAM
              ↓
map → odom → imu_link → base_link
                    └──→ rslidar
              ↓
        Voxel-SLAM localization
              ↓
       map_server + Nav2 + RS16 costmap
              ↓
             /cmd_vel
              ↓
定位有效门控 → go2_twist_bridge → /api/sport/request
```

## 从远程仓库部署

目标机器需要 Ubuntu 22.04 **x86_64**，并已安装 ROS 2 Humble。当前随仓库
提供的 GTSAM、TEASER++ 和 Humble 运行库是 x86_64 构建；如果机器狗端是
ARM64/Jetson，需要先在目标架构重新构建这些第三方依赖。将下面的仓库地址
替换为实际 Git 远程地址：

```bash
git clone <repository-url> "$HOME/GO2_nav_ws"
cd "$HOME/GO2_nav_ws"

./scripts/install_dependencies.sh

./scripts/build.sh

source scripts/setup_env.sh
```

部署脚本会完成：

- 安装编译工具、PCL、Eigen、Boost、yaml-cpp、Python 串口等系统依赖。
- 运行 `rosdep install` 补齐 ROS 2 运行和开发依赖。
- 自动安装 `third_party/debs/<架构>/` 下的闭源 `voxel_map_tools` Debian 包。
- 不在公开源码树中编译闭源执行器；工作空间只编译本地 launch 和参数包。
- 加载随仓库提供的 GTSAM、TEASER++、Nav2、CycloneDDS 等第三方前缀。
- 执行完整 `colcon build`，启用 Scan Context、Quatro 和 Nano-GICP 定位。

工作空间路径不是写死的。`setup_env.sh` 会根据自身位置推导
`GO2_NAV_WORKSPACE`，因此可以 clone 到 `$HOME/GO2_nav_ws`、`/opt` 或其他目录。
克隆后不需要执行 Git submodule 初始化；`rslidar_sdk` 所需的
`src/rs_driver` 源码已经包含在仓库中。

部署完成后检查：

```bash
source scripts/setup_env.sh
ros2 pkg prefix voxel_map_tools
ros2 pkg prefix voxel_map_tools_runtime
ros2 pkg prefix go2_nav_bringup
ros2 pkg prefix voxel_slam
ros2 pkg prefix nav2_bringup
ros2 pkg executables voxel_map_tools
```

仓库包含约 179 MB 示例地图和约 122 MB 第三方运行前缀，首次 clone 会比普通
源码仓库慢。`build/`、`install/`、`log/`、Python 缓存、rosbag 和临时地图
不会进入 Git。

## 私有地图处理包

`voxel_map_tools` 的源码保存在不进入公开仓库的 `private/` 目录中。发行时只交付
安装到 `/opt/ros/humble` 的 `ros-humble-voxel-map-tools` Debian 包。包内包含
stripped C++ 可执行文件、launch 文件、默认参数和 ROS 2 包索引，不包含源码、
内部静态库、头文件、CMake 开发接口或编译缓存。

内部维护者在打包机上生成发行包：

```bash
./scripts/package_voxel_map_tools_deb.sh
```

默认读取 `private/voxel_map_tools_src`，输出到
`third_party/debs/<目标架构>/`。生成后应同时保存 `.deb` 和 `.sha256`，将二者
放入 Git，或通过实验室私有文件服务器、私有制品库或对象存储同步。部署脚本会
根据当前主机架构选择对应的 `.deb` 并安装：

```bash
./scripts/install_dependencies.sh
ros2 pkg executables voxel_map_tools
```

如果 `.deb` 位于其他目录，可显式指定：

```bash
VOXEL_MAP_TOOLS_DEB=/absolute/path/to/package.deb \
  ./scripts/install_dependencies.sh
```

同一个版本号重新打包后需要强制重装时使用
`VOXEL_MAP_TOOLS_FORCE_REINSTALL=1`。`amd64` 与 ARM64 的 `.deb` 不能混用，
每种目标架构应在对应架构上重新打包。

客户端的 launch 和参数位于：

```text
src/localization/voxel_map_tools_runtime/launch/process_map.launch.py
src/localization/voxel_map_tools_runtime/config/default.yaml
```

默认 launch 直接读取工作空间中的 YAML，因此修改参数后只需重新执行 launch，
不需要重新编译闭源包。安装到 `/opt/ros/humble` 的二进制包仍保留自身默认配置，
但正常工作时不要修改 `/opt` 下的文件。

## 构建

正常部署直接使用：

```bash
cd "$HOME/GO2_nav_ws"
./scripts/install_dependencies.sh
./scripts/build.sh
source scripts/setup_env.sh
```

部署机资源有限，或完整构建因内存压力卡住时，建议使用以下低资源构建方式。
它不会编译测试目标，并限制 CMake 的并行编译进程数：

```bash
cd "$HOME/GO2_nav_ws"

set +u
export GO2_SKIP_WORKSPACE_INSTALL=1
source scripts/setup_env.sh
unset GO2_SKIP_WORKSPACE_INSTALL
set -u

export CMAKE_BUILD_PARALLEL_LEVEL=2

colcon build \
  --executor sequential \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF

source scripts/setup_env.sh
```

使用该方式前应确认第三方前缀和闭源地图处理包已经安装：

```bash
test -d "$HOME/GO2_nav_ws/third_party/install/gtsam"
test -d "$HOME/GO2_nav_ws/third_party/install/teaserpp"
test -d "$HOME/GO2_nav_ws/third_party/ros-humble"
ros2 pkg prefix voxel_map_tools
```

最后一条命令应返回 `/opt/ros/humble`。如果内存仍然不足，可将
`CMAKE_BUILD_PARALLEL_LEVEL` 降到 `1`。中止后的构建可以重新执行，并利用已有
CMake 缓存继续编译。

该低资源命令适用于部署机。开发机需要编译和执行回归测试时，应继续使用
`./scripts/build.sh`，或者显式设置 `BUILD_TESTING=ON`。

构建日志必须包含：

```text
Scan Context + Quatro + Nano-GICP relocalization enabled
```

`third_party/install/gtsam` 和 `third_party/install/teaserpp` 已随仓库提供。
`setup_env.sh` 使用当前工作空间路径设置 `CMAKE_PREFIX_PATH` 和
`LD_LIBRARY_PATH`，不要依赖旧机器上的绝对路径。

宇树消息包编译需要 `rosidl_generator_dds_idl`。工作空间已把 Humble 版本的
deb 内容展开到 `third_party/ros-humble`。同一前缀还包含 CycloneDDS、
Iceoryx、Nav2 运行包和地图服务器所需的 GraphicsMagick，`scripts/setup_env.sh`
会自动加载；如果重新制作工作空间，应再次放入这些依赖。

RS16 驱动当前关闭 PCAP 文件回放支持，因为本项目只需要在线雷达和 ROS 2 bag。
如需直接读取 PCAP，可安装 `libpcap-dev`，并从 `colcon.meta` 删除
`DISABLE_PCAP_PARSE`。

## 环境

建议在每个新终端、且当前目录为工作空间根目录时使用：

```bash
source scripts/setup_env.sh
```

`setup_env.sh` 使用 `${ROS_DOMAIN_ID:-0}`。默认 Domain 0 可以正常通信；
只有需要隔离其他 ROS 2 进程时才设置独立 Domain。该变量需要在 source 之前
设置，例如：

```bash
export ROS_DOMAIN_ID=77
source scripts/setup_env.sh
```

连接实机时指定接 GO2 的有线网卡：

```bash
export GO2_NETWORK_INTERFACE=enp4s0
source scripts/setup_env.sh
```

不需要固定网卡时可以只设置 `RMW_IMPLEMENTATION`，或者不设置
`GO2_NETWORK_INTERFACE`。

## 实时建图

先确认 RS16 和外置 IMU 已连接。建图阶段不会启动任何 GO2 本体节点：

```bash
ros2 launch go2_nav_bringup mapping.launch.py \
  sensor_source:=live \
  map_name:=my_scene_sc_map \
  calibration_file:=$GO2_NAV_WORKSPACE/config/calibration/current_robot.yaml
```

Voxel-SLAM 的旧格式兼容目录由 launch 固定到工作空间下的
`$GO2_NAV_WORKSPACE/.mapping_legacy/`，每次建图创建独立子目录，避免复用
`/tmp/voxel_slam_legacy` 时因父目录或旧目录存在而创建失败。可通过
`legacy_save_root:=/其他可写目录` 覆盖根目录。该目录只用于旧逻辑兼容，
最终 v4 地图仍保存在 `$GO2_NAV_WORKSPACE/maps/<map_name>`。

需要控制机器狗行走时使用遥控器。建图结束后，在另一个终端选择结束方式。

执行最终全局 BA：

```bash
source scripts/setup_env.sh
"$GO2_NAV_WORKSPACE/scripts/finish_map.sh" finish
```

跳过最终全局 BA，直接使用当前优化位姿导出地图：

```bash
source scripts/setup_env.sh
"$GO2_NAV_WORKSPACE/scripts/finish_map.sh" finish_no_ba
```

不传参数时默认等同于 `finish`，即执行最终全局 BA：

```bash
source scripts/setup_env.sh
"$GO2_NAV_WORKSPACE/scripts/finish_map.sh"
```

必须等待节点输出：

```text
Saved optimized SC scan map to ...
```

日志末尾会分别显示：

```text
final_global_ba=enabled
```

或者：

```text
final_global_ba=skipped
```

`finish` 会使用 `ros2_rslidar_slam.yaml` 中的 `GBA.*` 参数执行最终全局
BA；`finish_no_ba` 会跳过这一步。不要把 YAML 中的 `finish_no_ba` 长期设为
`true`，应在建图结束时通过 `finish_map.sh finish_no_ba` 一次性选择。

不要用 Ctrl+C 代替导出。

## Bag 建图

默认 bag 为
`/home/gzq/voxel_slam_ws/bag/ros2bag/zoulang1111`。输出目录必须尚不存在：

```bash
ros2 launch go2_nav_bringup mapping.launch.py \
  sensor_source:=bag \
  bag_path:=/home/gzq/voxel_slam_ws/bag/ros2bag/zoulang1111 \
  map_name:=zoulang1111_sc_map_rerun \
  auto_finish:=true
```

bag 结束后等待 5 秒，launch 会调用：

```bash
ros2 param set /voxelslam finish true
```

`auto_finish_mode` 用于选择自动结束时是否执行最终全局 BA，默认值是
`finish`。如果需要在 bag 播放完成后自动跳过全局 BA：

```bash
ros2 launch go2_nav_bringup mapping.launch.py \
  sensor_source:=bag \
  bag_path:=/home/gzq/voxel_slam_ws/bag/ros2bag/zoulang1111 \
  map_name:=zoulang1111_sc_map_rerun \
  auto_finish:=true \
  auto_finish_mode:=finish_no_ba
```

bag 结束后等待 5 秒，launch 会调用：

```bash
ros2 param set /voxelslam finish_no_ba true
```

两种模式默认都在 bag 播放进程退出后等待 `5.0` 秒。如需临时调整，可传入
`finish_delay_sec:=<秒数>`。

两种模式：

| `auto_finish_mode` | 自动结束行为 |
|---|---|
| `finish` | 执行最终全局 BA，默认值 |
| `finish_no_ba` | 跳过最终全局 BA，使用当前优化位姿导出地图 |

`auto_finish:=false` 时 `auto_finish_mode` 不触发自动结束，仍可通过
`scripts/finish_map.sh` 手动选择。

## 地图处理

处理后的输出位于地图包的 `processed/`：

```bash
ros2 launch voxel_map_tools_runtime process_map.launch.py \
  input_map:=$GO2_NAV_WORKSPACE/maps/my_scene_sc_map \
  overwrite:=true
```

已有 `processed/` 时必须显式 `overwrite:=true`，工具只替换它管理的输出文件。
`calibration_file` 会把 `imu_to_lidar.translation` 同步到动态点剔除所需的
`sensor_origin_imu_m`，默认读取
`$GO2_NAV_WORKSPACE/config/calibration/current_robot.yaml`。

## 定位与导航

### （1）bag包：IMU/雷达bag包测试定位与导航（不控制 GO2）

```bash
source scripts/setup_env.sh

ros2 launch go2_nav_bringup localization_nav.launch.py \
  sensor_source:=bag \
  bag_path:=/home/gzq/voxel_slam_ws/bag/ros2bag/zoulang1111 \
  map_path:=$GO2_NAV_WORKSPACE/maps/zoulang1111_sc_map \
  control_robot:=false
```

如需隔离实机或其他测试进程，可在 `source` 前可选执行：

```bash
export ROS_DOMAIN_ID=77
```

bag 模式会在 launch 内切换到 Fast DDS 并启用 localhost-only，避免外部
GO2 网卡和多网卡发现干扰。单独执行 `ros2 topic` 检查时应使用相同环境：

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
source scripts/setup_env.sh
```

如果启动实验时设置了独立 Domain，检查终端也必须设置相同的
`ROS_DOMAIN_ID`；全部使用默认 Domain 0 时不需要设置。

### （2）实机：仅外置 IMU/雷达测试定位与导航（不控制 GO2）

不启动 GO2 速度桥时，可以只使用外置 IMU、激光雷达和历史地图测试定位、
全局/局部代价地图、Nav2 规划与 RViz 显示。机器狗可以不上电，也可以由原厂
遥控器手动移动；本工作空间不会向宇树运动 API 发送自动控制请求。

先清除可能残留的 GO2 DDS 网卡绑定：

```bash
cd "$HOME/GO2_nav_ws"

unset GO2_NETWORK_INTERFACE
unset CYCLONEDDS_URI
source scripts/setup_env.sh

ros2 launch go2_nav_bringup localization_nav.launch.py \
  sensor_source:=live \
  map_path:=$GO2_NAV_WORKSPACE/maps/my_scene_sc_map \
  calibration_file:=$GO2_NAV_WORKSPACE/config/calibration/current_robot.yaml \
  control_robot:=false \
  rviz:=true
```

该模式仍会启动 IMU、雷达、Voxel-SLAM、`map_server`、Nav2、全局/局部代价
地图和 RViz，但不会启动 `go2_twist_bridge`。Nav2 发布的 `/cmd_vel` 不会转发
到真实机器狗。需要观察动态定位效果时，可用原厂遥控器手动缓慢移动机器人。

无机器狗、只供电给外置传感器时也可以启动并观察静态定位、地图、TF 和代价
地图，但由于没有实际运动，无法验证运动过程中的定位连续性。

确认局部和全局定位状态：

```bash
ros2 topic echo /localization/state
ros2 topic echo /localization/local_valid
ros2 topic echo /localization/valid
```

定位有效后检查完整 TF：

```bash
ros2 run tf2_ros tf2_echo odom imu_link
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_link
```

状态进入 `GLOBAL_TRACKING` 且 `/localization/valid=true` 后，可在 RViz 中
发送 `Nav2 Goal`。应能看到栅格地图、膨胀层、全局/局部代价地图、
`/plan` 和 `/local_plan`、历史地图点云、实时点云和当前扫描。

如果只有 `LOCAL_ONLY` 或缺少 `map→odom`，说明局部 LIO 正常但全局重定位
尚未通过，此时应检查历史地图、实时点云和 `current_robot.yaml` 外参是否
来自同一套建图配置，而不是仅把 `control_robot` 改为 `true`。

### （3）实机：机器狗/IMU/雷达全部开启测试定位与导航（控制 GO2）

```bash
export GO2_NETWORK_INTERFACE=enp4s0
source scripts/setup_env.sh
ros2 launch go2_nav_bringup localization_nav.launch.py \
  sensor_source:=live \
  map_path:=$GO2_NAV_WORKSPACE/maps/my_scene_sc_map \
  calibration_file:=$GO2_NAV_WORKSPACE/config/calibration/current_robot.yaml \
  control_robot:=true
```

实测外参和 `robot_motion_authorized: true` 未完成前，实机控制会被 launch
拒绝。这是防止错误 TF 驱动真实机器狗的安全限制。

等待：

```bash
ros2 topic echo /localization/state
ros2 topic echo /localization/valid
```

状态进入 `IN_MAP_TRACKING` 或 `GLOBAL_TRACKING`，且 `valid=true` 后，
在 RViz 使用 `Nav2 Goal`
下发目标。建图/定位链固定 TF 为：

```text
map → odom → imu_link → base_link
                 └─────→ rslidar
```

Nav2 的 `robot_base_frame` 是 `base_link`，不发布旧
`odom→base_footprint`。不要同时启动旧 `go2_driver`，否则会产生重复 TF。

## 参数配置

只修改 `src/`、`config/`、`scripts/` 下的源码和配置文件，不要直接修改
`install/` 中的生成文件。本工作空间默认使用 `--symlink-install`，YAML 和
launch 修改通常会在节点或 launch 重启后生效；C++ 源码修改后仍需重新构建对应包。

### 各步骤参数文件快速索引

| 功能或步骤 | 参数文件 | 主要配置内容 |
|---|---|---|
| ROS Domain、RMW、GO2 网卡 | [scripts/setup_env.sh](scripts/setup_env.sh) | `ROS_DOMAIN_ID` 默认值、`RMW_IMPLEMENTATION`、`GO2_NETWORK_INTERFACE`、工作空间加载 |
| 构建依赖 | [colcon_defaults.yaml](colcon_defaults.yaml) | 构建类型、`--symlink-install`、源码路径 |
| 包级构建选项 | [colcon.meta](colcon.meta) | GTSAM、TEASER++、SC-QN、RS16 PCAP 开关 |
| 外置 IMU | [mapping.launch.py](src/integration/go2_nav_bringup/launch/mapping.launch.py) 和 [localization_nav.launch.py](src/integration/go2_nav_bringup/launch/localization_nav.launch.py) | `sensor_serial_port`、`sensor_serial_baud` |
| 外置 RS16 雷达 | [config.yaml](src/sensors/rslidar_sdk/config/config.yaml) | 雷达型号、MSOP/DIFOP 端口、雷达帧、点云话题、时钟来源、距离范围 |
| IMU、雷达、`base_link` 外参 | [current_robot.yaml](config/calibration/current_robot.yaml) | `imu_to_base`、`imu_to_lidar`、标定和实机控制授权 |
| 实时或 bag 建图入口 | [mapping.launch.py](src/integration/go2_nav_bringup/launch/mapping.launch.py) | `sensor_source`、`bag_path`、`map_name`、`auto_finish`、`auto_finish_mode`、`finish_delay_sec`、`legacy_save_root`、`use_sim_time`、`rviz` |
| Voxel-SLAM 建图算法 | [ros2_rslidar_slam.yaml](src/localization/voxel_slam/config/ros2_rslidar_slam.yaml) | 建图话题、IMU/雷达外参、LIO、局部 BA、回环、全局 BA、地图导出 |
| 地图处理 | [default.yaml](src/localization/voxel_map_tools_runtime/config/default.yaml) | 输入地图、地面拟合、障碍高度、栅格分辨率、动态点剔除 |
| 地图处理入口 | [process_map.launch.py](src/localization/voxel_map_tools_runtime/launch/process_map.launch.py) | `input_map`、`output_dir`、`calibration_file`、`overwrite` |
| 定位入口 | [localization_nav.launch.py](src/integration/go2_nav_bringup/launch/localization_nav.launch.py) | 地图、bag、标定、RViz、是否控制 GO2、传感器输入 |
| Voxel-SLAM 输入和 LIO | [ros2_localization_rslidar.yaml](src/localization/voxel_slam/config/ros2_localization_rslidar.yaml) | 定位话题、雷达/IMU 外参、LIO 噪声、局部位姿输出 |
| 定位有效期和双 TF | [ros2_localization_common.yaml](src/localization/voxel_slam/config/ros2_localization_common.yaml) | `map→odom`、修正时效、失效门限、状态检查 |
| 全局重定位 | [ros2_scancontext_rslidar_relocalization.yaml](src/localization/voxel_slam/config/ros2_scancontext_rslidar_relocalization.yaml) | Scan Context、Quatro、Nano-GICP、候选数和配准门限 |
| Nav2 全局与局部导航 | [nav2_params.yaml](src/navigation/go2_navigation2/config/nav2_params.yaml) | BT Navigator、DWB、NavFn、代价地图、膨胀、恢复行为和速度平滑 |
| GO2 控制门控 | [localization_nav.launch.py](src/integration/go2_nav_bringup/launch/localization_nav.launch.py) 和 [twist_bridge.cpp](src/robot/go2_twist_bridge/src/twist_bridge.cpp) | `valid`、允许状态、定位/速度超时、停止请求 |
| RViz 显示 | [localization_nav.rviz](src/integration/go2_nav_bringup/config/rviz/localization_nav.rviz) | 地图、代价地图、TF、历史/实时点云、全局/局部路径 |

### 传感器参数

RS16 的实际驱动参数在
[src/sensors/rslidar_sdk/config/config.yaml](src/sensors/rslidar_sdk/config/config.yaml)：

```yaml
lidar_type: RS16
msop_port: 6699
difop_port: 7788
use_lidar_clock: false
ros_frame_id: rslidar
ros_send_point_cloud_topic: /rslidar_points
```

- 更换雷达型号或固件时修改 `lidar_type`。
- 雷达和主机不在同一时钟源时保持 `use_lidar_clock: false`。
- 点云帧名必须继续与标定文件中的 `lidar_frame` 一致。
- 修改端口后，主机防火墙和网卡配置也要同步调整。

外置 IMU 的串口和波特率通过 launch 参数配置：

```bash
ros2 launch go2_nav_bringup mapping.launch.py \
  sensor_source:=live \
  sensor_serial_port:=/dev/imu_usb \
  sensor_serial_baud:=115200
```

如需修改默认串口、波特率或帧名，编辑
[launch_utils.py](src/integration/go2_nav_bringup/go2_nav_bringup/launch_utils.py)
中的 `sensor_nodes()`。IMU 数据解析、单位和比例在
[wit_ros2_imu.py](src/sensors/wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py)。

### 标定参数

当前实机的唯一标定入口是
[config/calibration/current_robot.yaml](config/calibration/current_robot.yaml)：

```yaml
imu_frame: imu_link
base_frame: base_link
lidar_frame: rslidar
calibration_ready: true
robot_motion_authorized: true

imu_to_base:
  translation: [x, y, z]
  rotation_xyzw: [qx, qy, qz, qw]

imu_to_lidar:
  translation: [x, y, z]
  rotation_xyzw: [qx, qy, qz, qw]
```

`imu_to_lidar` 会同时用于：

- 静态 TF `imu_link→rslidar`；
- Voxel-SLAM 的 `General.extrinsic_tran` 和 `General.extrinsic_rota`；
- 地图处理动态剔除的 `sensor_origin_imu_m`。

建图和定位必须使用同一份外参。修改外参后，旧地图不能直接与新外参混用，
需要重新建图。实机控制前才把 `robot_motion_authorized` 改为 `true`。

### 建图和地图处理参数

建图阶段最常修改的是
[ros2_rslidar_slam.yaml](src/localization/voxel_slam/config/ros2_rslidar_slam.yaml)：

| 参数 | 作用 |
|---|---|
| `General.lid_topic` | 雷达输入话题，当前为 `/rslidar_points` |
| `General.imu_topic` | IMU 输入话题，当前为 `/imu/data_raw` |
| `General.lidar_type` | Voxel-SLAM 的雷达解析类型 |
| `General.blind` | 雷达盲区过滤 |
| `General.map_scan_leaf_size_m` | 保存逐帧地图时的降采样边长 |
| `Odometry.*` | LIO 噪声、预测时间和局部位姿坐标系 |
| `LocalBA.*` | 局部 BA 窗口和优化参数 |
| `Loop.*` | 回环检测和 ICP 参数 |
| `GBA.*` | 最终全局 BA 参数 |

通过顶层 launch 的 `map_name` 指定输出目录更安全，例如
`map_name:=my_scene_sc_map` 会输出到
`$GO2_NAV_WORKSPACE/maps/my_scene_sc_map`。直接改 YAML 中的
`General.map_export_path` 也可，但目录必须尚不存在。

地图处理参数在
[voxel_map_tools_runtime/config/default.yaml](src/localization/voxel_map_tools_runtime/config/default.yaml)。
直接修改该工作空间文件并重新执行
`ros2 launch voxel_map_tools_runtime process_map.launch.py` 即可生效：

| 参数组 | 主要作用 |
|---|---|
| `input_map`、`output_dir` | 输入 v4 地图和输出目录 |
| `outputs.*` | 是否输出完整点云、去地面点云、障碍点云和栅格地图 |
| `ground.*` | 地面拟合、坡度、连续性和地面高度容差 |
| `obstacles.min_height_m/max_height_m` | 进入 Nav2 栅格障碍层的地面以上高度 |
| `grid.resolution_m` | Nav2 地图分辨率 |
| `dynamic_removal.*` | 动态点剔除门限 |

`dynamic_removal.sensor_origin_imu_m` 默认由 launch 的 `calibration_file`
覆盖，以保证与建图标定一致。如果确实要以 YAML 中的值为准，可传
`calibration_file:=''`。

### 定位参数

定位传感器和 LIO 参数在
[ros2_localization_rslidar.yaml](src/localization/voxel_slam/config/ros2_localization_rslidar.yaml)：

- `General.lid_topic`、`General.imu_topic`：输入话题。
- `General.extrinsic_*`：由标定文件覆盖，不建议在 YAML 中单独改。
- `Odometry.*`：局部里程计、IMU 高频预测和坐标系。
- `Localization.visualization.*`：历史地图显示点云。

定位稳定性和时效参数在
[ros2_localization_common.yaml](src/localization/voxel_slam/config/ros2_localization_common.yaml)；
Scan Context 重定位在
[ros2_scancontext_rslidar_relocalization.yaml](src/localization/voxel_slam/config/ros2_scancontext_rslidar_relocalization.yaml)。

常见定位调参：

- 初次重定位慢：检查 `scancontext.query_scans`、`candidates` 和 `distance_threshold`。
- 定位修正频繁跳变：减小 `max_correction_translation_m` 和
  `max_correction_rotation_deg`。
- `GLOBAL_STALE` 出现频繁：检查 `max_result_age_sec`、`stale_after_sec`、
  `global_valid_timeout_sec` 以及实时输入频率。
- 始终只有局部里程计：检查地图路径、输入点云格式、标定一致性和 SC-QN 编译日志。

### Nav2 参数

Nav2 的所有运行参数集中在
[go2_navigation2/config/nav2_params.yaml](src/navigation/go2_navigation2/config/nav2_params.yaml)。

常用功能对应位置：

| 需求 | YAML 节点或参数 |
|---|---|
| 最大前进速度 | `controller_server.FollowPath.max_speed_xy` |
| 最大转向速度 | `controller_server.FollowPath.max_vel_theta` |
| 机器人矩形足迹 | `local_costmap` 和 `global_costmap` 的 `footprint` |
| 膨胀层距离 | 两个代价地图的 `inflation_radius`、`cost_scaling_factor` |
| 局部障碍点云 | `local_costmap.voxel_layer.rslidar` |
| 全局障碍点云 | `global_costmap.obstacle_layer.rslidar` |
| 障碍高度范围 | `min_obstacle_height`、`max_obstacle_height` |
| 障碍/清除距离 | `obstacle_max_range`、`raytrace_max_range` |
| 全局规划器 | `planner_server.GridBased.plugin` |
| 局部控制器 | `controller_server.FollowPath.plugin` |
| 目标位置和朝向容差 | `controller_server.general_goal_checker.xy_goal_tolerance/yaw_goal_tolerance` |
| 速度平滑频率 | `velocity_smoother.smoothing_frequency` |
| 速度平滑反馈模式 | `velocity_smoother.feedback` |

当前参数配置：

- 最大合成线速度 `0.35 m/s`。
- 全局规划器为 `NavfnPlanner`。
- 局部控制器为 `DWBLocalPlanner`。
- 局部代价地图尺寸 `4 m × 4 m`。
- 障碍输入为 `/rslidar_points` 的 `PointCloud2`。
- 速度平滑使用 `CLOSED_LOOP`，里程计为 `/LIO/odom_imu`。

实机首次测试建议先临时把 `max_speed_xy` 降到约 `0.15～0.20 m/s`。修改 YAML
后重启导航 launch；运行时临时查看和设置参数可使用：

```bash
ros2 param get /controller_server FollowPath.max_speed_xy
ros2 param set /controller_server FollowPath.max_speed_xy 0.2
```

运行时 `ros2 param set` 不一定永久，正式值应写回源码 YAML。

### GO2 控制桥接和 RViz

控制门控参数位于
[localization_nav.launch.py](src/integration/go2_nav_bringup/launch/localization_nav.launch.py)：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `require_localization_valid` | `true` | 定位无效时不转发运动 |
| `required_localization_states` | `IN_MAP_TRACKING,GLOBAL_TRACKING` | 允许控制的定位状态 |
| `localization_timeout_sec` | `1.0` | 定位状态消息允许的最大年龄 |
| `cmd_vel_timeout_sec` | `0.5` | `/cmd_vel` 超时时间 |

桥接逻辑和停止请求在
[twist_bridge.cpp](src/robot/go2_twist_bridge/src/twist_bridge.cpp)。
宇树 API ID 定义在
[sport_model.hpp](src/robot/go2_twist_bridge/include/sport_model.hpp)，
除非明确了解宇树运动接口，否则不要随意修改。

RViz 的地图、代价地图、点云、TF 和路径显示都在
[localization_nav.rviz](src/integration/go2_nav_bringup/config/rviz/localization_nav.rviz)：

- `Nav2 Global Plan` 显示 `/plan`。
- `Nav2 Local Plan` 显示 `/local_plan`。
- `Historical Map` 显示 `/localization/map_cloud`。
- `Live RS16 Point Cloud` 显示 `/rslidar_points`。
- `Current Scan` 显示 Voxel-SLAM 的 `/map_scan`。

修改 `.rviz` 后需要在 RViz 中重新打开配置，或重启定位导航 launch。

## 目录

- `src/sensors`：外置 IMU、RS16 驱动及雷达消息。
- `src/localization`：Voxel-SLAM、地图处理和 RViz 点云插件。
- `src/robot`：宇树消息接口和带定位门控的速度桥接。
- `src/navigation`：Nav2 参数及独立 Nav2 入口。
- `src/integration/go2_nav_bringup`：实时/bag 建图、定位导航统一入口。
- `maps`：v4 扫描地图和 `processed` Nav2 栅格地图。
- `config/calibration`：静态 TF 与 Voxel-SLAM 外参的唯一配置源。
- `third_party/install`：本工作空间使用的 GTSAM、TEASER++ 前缀。
