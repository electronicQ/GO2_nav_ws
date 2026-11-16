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

## 各步骤参数配置

下面按前面的三个启动功能列出实际使用的配置。只修改 `src/`、`config/` 下的
源文件，不直接修改 `install/` 中的生成文件。默认使用 `--symlink-install`，
已有 YAML 和 launch 修改后需要重启对应功能；C++ 修改后需要重新构建对应包。
同一参数出现多次时，后加载的值覆盖前面的值。

### （1） 建图：`go2_nav_bringup mapping.launch.py`

对应前面的实时建图和 Bag 建图命令，例如：

```bash
ros2 launch go2_nav_bringup mapping.launch.py \
  sensor_source:=live \
  map_name:=my_scene_sc_map \
  calibration_file:=$GO2_NAV_WORKSPACE/config/calibration/current_robot.yaml
```

实时和 bag 模式使用同一份建图算法配置；`sensor_source:=bag` 时从
`bag_path` 回放传感器数据，不启动实时雷达和 IMU 驱动。

| 配置文件 | 配置内容 | 如何选择或覆盖 |
|---|---|---|
| [ros2_rslidar_slam.yaml](src/localization/voxel_slam/config/ros2_rslidar_slam.yaml) | 雷达解析、LIO、局部 BA、回环、全局 BA、逐帧地图导出 | `voxel_config:=<文件路径>` |
| [current_robot.yaml](config/calibration/current_robot.yaml) | IMU 到雷达、IMU 到车体的外参和坐标系名 | `calibration_file:=<文件路径>`；默认 `auto` 使用此文件 |
| [雷达 config.yaml](src/sensors/rslidar_sdk/config/config.yaml) | 实时雷达型号、端口、距离过滤、时间戳和点云话题 | `lidar_config:=<文件路径>`；仅 live 模式启动驱动时使用 |
| [mapping.rviz](src/integration/go2_nav_bringup/config/rviz/mapping.rviz) | 建图 RViz 显示 | 此入口固定使用该文件；`rviz:=false` 关闭 RViz |
| [mapping.launch.py](src/integration/go2_nav_bringup/launch/mapping.launch.py) | 输入模式、bag、输出地图名、结束方式、串口、仿真时间 | 启动命令的同名参数覆盖默认值 |

建图节点先加载 `ros2_rslidar_slam.yaml`，再由 launch 覆盖运行模式、仿真时间、
地图输出路径、旧格式保存目录和雷达外参。**外参以标定文件为准，地图输出以
`map_name` 为准**；在算法 YAML 中修改这些被覆盖的字段不会改变该入口的最终值。

| 常用参数或参数组 | 修改位置与作用 |
|---|---|
| `General.lid_topic`、`General.imu_topic` | 建图算法 YAML：输入话题 |
| `General.lidar_type`、`General.blind` | 建图算法 YAML：点云解析类型和盲区过滤 |
| `General.map_scan_leaf_size_m` | 建图算法 YAML：保存逐帧地图时的降采样边长 |
| `Odometry.*` | 建图算法 YAML：LIO 噪声、IMU 预测时间和位姿坐标系 |
| `LocalBA.*`、`Loop.*`、`GBA.*` | 建图算法 YAML：局部优化、回环和最终全局优化 |
| `map_name` | 启动参数：输出到 `$GO2_NAV_WORKSPACE/maps/<map_name>`；目录必须尚不存在 |
| `auto_finish`、`auto_finish_mode`、`finish_delay_sec` | 启动参数：bag 结束后是否自动导出、是否执行最终全局 BA、等待时间 |
| `legacy_save_root` | 启动参数：旧格式中间文件的保存根目录 |
| `bag_path`、`bag_rate`、`bag_start_paused` | 启动参数：bag 路径、回放速度和是否暂停开始 |

#### 建图与定位共用的传感器、标定和环境

实时建图和实时定位使用同一份雷达驱动配置，当前
[config.yaml](src/sensors/rslidar_sdk/config/config.yaml) 中型号为
`RSHELIOS_16P`，MSOP/DIFOP 端口为 `6699/7788`，`min_distance=1`，
`use_lidar_clock=false`，点云话题为 `/rslidar_points`。更换型号时修改驱动的
`lidar_type`；它与 Voxel-SLAM 算法 YAML 中的数字解析类型属于不同配置项。
点云帧名应与标定文件的 `lidar_frame` 一致。

外置 IMU 的串口和波特率通过两个集成 launch 的
`sensor_serial_port`、`sensor_serial_baud` 指定，例如
`sensor_serial_port:=/dev/imu_usb sensor_serial_baud:=115200`。
驱动启动参数见 [launch_utils.py](src/integration/go2_nav_bringup/go2_nav_bringup/launch_utils.py)
的 `sensor_nodes()`；数据解析、单位和比例见
[wit_ros2_imu.py](src/sensors/wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py)。

三个功能默认都读取 [current_robot.yaml](config/calibration/current_robot.yaml)：

| 标定字段 | 使用位置 |
|---|---|
| `imu_to_lidar.translation`、`rotation_xyzw` | 建图和定位的 `General.extrinsic_tran`、`General.extrinsic_rota`；定位静态 TF `imu_link→rslidar` |
| `imu_to_base`、`imu_frame`、`base_frame`、`lidar_frame` | 定位导航静态 TF 和手动初始位姿的车体参考系 |
| `imu_to_lidar.translation` | 地图处理的 `dynamic_removal.sensor_origin_imu_m` |
| `calibration_ready`、`robot_motion_authorized` | 标定就绪和实机运动授权检查 |

建图和定位应使用一致的外参，修改后不能直接与旧标定生成的地图混用。
ROS Domain、RMW、GO2 网卡和工作空间环境统一在
[scripts/setup_env.sh](scripts/setup_env.sh) 中加载；网卡通过
`GO2_NETWORK_INTERFACE` 指定。

### （2） 地图处理：`voxel_map_tools_runtime process_map.launch.py`

对应前面的地图处理命令：

```bash
ros2 launch voxel_map_tools_runtime process_map.launch.py \
  input_map:=$GO2_NAV_WORKSPACE/maps/my_scene_sc_map \
  overwrite:=true
```

| 配置文件 | 配置内容 | 如何选择或覆盖 |
|---|---|---|
| [default.yaml](src/localization/voxel_map_tools_runtime/config/default.yaml) | 地面拟合、障碍高度、栅格、动态点剔除及各类输出 | `config:=<文件路径>` |
| [current_robot.yaml](config/calibration/current_robot.yaml) | 动态点剔除使用的雷达射线原点 | `calibration_file:=<文件路径>` |
| [process_map.launch.py](src/localization/voxel_map_tools_runtime/launch/process_map.launch.py) | 输入、输出、标定覆盖和是否替换已有结果 | `input_map`、`output_dir`、`calibration_file`、`overwrite` |

加载顺序为：读取处理 YAML → 用非空的 `input_map`、`output_dir` 启动参数覆盖
对应路径 → 用标定文件覆盖 `dynamic_removal.sensor_origin_imu_m` → 执行处理。
默认输出到输入地图的 `processed/`；已有输出时需要 `overwrite:=true`。
如确实需要使用处理 YAML 中的射线原点，可传 `calibration_file:=''`。

| 处理 YAML 参数组 | 作用 |
|---|---|
| `outputs.*` | 完整、去地面、障碍点云和二维栅格的输出及降采样 |
| `window.*`、`preprocess.*` | 地面拟合窗口和预处理 |
| `ground.*` | 地面拟合、坡度、连续性及地面高度容差 |
| `obstacles.min_height_m/max_height_m` | 进入栅格障碍层的地面以上高度范围 |
| `grid.resolution_m` | 二维栅格分辨率 |
| `dynamic_removal.*` | 动态点剔除开关和证据阈值 |

此步骤不加载建图或定位算法 YAML。定位匹配使用地图包的原始逐帧扫描；
处理后的 `map.yaml/map.pgm` 用于 Nav2，`map_full.pcd` 用于历史地图显示。

### （3） 定位导航：`go2_nav_bringup localization_nav.launch.py`

对应前面的 bag、仅传感器实机和控制 GO2 三种定位导航命令，例如：

```bash
ros2 launch go2_nav_bringup localization_nav.launch.py \
  sensor_source:=live \
  map_path:=$GO2_NAV_WORKSPACE/maps/my_scene_sc_map \
  calibration_file:=$GO2_NAV_WORKSPACE/config/calibration/current_robot.yaml \
  control_robot:=false
```

三种模式使用相同的定位与导航配置。`sensor_source` 选择实时驱动或 bag 回放；
`control_robot` 决定是否启动 GO2 速度桥，不切换定位算法或 Nav2 参数。

| 配置文件 | 配置内容 | 如何选择或覆盖 |
|---|---|---|
| [ros2_localization_rslidar.yaml](src/localization/voxel_slam/config/ros2_localization_rslidar.yaml) | 雷达解析、局部 LIO、IMU 高频预测、LocalBA、定位显示 | `voxel_config:=<文件路径>` |
| [ros2_localization_common.yaml](src/localization/voxel_slam/config/ros2_localization_common.yaml) | DIRECT 匹配、双 TF 时效、修正确认和后续点云验证 | 集成 launch 固定加载；DIRECT 开关有下述覆盖 |
| [ros2_scancontext_rslidar_relocalization.yaml](src/localization/voxel_slam/config/ros2_scancontext_rslidar_relocalization.yaml) | 历史扫描融合、地图缓存、SC 候选、Quatro、Nano-GICP 和 SC 修正跳变限制 | `relocalization_config:=<文件路径>` |
| [current_robot.yaml](config/calibration/current_robot.yaml) | 雷达外参、静态 TF、车体参考系及控制授权 | `calibration_file:=<文件路径>` |
| [nav2_params.yaml](src/navigation/go2_navigation2/config/nav2_params.yaml) | 规划器、局部控制器、代价地图、足迹、速度和平滑器 | `params_file:=<文件路径>` |
| [雷达 config.yaml](src/sensors/rslidar_sdk/config/config.yaml) | 实时雷达驱动 | `lidar_config:=<文件路径>`；仅 live 模式使用 |
| [localization_nav_light.rviz](src/integration/go2_nav_bringup/config/rviz/localization_nav_light.rviz) | 默认轻量 RViz 显示和交互工具 | `rviz_config:=<文件路径>`；`rviz:=false` 关闭 |
| 地图目录下的 `processed/map.yaml` | Nav2 静态地图及栅格图片路径 | 由 `map_path` 解析得到 |
| [localization_nav.launch.py](src/integration/go2_nav_bringup/launch/localization_nav.launch.py) | 地图、运行模式、串口、时间、控制桥门控及最终参数覆盖 | 启动参数覆盖默认值；固定覆盖项见下文 |

#### 定位参数的加载顺序与覆盖

Voxel-SLAM 节点按以下顺序加载：

```text
voxel_config（默认 ros2_localization_rslidar.yaml）
  → ros2_localization_common.yaml
  → relocalization_config（默认 ros2_scancontext_rslidar_relocalization.yaml）
  → 集成 launch 的显式覆盖参数
```

最后的显式覆盖包括：定位模式、地图路径、关闭旧地图保存、固定输入话题
`/rslidar_points` 和 `/imu/data_raw`、历史显示点云路径、标定雷达外参、
手动初始位姿的 `base_frame`、仿真时间，以及
**`Localization.direct_match.enabled=true`**。

因此，公共 YAML 中虽然写着 `enabled: false`，此集成入口仍保持
“首次 SC 定位 → DIRECT 持续修正 → 连续失败或修正超时后启用 SC 恢复”的策略。
如需关闭此入口的 DIRECT，应修改该 launch 中的显式覆盖值；只修改公共 YAML
或 `relocalization_config` 中的开关会再次被覆盖。
原生 `vxlm_localization_rslidar.launch.py` 没有这一强制开启项，仍采用公共 YAML
的 `false`；不要混淆两个入口的最终值。

| 想调整的行为 | 对应配置位置 |
|---|---|
| 局部里程计、预测上限、局部 BA | 传感器定位 YAML 的 `Odometry.*`、`LocalBA.*` |
| DIRECT 尝试周期、位移/角度触发、目标范围和配准阈值 | 公共 YAML 的 `Localization.direct_match.*`（开关除外，见上文） |
| 全局修正过期、双 TF 修正确认、后续点云验证 | 公共 YAML 的 `Localization.dual_tf.*` |
| SC 查询融合帧数、候选数、描述子阈值 | SC YAML 的 `Localization.scancontext.*` |
| 历史描述子融合帧数和抽样间距 | SC YAML 的 `Localization.history.*` |
| SC 已对齐后的候选距离及修正跳变限制 | SC YAML 的 `candidate_max_distance_m`、`max_correction_translation_m`、`max_correction_rotation_deg`，均位于 `Localization.scancontext` 下 |
| IMU/雷达和车体安装外参 | 标定 YAML；不要单独修改算法 YAML 中会被覆盖的外参 |

`use_sim_time` 默认根据输入自动选择：bag 为 `true`，live 为 `false`。
`GLOBAL_STALE` 频繁出现时，应同时检查传感器输入、处理延迟、配准结果和公共配置
中的时效参数；放宽阈值不会恢复已经停止的局部里程计。

#### Nav2：规划、避障和速度

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

#### GO2 控制桥接和 RViz

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

当前双 TF 定位支持 RViz 顶部的 `2D Pose Estimate`（`/initialpose`）：在 `map`
坐标系下点击机器狗 `base_link` 的位置并拖动设置朝向，即可覆盖首次或后续的
全局位置，重新计算 `map -> odom`。它不重置局部 LIO，也不修改 `odom -> imu_link`。
人工指定的 X/Y/yaw 直接作为全局对齐使用，不需要等待 SC 检索或配准；消息中的
Z、roll/pitch 和协方差不参与二维修正，高度与倾斜沿用当前估计。
后续 DIRECT/SC 匹配继续从新位置运行，人工修正前的旧查询结果会被拒绝。

需要先让局部 LIO 初始化，并能查询点击时刻的 `odom -> base_link` TF。
错误坐标系、非有限位姿、过旧或无法查询对应 TF 的消息会被拒绝并输出原因，
此时请在局部定位正常后重新点击。点击后日志出现
`DUAL_TF accepted RViz initial pose` 表示已接受，下一次有效 IMU 预测发布新的 TF。
默认 `Localization.rviz_initialpose_enabled=true`；设置为 false 可禁用人工入口。
`Localization.initialpose_base_frame` 默认为 `base_link`，集成 launch 根据标定文件
的 `base_frame` 自动设置，计算包含 IMU 到车体的外参。
`2D Pose Estimate` 修改定位，`2D Goal Pose` 下发导航目标，二者独立。

定位导航默认使用
[localization_nav_light.rviz](src/integration/go2_nav_bringup/config/rviz/localization_nav_light.rviz)：
10 FPS，显示二维地图、局部代价地图、全局/局部规划路径及 `base_link` 坐标轴。
历史地图点云、实时原始点云、配准点云、轨迹点云和全局代价地图默认关闭，
需要时可以逐项勾选。顶部 `2D Goal Pose` 工具仍发布 `/goal_pose` 给 Nav2，
关闭点云显示不影响定位、避障或下发目标。显示帧率不改变 LIO 或 TF 发布频率。

`rviz:=true` 开启该轻量视图，`rviz:=false` 不启动 RViz。
使用 `rviz_config` 可以选择其他配置，例如恢复完整调试视图：

```bash
ros2 launch go2_nav_bringup localization_nav.launch.py \
  sensor_source:=bag \
  bag_path:=$GO2_NAV_WORKSPACE/bag/zoulang \
  map_path:=$GO2_NAV_WORKSPACE/maps/16zoulang_map \
  control_robot:=false rviz:=true \
  rviz_config:=$(ros2 pkg prefix --share go2_nav_bringup)/config/rviz/localization_nav.rviz
```

远程运行 RViz 可以进一步将渲染负载移到操作电脑。不过当前 bag 模式在 launch
内部强制 `ROS_LOCALHOST_ONLY=1`，直接在另一台电脑打开 RViz 无法接入；需要先
调整该隔离配置，并统一 ROS Domain、DDS 和网络发现设置。实机故障仍需在机器狗
主机上验证，轻量视图不能保证解决 GPU 驱动、软件渲染、内存或温度导致的资源问题。

完整调试视图的地图、代价地图、点云、TF 和路径显示在
[localization_nav.rviz](src/integration/go2_nav_bringup/config/rviz/localization_nav.rviz)：

- `Nav2 Global Plan` 显示 `/plan`。
- `Nav2 Local Plan` 显示 `/local_plan`。
- `Historical Map` 显示 `/localization/map_cloud`。
- `Live RS16 Point Cloud` 显示 `/rslidar_points`。
- `Current Scan` 显示 Voxel-SLAM 的 `/map_scan`。

修改 `.rviz` 后需要在 RViz 中重新打开配置，或重启定位导航 launch。

## 定位导航性能监测

只读脚本 [scripts/monitor_navigation.py](scripts/monitor_navigation.py) 可与导航同时运行，
不发送运动指令、不订阅原始点云，不需要 sudo、sysstat 或重新编译。
在机器狗主机的新终端加载与导航相同的环境（尤其是 ROS Domain、DDS 和网卡），然后运行：

```bash
cd "$GO2_NAV_WORKSPACE"
source scripts/setup_env.sh
python3 scripts/monitor_navigation.py --mode live
```

默认一直记录，按 **回车** 标记一次观察到的停顿，按 **Ctrl+C** 结束并打包。
也可以设置 `--duration 180` 记录 3 分钟。建议包含下目标前、正在导航、出现停顿、
恢复后的过程。开/关 RViz 对照测试应分别记录一份报告。
输出在 `~/go2_perf_logs/navigation_日期时间/`，结束时会打印旁边 `.tar.gz` 的完整路径；
发回这个压缩文件即可。`--output /path/to/reports` 可更改输出父目录。

bag 回放时使用同一个 ROS Domain，改为：

```bash
python3 scripts/monitor_navigation.py --mode bag --duration 180
```

bag 模式自动使用与集成 launch 相同的 Fast DDS、localhost-only 和模拟时间；
实机模式保留当前终端的 DDS/网卡配置。脚本运行后若提示没有 ROS 消息，应先核对
环境和节点是否已启动，此时不能将缺少消息认定为定位故障。
没有 ROS 环境时仍会记录系统数据并在报告中说明；也可显式指定 `--system-only`。

报告主要内容：

- `metadata.json`：主机、CPU、网络、图形渲染器（能查询时）、Git 版本及监测环境。
- `cpu.csv`、`processes.csv`、`threads.csv`：每核 CPU，以及关键进程和最忙的线程。
  进程/线程 CPU 以单核 100% 为单位，线程默认每进程每次保留前 8 个。
- `system.csv`、`disks.csv`、`sensors.csv`：内存、交换、系统压力、磁盘吞吐/I/O 等待、
  温度、CPU 频率及硬件支持时的节流计数。磁盘与分区不能重复求和。
- `topics.csv`：速度、IMU、里程计、两条动态 TF 和定位状态的每秒接收统计、
  当前持续失联时间和源时间戳年龄，不记录高带宽原始点云。
- `events.jsonl`：消息间隔异常、定位状态变化、导航 action 状态、生命周期、
  发布/订阅端发现信息和人工时间标记。
- `summary.json`：整体汇总，以及仅在同一 FollowPath 执行段中的消息间隔，
  避免把等待新目标的正常停发混入控制卡顿；持续未恢复的停发需结合 `topics.csv`。
- `ros_logs/`：相关节点日志尾部及最近三次 launch 日志尾部，每个文件最多 192 KiB。

采样间隔默认 1 秒；`--interval 2 --top-threads 4` 可以进一步降低监测负担。
消息间隔是监测节点收到消息的时间，可能受 DDS 或监测节点调度影响；需要将它与
CPU/线程、TF/里程计、action 状态和节点日志对齐后判断原因。报告会记录监测程序
自身 CPU 用时，不会仅凭高负载就认定其为根因。

## 目录

- `src/sensors`：外置 IMU、RS16 驱动及雷达消息。
- `src/localization`：Voxel-SLAM、地图处理和 RViz 点云插件。
- `src/robot`：宇树消息接口和带定位门控的速度桥接。
- `src/navigation`：Nav2 参数及独立 Nav2 入口。
- `src/integration/go2_nav_bringup`：实时/bag 建图、定位导航统一入口。
- `maps`：v4 扫描地图和 `processed` Nav2 栅格地图。
- `config/calibration`：静态 TF 与 Voxel-SLAM 外参的唯一配置源。
- `third_party/install`：本工作空间使用的 GTSAM、TEASER++ 前缀。
