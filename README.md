# outdoor_nav

outdoor 导航项目，基于 ROS 2 构建，融合激光雷达感知与 Nav2 导航框架，运行在 NVIDIA Orin NX 开发板上。

## 系统架构与运行逻辑

**目标**：无人机获取目标人物的 GPS 坐标后发送给地面端小车，小车根据自身 GPS 位置和目标 GPS 位置进行全局无图路点导航 + 局部动态避障。

支持两种平台：
- **轮式小车**：使用 CAN 总线驱动 + PX4 EKF2 融合定位
- **Go2-W 机器狗**：使用 DDS 高层运动控制 + PX4 EKF2 融合定位

为减少 GPS/IMU 融合算法的开发量，小车状态（位姿 + 速度）直接采用 **PX4 EKF2 融合定位**（`/mavros/local_position/odom`）。

### 坐标系统

1. **局部坐标原点**：小车上电并在室外获得稳定 GPS 信号（3D fix）后，Pixhawk EKF2 将当前位置锁定为全局原点 `gp_origin`（经纬度），对应局部坐标系原点 `(0, 0, 0)`
   - 方向：固定 ENU（X-东，Y-北，Z-天）
2. **坐标投影**：`nav2_input_bridge` 使用 **PX4 球面局部切平面投影**（`PX4-Autopilot/src/lib/geo/geo.cpp`），把 WGS84 经纬度投影到以原点为中心的米制平面坐标

### 目标输入

`nav2_input_bridge` 同时接收两类 GPS 目标点：

- **`/gp_goal`**：手动测试/地面站注入（`sensor_msgs/NavSatFix`）
- **`/uav/target_gps`**：无人机通过电台发来的目标坐标

节点将目标坐标转换为局部 ENU 后发布到 `/goal_pose`，Nav2 控制器执行路径跟踪与避障。

### YOLO 停止机制

当同机运行的 YOLO 检测到目标人物时，发布 **`/target_detected`**（`std_msgs/Bool`，`data=true`）。

`nav2_input_bridge` 收到后：
1. 发布 `stop_planner=true` 通知 `straight_planner` 停止发送新路径
2. 取消 `controller_server` 上正在执行的 FollowPath action，controller 直接减速停车

`straight_planner` 收到停止信号后：
- 取消当前 FollowPath，记录停止时的目标位置
- 此后收到的目标若与停止位置距离 `< 0.1 m`，则拒绝（防止重复目标导致反复启动）
- 若收到距离足够远的新目标，则清除停止状态，正常接受并前往

## 项目结构

```
outdoor_nav/
├── src/driver/                    # 硬件驱动
│   ├── go2w_driver/              # Go2-W DDS 驱动（替代 ros2can）
│   ├── livox_ros_driver2/         # Livox 激光雷达驱动 (submodule)
│   └── ros2can/                   # 小车 CAN 总线通信节点（Go2-W 不使用）
├── src/navigation2/               # 导航相关包
│   ├── nav2_common/               # Nav2 公共库
│   ├── nav2_controller/           # 控制器
│   ├── nav2_core/                 # 核心接口定义
│   ├── nav2_costmap_2d/           # 代价地图
│   ├── nav2_input_bridge/         # GPS→ENU 目标坐标转换 + YOLO 停止
│   ├── nav2_lifecycle_manager/    # 生命周期管理
│   ├── nav2_msgs/                 # 自定义消息
│   ├── nav2_regulated_pure_pursuit_controller/  # 纯追踪控制器
│   ├── nav2_straight_planner/     # 直线路径规划器
│   ├── nav2_util/                 # 工具函数
│   └── nav2_voxel_grid/           # 体素网格
├── src/slam/                      # SLAM（仅离线建图用，不参与导航运行时）
│   └── FAST_LIO/                  # 快速激光 SLAM (submodule)
└── src/robot_launch/              # 启动文件与配置
```

## 环境要求

> **Shell 说明**：本文档命令基于 zsh 编写。使用 bash 时，将 `.zsh` → `.bash`、`.zshrc` → `.bashrc`。

- **操作系统**：Ubuntu 22.04
- **ROS 发行版**：ROS 2 Humble
- **编译工具**：`colcon`、`CMake >= 3.16`

### 安装 ROS 2 Humble

参考官方文档：[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

一键安装命令： `source <(wget -qO- http://fishros.com/install)`

安装后记得 source 环境：

```bash
source /opt/ros/humble/setup.zsh
```

## 克隆仓库

本项目包含 **Git Submodule**，请使用 `--recurse-submodules` 参数克隆：

```bash
git clone --recurse-submodules git@github.com:VisionTrekker/outdoor_nav.git
cd outdoor_nav
```

如果已经克隆但忘记拉取子模块，可执行：

```bash
git submodule update --init --recursive
```

## 安装依赖

在项目根目录下，使用 `rosdep` 安装所有依赖：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> 若某些依赖无法通过 `rosdep` 自动安装（如 Livox-SDK2、OpenVDB 等），请根据对应子模块的 README 手动安装。

## 编译

项目提供了自动化编译脚本：

```bash
./build.sh
```

> 等价于：`colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF`

编译完成后，source 本地工作空间：

```bash
source install/setup.zsh
```

建议将 source 命令添加到 `~/.zshrc` 中：

```bash
echo "source [path_to_outdoor_nav]/install/setup.zsh" >> ~/.zshrc
```

## 仿真环境测试

接北京仿真接口、由 NX 运行导航时，使用 `bash launch_hil.sh`。
命名空间、时间和接口配置见 [北京 HIL 接入说明](deploy/hil/README.md)。
NX 使用 `bash deploy/nx/run.sh hil`；原本地 Classic 适配使用 `hil-classic`。
本次导航侧接口变更记录见 [ZNY_NAV_ALIGNMENT.md](docs/ZNY_NAV_ALIGNMENT.md)。

在实机部署前，建议先在本地仿真环境中验证算法逻辑。仿真使用 Gazebo Classic v11.10.2 + ROS 2 Humble。

### 前置要求

- ROS 2 Humble（已安装）
- Gazebo Classic v11.10.2

### 1. 创建并编译仿真工作空间

```bash
mkdir -p [path_to_sim_ws]/src
cd [path_to_sim_ws]/src

# 克隆 mid360_simulation
git clone https://github.com/inkccc/mid360_simulation.git

# 确保 gazebo_diffbot 也在 sim_ws/src 下
# （gazebo_diffbot 需自行准备）
```

编译仿真依赖：

```bash
cd [path_to_sim_ws]
source /opt/ros/humble/setup.zsh
colcon build --packages-select gazebo_diffbot mid360_simulation --symlink-install
```

验证插件安装成功：

```bash
ls install/mid360_simulation/lib | grep libmid360_plugin.so
```

### 2. 配置 Gazebo 环境变量

将以下环境变量追加到 `~/.zshrc` 以持久化：

```bash
echo 'export GAZEBO_MODEL_PATH=[path_to_sim_ws]/src/gazebo_diffbot/worlds:$GAZEBO_MODEL_PATH' >> ~/.zshrc
echo 'export GAZEBO_PLUGIN_PATH=[path_to_sim_ws]/install/lib:$GAZEBO_PLUGIN_PATH' >> ~/.zshrc
source ~/.zshrc
```

### 3. 编译 outdoor_nav

```bash
cd [path_to_outdoor_nav]
source /opt/ros/humble/setup.zsh
bash build.sh
```

### 4. 启动仿真环境

**终端 1 — 启动 Gazebo + diff_bot：**

```bash
source /opt/ros/humble/setup.zsh
source [path_to_sim_ws]/install/setup.zsh
source [path_to_outdoor_nav]/install/setup.zsh

ros2 launch gazebo_diffbot gazebo.launch.py \
  world:=[path_to_sim_ws]/src/gazebo_diffbot/worlds/slope_with_pillar_2.world
```

启动后验证 `/odom`、`/livox/lidar`、`/cmd_vel` 等话题正常输出即可。

**终端 2 — 启动导航栈：**

```bash
cd [path_to_outdoor_nav]
source /opt/ros/humble/setup.zsh
source install/setup.zsh
bash launch.sh
```

### 5. 功能验证测试

#### 测试 1：基本导航

发送目标点：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {stamp: {sec: 0}, frame_id: "map"}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}'
```

预期：机器人开始移动。可在 rviz 中观察局部地图和机器人位置。

监控速度（可选）：

```bash
ros2 topic echo /cmd_vel
```

#### 测试 2：YOLO 检测后立即停止

模拟 YOLO 检测到人：

```bash
ros2 topic pub --once /target_detected std_msgs/msg/Bool '{data: true}'
```

验证：
- `/cmd_vel` 速度归零（`vx=0, vy=0, vz=0`）
- `straight_planner` 日志输出 `Stop requested!`

#### 测试 3：拒绝重复目标 + 接受新目标

1. 发送目标 A：
   ```bash
   ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
     '{header: {stamp: {sec: 0}, frame_id: "map"}, pose: {position: {x: 0.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}'
   ```

2. 模拟 YOLO 检测停止：
   ```bash
   ros2 topic pub --once /target_detected std_msgs/msg/Bool '{data: true}'
   ```

3. 再次发送**相同**目标 A：
   - 预期：`straight_planner` 输出 `Rejecting goal that matches stopped goal`，小车不动

4. 发送**新**目标 B（距离足够远）：
   ```bash
   ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
     '{header: {stamp: {sec: 0}, frame_id: "map"}, pose: {position: {x: 10.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}'
   ```
   - 预期：小车清除停止状态，前往新目标 B

---

## 实机测试流程

### 1. 重启小车

> 每次实机测试前建议先重启小车。因为小车与遥控器建立通信后，可能会导致 NVIDIA Orin NX 板的控制断连，重启可恢复稳定的控制链路。

### 2. 一键启动全车系统

编译完成后，在项目根目录下直接运行：

```bash
bash launch_all.sh
```

*(可选)* 如需手动设置原点（如在室内测试无GPS信号，不想由于超时等待报错），可增加 `use_auto_origin` 参数禁用自动设置脚本：

```bash
bash launch_all.sh use_auto_origin:=false
```

> 该脚本会自动 source ROS 2 环境和本地 install，然后启动 `outdoor_all.launch.py`。

该 launch 会依次启动：
1. 小车运控（`ros2can car_driver.launch.py`）
2. PX4 融合定位（`mavros px4.launch`）
3. 导航栈（`gp_goal_nx_nav2.launch.py`，含 Livox 驱动、Nav2 控制器、局部避障等）
4. **自动设置局部坐标原点**：启动 10 秒后自动发布 `set_gp_origin`，并订阅反馈验证；若失败会自动重试（最多 3 次）

启动后观察终端日志，当看到 `gp_origin verified successfully` 时，系统已就绪（若已禁用自动原点设置，则待系统稳定后可通过命令行或地面站手动注入原点）。

### 下发目标点

系统就绪后，在**另一个终端**下发 GPS 目标点：

```bash
# 手动测试
ros2 topic pub --once /gp_goal sensor_msgs/msg/NavSatFix \
  "{header: {frame_id: 'gps'}, latitude: 30.8135718, longitude: 120.8338169, altitude: 12.318126322268082}"

# 或模拟无人机链路
ros2 topic pub --once /uav/target_gps sensor_msgs/msg/NavSatFix \
  "{header: {frame_id: 'gps'}, latitude: 30.8135718, longitude: 120.8338169, altitude: 12.318126322268082}"
```

### 手动测试 YOLO 停止机制

在小车行驶过程中，可通过以下命令模拟 YOLO 检测到目标，触发立即停车。停止后相同位置的目标会被拒绝，距离足够远的新目标可正常接受：

```bash
ros2 topic pub --once /target_detected std_msgs/msg/Bool '{data: true}'
```

若需单独观察原点反馈：

```bash
ros2 topic echo /mavros/global_position/gp_origin
```

原点设置参数位于 `src/robot_launch/config/gp_goal_nx_nav2.yaml`：

```yaml
nav2_input_bridge:
  ros__parameters:
    reference_latitude: 30.8135718
    reference_longitude: 120.8338169
    reference_altitude: 12.318126322268082
    goal_input_topic: "/gp_goal"
    uav_goal_input_topic: "/uav/target_gps"
    target_detected_topic: "/target_detected"
    goal_output_topic: "/goal_pose"
    goal_output_frame: "map"
```

### 备选：分步手动启动

若某模块需要单独调试，仍可分别启动：

```bash
# 终端 1：小车运控
bash launch_car.sh

# 终端 2：PX4 融合定位
bash launch_loc.sh

# 终端 3：导航
bash launch.sh
```

---
## Go2-W 部署

本节记录将 outdoor_nav 部署到宇树 Go2-W 机器人的完整流程。

### go2w_driver 驱动节点

`go2w_driver` 替代原 ros2can，实现 `/cmd_vel` → `/api/sport/request` 的桥接：

- **订阅**：`/cmd_vel` (geometry_msgs/Twist)
- **发布**：`/api/sport/request` (unitree_api/Request)
- **订阅状态**：`/lf/sportmodestate` (unitree_go/SportModeState)
- **核心功能**：
  - 速度限幅：vx[-1.5,1.5], vy[-0.6,0.6], vyaw[-1.0,1.0]
  - 超时保护：0.5s 无新指令自动 StopMove（狗 1s 无指令自动停下）
  - 启动序列：延迟 2s 发送 BalanceStand，等待 mode=1 后开始接受指令

### NX 实机测试流程

#### 1. 网络配置

NX 通过 USB 网口连接狗的主控板：

```bash
# 查询网口名称（通常为 enx* 格式）
ifconfig

# 配置静态 IP（狗的机载电脑 IP 为 192.168.123.161）
sudo ip addr add 192.168.123.222/24 dev <网口名>
sudo ip link set <网口名> up

# 验证连接
ping 192.168.123.161
```

#### 2. 设置 DDS 通信网口

> **重要**：`launch_go2w.sh` 和 `launch_all_go2w.sh` 要求 `GO2W_NET_IFACE` 环境变量必须设置，否则会报错退出。`GO2W_NET_IFACE` 指定 CycloneDDS 使用的网络接口，必须设置为连接狗的 USB 网口。

```bash
# 查询 USB 网口名称（通常为 enx* 格式）
ifconfig

# 临时设置（仅当前终端生效）
export GO2W_NET_IFACE=<连接狗的网口名>

# 持久化设置（推荐，每次开机自动生效）
echo 'export GO2W_NET_IFACE=<连接狗的网口名>' >> ~/.bashrc
source ~/.bashrc
```

#### 2.1 可选：配置局域网网口（用于远程 RViz）

如果需要从本机远程查看点云和导航状态，需要配置第二个网口：

```bash
# 设置局域网网口（连接本机的网口）
export GO2W_LAN_IFACE=<局域网网口名>

# 持久化设置
echo 'export GO2W_LAN_IFACE=<局域网网口名>' >> ~/.bashrc
source ~/.bashrc
```

**网口配置示例**（根据实际情况修改）：

| 网口 | IP 地址 | 用途 |
|------|--------|------|
| `enx0826ae32db83` | 192.168.123.222 | 连接 Go2W 狗 |
| `enP8p1s0` | 10.88.103.187 | 连接局域网（本机） |

设置后，`setup_go2w.sh` 会自动配置 CycloneDDS 使用两个网口进行通信。

#### 3. 代码同步

```bash
rsync -avz --progress \
    --exclude='build/' \
    --exclude='install/' \
    --exclude='log/' \
    --exclude='src/slam/FAST_LIO/' \
    --exclude='*.so' \
    --exclude='*.a' \
    /media/lenovo/disk/KT5/outdoor_nav/ \
    nhy@<NX_IP>:~/ww/outdoor_nav/
```

#### 4. NX 上编译

```bash
cd ~/ww/outdoor_nav
# 编译只需要 unitree_go 包可见
source ~/ww/3rd/unitree_ros2/cyclonedds_ws/install/setup.bash

bash build.sh
```

> `setup_go2w.sh` 用于运行时启动 DDS 通信（需要 `GO2W_NET_IFACE`），而编译阶段只需 `unitree_go` 包在 `CMAKE_PREFIX_PATH` 中可见，两者使用场景不同。

#### 5. 启动 wheeled_sport 服务

在宇树 App 中：设置 → 服务状态 → 启动 `wheeled_sport`

#### 6. 全系统启动

> 确保已设置 `GO2W_NET_IFACE`（见步骤 2）
> 启动脚本会从 `src/robot_launch/config/go2w_nav2.yaml` 的 `nav2_input_bridge` 参数中读取 `reference_latitude/longitude/altitude`，作为 `gp_origin` 原点坐标，无需单独修改。

```bash
cd ~/ww/outdoor_nav
source ~/ww/3rd/unitree_ros2/cyclonedds_ws/install/setup.bash
source setup_go2w.sh
# 查询能否成功接收到狗的topic
ros2 topic list | grep -E "(lf|sport|mode)"

bash launch_all_go2w.sh
```

#### 7. GPS 目标点下发

系统就绪后（另一个终端）：

```bash
source ~/ww/3rd/unitree_ros2/cyclonedds_ws/install/setup.bash
ros2 topic pub --once /gp_goal sensor_msgs/msg/NavSatFix \
  "{header: {frame_id: 'gps'}, latitude: 30.8135718, longitude: 120.8338169, altitude: 12.318126322268082}"
```

预期：狗开始朝目标方向运动。

#### 8. 监控 /cmd_vel

```bash
source ~/ww/3rd/unitree_ros2/cyclonedds_ws/install/setup.bash
ros2 topic echo /cmd_vel
```

验证速度指令正常输出。

#### 9. 测试转向

```bash
source ~/ww/3rd/unitree_ros2/cyclonedds_ws/install/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

预期：狗原地旋转。

#### 10. 测试 YOLO 停止机制

在狗运动过程中（另一个终端）：

```bash
source ~/ww/3rd/unitree_ros2/cyclonedds_ws/install/setup.bash
ros2 topic pub --once /target_detected std_msgs/msg/Bool '{data: true}'
```

预期：`/cmd_vel` 速度归零，狗立即停止。

### 跳过 go2w_driver（仅测试定位和导航）

如需在狗不运动的情况下测试定位、MID360 障碍物显示和 Nav2 导航流程，可以注释掉 `outdoor_all_go2w.launch.py` 中的 `go2w_driver_launch`：

```python
# go2w_driver_launch,
```

这样可以正常验证：
- MAVROS 定位（`/mavros/local_position/odom`）
- MID360 障碍物在 RViz costmap 中显示
- GPS 目标点下发和导航规划

### 参数调整（如需要）

| 参数 | 文件位置 |
|------|----------|
| 原点经纬度 | `src/robot_launch/config/go2w_nav2.yaml` 的 `nav2_input_bridge` 节点 |
| 导航参数 | `src/robot_launch/config/go2w_nav2.yaml` |
| MID360 外参 | `src/robot_launch/launch/go2w_nav2.launch.py` 第 66 行 `static_tf_base_lidar` |

### 跨机 RViz 查看 NX 点云

在本机（开发机）上通过 RViz 查看 NX 发布的 `/livox/lidar` 点云，需确保 DDS 通信正常。

**步骤 1**：确保本机与 NX 在同一 ROS 域

```bash
# 临时设置（仅当前终端生效）
export ROS_DOMAIN_ID=18

# 持久化设置（推荐）
echo 'export ROS_DOMAIN_ID=18' >> ~/.bashrc
source ~/.bashrc
```

**步骤 2**：设置本机网口（连接局域网的网口）

<本机连接局域网的网口为 `eno1`，则配置如下：

```bash
# 临时设置（仅当前终端生效）
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eno1" priority="default"/></Interfaces></General></Domain></CycloneDDS>'
```

**步骤 3**：验证话题可达

```bash
ros2 topic list | grep livox
```

若能看到 `/livox/lidar` 和 `/livox/imu`，说明 DDS 发现正常。

**步骤 4**：在 RViz 中添加点云

1. 本机启动 RViz：`rviz2`
2. Fixed Frame 设置为 `map`
3. 添加 `By Topic` → `/livox/lidar` → `PointCloud2`


## 模块调试（开发场景）

### 建图（FAST_LIO + Livox）

> FAST-LIO 仅用于离线建图/调试，不参与导航运行时。日常编译已通过 `build.sh` 中的 `--packages-skip fast_lio` 跳过，如需编译 FAST-LIO，请使用 `./build.sh --packages-skip ''` 或单独编译：`colcon build --packages-select fast_lio --symlink-install --cmake-args -DBUILD_TESTING=OFF`。

```bash
bash launch_fastlio.sh
```

## 常见问题

1. **Submodule 目录为空**
   - 确保克隆时使用了 `--recurse-submodules`，或后续执行 `git submodule update --init --recursive`。

2. **编译时找不到 Livox-SDK2**
   - 请参考 `src/driver/livox_ros_driver2/README.md` 手动编译安装 Livox-SDK2。

3. **YOLO 触发后小车会停在哪里？会不会掉头回来？**
   - 当前采用方案B：通过 `stop_planner` + 取消 FollowPath action 实现停止。controller 收到 cancel 后直接减速到零，**不会引入新的目标点**，因此不存在”目标在后方导致掉头”的问题。
   - 从 YOLO 检测到目标 → `stop_planner` 发出 + cancel action，端到端延迟约 **100-200 ms**（含图像推理、ROS2 传输、action 异步取消、CAN 响应）。
   - 按当前线速度 `0.5 m/s` 估算，惯性滑行约 **0.05-0.1 m**。
   - 若未来提高车速，可引入”提前制动距离”或”检测框相对位姿”优化。
