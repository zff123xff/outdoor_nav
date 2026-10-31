# 北京 ZNY 导航侧接口对齐记录

日期：2026-10-21。依据：仓库内“北京ZNY仿真端对接/图片和附件/go2w仿真接口说明(2).md”及主文档的半实物启动、原点设置步骤。

## 改动目标与入口变化

当前仿真提供 odom、点云，并接收 cmd_vel；导航发布公共 TF，默认使用系统时间。
这次完成导航侧修改，没有调整仿真雷达外参、点云格式、QoS 或速度上限。

| 命令 | 当前作用 |
| --- | --- |
| `bash deploy/nx/run.sh hil` | **北京 ZNY 模式**，独立于 outdoor_nav_sim |
| `bash launch_hil.sh` | 同上，默认 robot_1、系统时间 |
| `bash deploy/nx/run.sh nav` | 原真机模式 |


## 修改内容

1. 独立导航启动链路：新增 src/robot_launch/launch/outdoor_zny.launch.py 和 config/zny_hil.yaml。 复用项目内 HIL 导航、odom QoS relay 和 twist_mux，不查找仿真项目，不启动狗驱动、MAVROS、MID360、自动原点或遥控转换节点。
2. 仿真拥有完整 TF：map→odom 为单位阵；base_link→livox_frame 为 (0.18,0,0.53)m、前倾约 18°； odom→base_link 来自仿真融合里程计，保留完整高度、姿态及消息时间戳。导航 HIL 入口固定 publish_tf=false，关闭输入桥动态 TF 和两个固定 TF 节点。
3. 系统时间默认值：导航默认 use_sim_time=false，没有 /clock 也可工作。 go2w_nav2.launch.py 同步改写 YAML 中所有 use_sim_time，包括 costmap 子节点，防止只修改父节点而遗留子节点的仿真时钟。 若双方明确采用 /clock，可显式传 use_sim_time:=true；这时仿真必须发布 /clock。
4. 外部原点输入：导航开启 accept_external_origin=true，订阅 /{ns}/mavros/set_gp_origin，类型 geographic_msgs/msg/GeoPointStamped。 接收后更新后续 GPS 目标使用的 ENU 原点，以及 reference_latitude/longitude/altitude 参数查询值。 非有限值、纬度或经度越界会被拒绝。未收到消息前使用YAML 中的参考点。
5. 运行环境解耦：launch_hil.sh 加载导航 local_setup.bash，避免 workspace setup 隐式加载构建时的仿真 underlay。 run.sh hil 不读取真机 config.env，不要求狗网口或飞控。默认 domain 30，保留已设置的 ROS_DOMAIN_ID、RMW 和 DDS 配置。
6. 速度限幅：新配置将控制器 max_angular_vel 设为 1.0 rad/s，线速度保持原配置的 1.0 m/s。 这些是导航配置，不代替仿真/真机执行端的速度限幅和超时保护。

## 主要文件

| 文件（相对 outdoor_nav） | 内容 |
| --- | --- |
| `launch_hil.sh` | 加载本导航工作空间，启动 outdoor_zny |
| `src/robot_launch/launch/outdoor_zny.launch.py` | 独立 ZNY 参数与启动组合 |
| `src/robot_launch/config/zny_hil.yaml` | ZNY 原点、导航和限速配置 |
| `src/robot_launch/launch/outdoor_hil.launch.py` | 转发高度和外部原点开关 |
| `src/robot_launch/launch/go2w_nav2.launch.py` | 桥接参数、递归统一节点时钟 |
| `src/navigation2/nav2_input_bridge/{include,src}` | TF 高度直接使用 odom.z，其他输入桥逻辑恢复原始实现 |
| `src/navigation2/nav2_input_bridge/{CMakeLists.txt,package.xml}` | geographic_msgs 构建依赖 |
| `src/robot_launch/robot_launch/lifecycle_*.py`、`setup.py`、`package.xml` | 独立生命周期中继、启动截止检查与入口注册 |
| `tests/test_lifecycle_relay.py` | 转换响应丢失的故障注入回归 |
| `tests/zny_contract.py` | 独立三话题契约与旧默认行为回归测试 |

仿真项目仅同步了 Classic 入口名称的说明和回归测试调用，不改变其运行协议。

## 编译和运行

因为修改了 C++，需要重新编译 nav2_input_bridge 和 robot_launch，不能只重新安装 launch 文件。

开发机（已有依赖）：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --packages-select nav2_input_bridge robot_launch --symlink-install --cmake-args -DBUILD_TESTING=OFF
ROS_DOMAIN_ID=30 bash deploy/nx/run.sh hil
```

NX 使用原生 ARM64 构建，已有部署可执行 `bash deploy/nx/install.sh build`，或按原 install_nx/build_nx 布局重新编译上述两个包。
运行需安装 twist_mux、导航包和消息依赖；NX 安装脚本已包含相关依赖。无需安装 Gazebo、仿真源码或仿真二进制。

仿真端与导航端的 domain、命名空间、时间基准必须一致。系统时间模式需同步两机时钟；odom/点云时间戳也应使用该时间基准。
仿真端按接口发布 /robot_1/mavros/odom 和 /robot_1/livox/lidar，订阅 /robot_1/cmd_vel。
频率、雷达安装和点云字段仍须由仿真端保证；导航侧改动不会自动修正不符合文档的仿真数据。

原点设置示例（在下发目标前执行）：

```bash
ros2 topic pub --once /robot_1/mavros/set_gp_origin geographic_msgs/msg/GeoPointStamped \
  "{header: {frame_id: 'robot_1/map'}, position: {latitude: 40.9445353, longitude: 113.172819, altitude: 1298.7345711102703}}"
```

该输入只更新 ROS 导航的坐标换算，不向真实 PX4 下发设置，也不模拟 MAVROS 的飞控状态或原点反馈。
原点改变不会自动重投影既有目标、更新仿真世界或重置多航点队列。应在任务开始前统一原点；已有任务需先取消，再将导航、仿真和队列配置同步。

## 验证记录与范围

结果见 [validation/zny_contract_20261021.json](validation/zny_contract_20261021.json)。
执行：`python3 tests/zny_contract.py --domain-id 96`（先 source ROS 与导航工作空间）。
开发机未安装系统 twist_mux，测试时通过 AMENT_PREFIX_PATH 提供已有的 twist_mux 安装路径；未加载仿真 launch、传感器或 TF 节点。

空闲隔离域内，测试源只发布协议用人工 odom/点云和目标，无机器人、Gazebo、/clock 或外部 TF：

- 两个导航生命周期节点进入 active，管理器验证 bond；BEST_EFFORT 输入通过 odom relay 交给 Nav2。
- 导航独立生成完整 TF，保留 odom z=0.42；雷达外参符合文档。
- 即使输入 YAML 写 use_sim_time=true，默认入口仍使桥接、控制器、规划器和 costmap 使用系统时间。
- 外部原点更新与非法原点拒绝通过；GPS→ENU 使用更新后的原点。
- 收到目标后产生非零 cmd_vel，取消后归零；原真机默认仍输出平面 TF、不订阅外部原点。
- 运行 `python3 tests/test_lifecycle_relay.py`，3 项故障回归通过：响应丢失仅在目标状态核实后成功、无法核实则失败且不重发、初始状态未知则不发送转换。

这是导航接口测试，不是物理运动/到达验收，也未在 NX 跨机网络或真实机器狗上执行。

## 最终恢复记录

按用户要求撤回 TF 迁移和外部原点接入：公共定位与雷达安装 TF 恢复由导航发布；输入桥恢复原始逻辑，仅保留 z=odom.z 的修正。仿真私有模型 TF 和 RViz 显示合并链路保留，命名空间配置与项目结构整理不回退。必须重启两侧，避免旧仿真定位 TF 和新导航 TF 重复。

当前验证记录位于相邻仿真项目 tests/results/navigation_tf_restored.json；simulation_tf_boundary.json 是此前方案的历史结果。

## 故障记录：local_costmap 报 “two or more unconnected trees”

### 现象

```
[robot_1.local_costmap.local_costmap]: Timed out waiting for transform from
robot_1/base_link to robot_1/map to become available, tf error: Could not find a
connection between 'robot_1/map' and 'robot_1/base_link' because they are not
part of the same tree.Tf has two or more unconnected trees.
```

### 根因

`robot_1/odom -> robot_1/base_link` 这条动态 TF **只由 nav2_input_bridge 在收到
里程计时广播**。里程计不来 → 这条 TF 不存在 → TF 树被切成两半：

```
{robot_1/map -> robot_1/odom}      （静态发布）
{robot_1/base_link -> robot_1/livox_frame}   （静态发布）
```

两半互不连通，local_costmap 无法把机器人位姿从 `robot_1/base_link` 变换到
`robot_1/map`，于是反复打印上面这条错误。**错误信息指向 TF，实际原因是里程计
没有到达 nav2_input_bridge。**

排查命令（导航侧）：

```bash
ros2 topic info -v /robot_1/mavros/odom    # Publisher count 必须是 1
ros2 topic hz   /robot_1/mavros/odom
ros2 run tf2_ros tf2_echo robot_1/map robot_1/base_link
```

### 导航侧加固

`nav2_input_bridge` 现在会自己说清楚原因，不再让 nav2 的 TF 报错单独承担诊断：

- 启动时打印订阅的里程计话题、备用话题和广播的 TF 帧名；
- 3 秒内没有任何里程计 → 每 10 秒打印一条 ERROR，区分两种情况：
  - 话题上**没有发布者** → 提示去启动里程计源（实机 mavros/PX4、HIL 仿真）；
  - 话题上**有发布者但收不到消息** → 提示两侧 ROS_DOMAIN_ID / RMW 实现不一致
    （Cyclone DDS 与 Fast DDS 不能互通），或发布者用了 TRANSIENT_LOCAL 持久性
    而本节点用 VOLATILE 订阅。
- 里程计中途断流 → 打印 ODOMETRY STOPPED。
- 新增 `local_odom_fallback_topic`（默认 `odom`）：仅当主话题
  `mavros/odom` 上没有发布者、且超过 `odom_timeout_s` 无数据时，才改用备用话题，
  覆盖“仿真直接发到 /{ns}/odom” 的情况。主话题一旦有数据就始终优先。

参数（`nav2_input_bridge.ros__parameters`）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `local_odom_topic` | `mavros/odom` | 主里程计话题（相对名，随命名空间解析） |
| `local_odom_fallback_topic` | `odom` | 备用里程计话题，空字符串关闭 |
| `odom_timeout_s` | `2.0` | 超过多久没有里程计算断流 |
| `odom_watchdog_period_s` | `1.0` | 看门狗检查周期 |

### 顺带修正

`nav2_input_bridge/config/defaults.yaml` 原先写的是 ROS 1 的
`/mavros/local_position/odom`；ROS 2 的 mavros 发布的是 `~/odom`，用 ROS 1 路径
同样会导致没有里程计 → 同样的 TF 报错。已改为 `mavros/odom`，并把话题名改成相对
名以支持命名空间。
