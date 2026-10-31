# NX 接入仿真

北京 ZNY 接口使用：

```bash
ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash deploy/nx/run.sh hil
# 等价入口
ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash launch_hil.sh
```

指定启动哪台机器人（命名空间决定全部话题、TF 帧名和节点名）：

```bash
# 显式命名空间参数
bash launch_hil.sh --namespace robot_2
bash launch_hil.sh -n robot_2
bash launch_hil.sh --namespace=robot_2

# 原始 ROS launch 参数仍然可用
bash launch_hil.sh robot_namespace:=robot_2

# 环境变量（deploy/nx/run.sh hil 用的是这个）
HIL_ROBOT_NAMESPACE=robot_2 bash launch_hil.sh
HIL_ROBOT_NAMESPACE=robot_2 bash deploy/nx/run.sh hil
```

优先级：`--namespace` > `robot_namespace:=` > params_file 里的
`launch_config.ros__parameters.robot_namespace` > `robot_1`。命名空间必须符合
`[A-Za-z][A-Za-z0-9_]*`，不符合会在启动前直接报错退出。

默认 robot_1、系统时间。仿真提供 mavros/odom、livox/lidar，消费 cmd_vel；导航提供完整公共 TF。
导航端无需 Gazebo 或仿真项目适配器；默认无需 /clock，公共 TF 由导航生成。
可传 robot_namespace、params_file、use_foxglove；只有明确使用 /clock 时传 use_sim_time:=true。
两侧 domain、命名空间、GPS 原点和消息时间基准应一致。
本 HIL 链路两侧统一使用 Fast DDS（`rmw_fastrtps_cpp`）。启动命令显式指定 RMW，
避免继承实机 Unitree 终端的 Cyclone DDS 设置。本次联调出现过跨 DDS 话题可接收、
但参数和生命周期服务无响应，导致 forward、waypoints 无法通过发送前检查的情况。
此设置只用于 HIL，不修改实机 Unitree 驱动所需的 Cyclone DDS 配置。

板卡若未安装 Fast DDS 的 ROS 适配包，先执行：

```bash
sudo apt install ros-humble-rmw-fastrtps-cpp
```

导航就绪后，在主机 `outdoor_nav_sim` 目录做只读服务检查：

```bash
source ros_env.sh
ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_fastrtps_cpp timeout 10 \
  ros2 param get /robot_1/nav2_input_bridge reference_latitude
```

应返回与导航配置一致的纬度；仅发现话题不能证明服务通信正常。

## 常见故障：local_costmap 报 "two or more unconnected trees"

```
Timed out waiting for transform from robot_1/base_link to robot_1/map ...
Could not find a connection between 'robot_1/map' and 'robot_1/base_link' because
they are not part of the same tree.Tf has two or more unconnected trees.
```

这条报错指向 TF，**实际原因是里程计没有到达导航**。导航侧的
`robot_1/odom -> robot_1/base_link` 动态 TF 只在收到里程计时广播；里程计不来，
TF 树就分成 `{map -> odom}` 和 `{base_link -> livox_frame}` 两半，互不连通。

按顺序检查：

```bash
# 1) 仿真是否在发里程计（Publisher count 必须 >= 1）
ros2 topic info -v /robot_1/mavros/odom

# 2) 里程计是否真的有数据
ros2 topic hz /robot_1/mavros/odom

# 3) TF 是否连通（能打印 Translation 就说明正常）
ros2 run tf2_ros tf2_echo robot_1/map robot_1/base_link
```

第 1 步 count 为 0：仿真没发里程计，或两侧 domain / RMW 不一致（两侧都必须
`ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_fastrtps_cpp`；Cyclone DDS 与 Fast DDS
不能互通）。

第 1 步有发布者、第 2 步没数据：发布者与导航不在同一个 DDS 域，或发布者用了
TRANSIENT_LOCAL 持久性而导航端是 VOLATILE 订阅。

导航端现在会自己把原因打出来，不用只靠上面这条 TF 报错猜：

```
NO ODOMETRY on '/robot_1/mavros/odom' after 11.0 s and no publisher is connected. ...
```

`nav2_input_bridge` 另有备用里程计话题 `local_odom_fallback_topic`（默认 `odom`）：
只有当主话题 `mavros/odom` 上没有发布者、且超过 `odom_timeout_s`（默认 2 s）没有
数据时才会启用，覆盖“仿真直接发到 /robot_1/odom”的情况。

旧 Classic 导航适配入口已移除，仿真与导航统一使用 ZNY 对接链路。

本次修改了 C++，需重新编译 nav2_input_bridge 和 robot_launch。
完整变更位置、原点输入、编译方法与验证范围见 [ZNY 导航对齐记录](../../docs/ZNY_NAV_ALIGNMENT.md)。

将导航代码同步到nx板卡上：
```bash
rsync -avz --itemize-changes \
  --exclude='.git/' \
  --exclude='build*/' --exclude='install*/' --exclude='log/' \
  --exclude='hil_deps/' --exclude='nx_deps/' \
  --exclude='__pycache__/' --exclude='*.pyc' \
  --exclude='deploy/nx/config.env' --exclude='deploy/nx/hil.env' \
  /home/nhy/Go2-w_nav/outdoor_nav/ \
  nhy@192.168.55.1:~/outdoor_nav/
```
