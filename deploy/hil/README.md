# NX 接入仿真

北京 ZNY 接口使用：

```bash
ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash deploy/nx/run.sh hil
# 等价入口
ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash launch_hil.sh
```

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
