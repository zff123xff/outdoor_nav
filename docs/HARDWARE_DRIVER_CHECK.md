# 本机真机驱动检查

本文保留安装前检查结果。后续集中安装路径及环境加载方式见 [真机驱动集中安装与加载](HARDWARE_DRIVERS_3RD.md)。

检查日期：2026-11-01；当前机器为 x86_64 Linux、ROS 2 Humble，不是 ARM64 Jetson NX。
本次仅检查安装、链接、环境和设备可见性，没有启动硬件节点，没有发送底盘、站立或舵机指令。

## 结论

本机尚不具备完整真机启动条件。Go2-W 驱动及 Unitree 消息已构建，Livox SDK 已安装；MAVROS 主程序、MID360 ROS 驱动和 CAN 厂商库缺失。没有验证到真实机器人通信。

| 组件 | 检查结果 |
|---|---|
| `go2w_driver` | 工作区有 `go2w_driver_node` 可执行文件；`ldd` 未发现缺失库 |
| `unitree_go` / `unitree_api` | 已安装于 `/home/xiaofanfan/ww/3rd/unitree_ros2/cyclonedds_ws/install`；显式加载后 Python 消息及类型支持正常 |
| Cyclone DDS | `ros-humble-rmw-cyclonedds-cpp` 已安装，动态库可加载 |
| `remote_override` | 可执行文件存在，`ldd` 未发现缺失库 |
| MAVROS | 当前仅找到系统 `mavros_msgs`；未找到 `mavros` 包及主程序。`/home/xiaofanfan/mavros_ws` 只有源码，没有构建安装目录 |
| GeographicLib | 工具已安装；默认 `/usr/share/GeographicLib` 为空，未找到 MAVROS 使用的大地水准面数据 |
| Livox SDK2 | `/usr/local/lib/liblivox_lidar_sdk_shared.so` 可加载，SDK 头文件及静态库存在 |
| MID360 ROS 驱动 | 未找到 `livox_ros_driver2` 安装包；项目 `src/driver/livox_ros_driver2` 是空目录，只有 SDK 不能发布 ROS 点云 |
| `twist_mux` | 当前可找到，但来自相邻 `sim_ws/install/twist_mux`，不是本项目或系统的独立安装 |
| `foxglove_bridge` | 未找到；可视化可选组件，真机启动默认启用，缺失时需显式关闭或安装 |
| `ros2can` | 包存在，但只有 `control_publisher` 和 `control_motion`。CMake 缓存显示 `CANBUS_LIB-NOTFOUND`；`car_driver`、`control_subscriber`、`odm_battery_publisher` 被跳过，没有实际硬件执行程序 |
| `servo_triger_control` | 包和 `servo_triger_node` 已安装。当前默认 PWM 路径存在，但设备为本机 Intel `INTC1055:00`，不能当成 NX 舵机接口；没有导出的 `pwm0`，当前用户也无 export 写权限，未做硬件动作验证 |

## 环境和硬件情况

- 单独加载 ROS helper 及本项目 `install/setup.bash` 不会自动找到 Unitree 消息。检查时显式加载 Unitree 工作区后，`SportModeState` 和 `Request` 类型支持均正常。
- 原 `setup_go2w.sh` 会加载上述 `~/ww/3rd` 路径，并强制使用 Cyclone DDS；还要求填写真实狗网口 `GO2W_NET_IFACE`。检查过程中没有执行该脚本修改 DDS 设置。
- `deploy/nx/config.env`、`nx_deps`、`install_nx` 不存在，因此当前不能直接使用 NX 真机部署入口。`deploy/nx/install.sh` 限定 ARM64 NX，不能原样在本 x86_64 机器执行。
- 当前继承环境中 `ROS_DOMAIN_ID` 和 `RMW_IMPLEMENTATION` 均未设置；未擅自切换域或 DDS。这不能证明真实机器人使用默认域。
- 当前可见网络：`enp4s0` 为 DOWN、无 IP；`wlo1` 为 `192.168.78.120/20`；没有配置在常见狗网段 `192.168.123.0/24` 的接口。真实狗网络仍需按现场设置确认。
- 当前环境没有可见 `/dev/ttyACM*`、`/dev/ttyUSB*` 或 `/dev/serial/by-id`；无法验证飞控串口连接。当前进程组列表也没有 `dialout`，上真机后还需核对设备权限。
- UDP socket 可创建；有界 ROS 发现仅看到 `/parameter_events` 和 `/rosout`，没有已运行的硬件节点或实际机器人数据可用于验证。未收到真实状态、odom 或点云，不能判定硬件通信正常。
- 当前 shell 的 `python3` 无法导入 `pymap3d`；这是部分部署检查依赖，需根据最终选用入口补齐。

## 后续补齐顺序

1. 按实际使用平台安装或构建 MAVROS 主程序，并安装其 GeographicLib 数据。
2. 保留现有 Livox SDK2，补齐 `livox_ros_driver2` ROS 2 源码及构建安装，设置 MID360 配置 JSON 的雷达和主机 IP。
3. 为真机环境提供可独立加载的 `twist_mux`；按需安装 Foxglove 或关闭其启动。
4. 如使用 CAN 轮式底盘，安装匹配当前架构的厂商 `libcanbus` 后重新编译 `ros2can`；Go2-W DDS 底盘不依赖这套 CAN 驱动。
5. 如果使用舵机，在真实目标板核对 PWM 控制器、引脚复用、供电、频率及用户权限；不能沿用开发机的 PWM 设备判断。
6. 接好狗网线、雷达及飞控后，再基于实际网卡、ROS 域、DDS 和串口设置进行只读收数验证。

注意：`go2w_driver` 启动后默认约 2 秒主动发送 `BalanceStand`，不是纯接收程序。因此本次没有通过启动它来测试是否可执行。
