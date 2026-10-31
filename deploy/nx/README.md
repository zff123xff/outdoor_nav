# NX 从空环境部署 outdoor_nav（Ubuntu 22.04 / ARM64 / ROS 2 Humble）

这些脚本在 NX 板上执行。初始化安装和编译不会启动机器人驱动，不发送运动任务。
目前仅在开发机做静态检查，尚未在你的 NX 上安装、编译或连接真机验收。

## 1. 将源码复制到板端

NX 需要已经能启动 Ubuntu 22.04、连接互联网，并有可使用 sudo 的普通用户。
从当前电脑执行，替换登录名和板端 IP：

```bash
rsync -av --exclude build --exclude install --exclude log \
  --exclude build_nx --exclude install_nx --exclude nx_deps --exclude hil_deps \
  /home/nhy/Go2-w_nav/outdoor_nav/ nx用户@NX_IP:~/outdoor_nav/
```

不要复制 `outdoor_nav_sim` 的安装目录或 `deps/mqtt/usr/lib/x86_64-linux-gnu`；本机生成的 `hil_deps/` 也不复制；NX 必须使用 ARM64 库。
板端若没有 rsync，先 `sudo apt-get update`，再 `sudo apt-get install -y rsync`。
建议准备充足的磁盘和内存，默认最多两个编译作业；内存不足可使用 `NX_BUILD_JOBS=1`。

### 后续增量同步

从主机执行以下命令，将 NX_IP 换成板卡地址。只传有变化的源码；不复制主机编译产物、架构相关依赖和板端环境配置。NX 上已单独修改 HIL build.sh 时，保留该脚本：

```bash
rsync -avz --itemize-changes \
  --exclude='.git/' --exclude='build*/' --exclude='install*/' --exclude='log/' \
  /home/nhy/Go2-w_nav/outdoor_nav/ nhy@192.168.56.1:~/outdoor_nav/
```

首次先加 `--dry-run` 查看清单，去掉后实际同步。源目录末尾 `/` 表示同步目录内容。本命令不删除 NX 独有文件；其他同路径源码会被覆盖。若板端还有单独修改的文件，应先备份或补充排除规则。

同步后在 NX 的 outdoor_nav 中执行板端构建脚本，再重启导航。不能同步主机的 install/build 到 ARM64 板卡。需要让板端也删除主机已删除的文件时，可先用 `--delete --dry-run` 检查删除清单，再决定是否应用删除。

## 2. 检查并初始化

SSH 登录 NX，在板端执行：

```bash
cd ~/outdoor_nav
bash deploy/nx/install.sh check
bash deploy/nx/install.sh all
```

`check` 只检查系统/架构、磁盘、内存、网卡及串口。
`all` 安装系统/ROS 依赖、下载源码、编译，不会修改网络 IP，不会设置自启动。
可拆开执行 `install`（依赖）和 `build`（源码编译）。安装失败会立即停止，修复原因后重试。
首次下载 Unitree、Livox SDK2 和驱动使用上游当前版本；重复运行保留已有源码，不自动 git pull。
具体提交记录保存到 `deploy/nx/reports/dependency_commits.txt`，后续批量部署应采用验证过的同一提交。

脚本安装 ROS Humble、Cyclone DDS、MAVROS、GeographicLib 数据、twist_mux、Nav2 依赖、
Foxglove、PCL、Paho MQTT 及 Python 任务依赖；编译 Unitree 消息、MID360 驱动、项目导航和任务包。
Livox SDK2 安装到 `/usr/local`；串口权限通过 dialout 用户组配置，完成后退出登录再登录。
不会刷写 JetPack，不运行系统全面升级，不覆盖 ~/.bashrc，不自动修改静态 IP。

## 3. 填写真实硬件参数

初始化会创建 `deploy/nx/config.env`，重复安装不覆盖此文件。先执行：

```bash
ip -brief address
ls -l /dev/serial/by-id/
nano deploy/nx/config.env
```

| 项目 | 配置位置 | 应填写的内容 |
| --- | --- | --- |
| 机器人命名空间 | `deploy/nx/config.env` 的 ROBOT_NAMESPACE | 例如 robot_a，与任务 agent_id 一致 |
| 狗的以太网接口 | 同文件 GO2W_NET_IFACE | NX 实际网卡名，不能照抄开发机的 enx 名字 |
| 第二网口 | 同文件 GO2W_LAN_IFACE | 可选，用于雷达/远程 ROS 网络；不用则留空 |
| ROS domain | 同文件 ROS_DOMAIN_ID | 与狗固件/实际部署一致，模板 0 需核对；不要沿用仿真的 30 |
| PX4 串口 | 同文件 FCU_URL | 推荐 `/dev/serial/by-id/实际设备:波特率`，必须与飞控配置一致 |
| GPS 参考原点 | `src/robot_launch/config/go2w_nav2.yaml` 中 reference_latitude/longitude/altitude | 实际场地经纬度和海拔；不要照搬仿真参考点 |
| 雷达安装外参 | `src/robot_launch/launch/go2w_nav2.launch.py` 的 static_tf_base_lidar | 实测安装平移与姿态 |
| 雷达/主机 IP | `nx_deps/livox_ws/src/livox_ros_driver2/config/MID360_config.json` | lidar_configs 的雷达 IP 与 host_net_info 的板端接收 IP |

狗网络通常使用 `192.168.123.0/24`，Unitree 官方示例主机地址为 `192.168.123.99/24`。
以实机为准，确保地址未被占用；通过板端 NetworkManager 配置对应有线连接的静态地址，
不要给狗和主机设置同一 IP，不要盲目更改正在承载 SSH 的接口。
MID360 默认网络通常是另一网段，需按设备现有 IP 设置网口/交换机；JSON 的主机 IP 必须确实配置在 NX 上。

项目原 `setup_go2w.sh` 写死了 `~/ww/3rd/unitree_ros2/...`。新入口使用 `deploy/nx/env.sh`，
按项目内 `nx_deps` 加载环境，不依赖旧路径。不要同时 source 仿真的 Fast DDS 环境。
新环境保留已显式设置的 ROS_DOMAIN_ID、RMW_IMPLEMENTATION、CYCLONEDDS_URI；
若继承的 RMW 不是 Cyclone，环境脚本会报错，改用干净终端并确认实际通信设置。

修改 YAML/JSON 后运行一次 `bash deploy/nx/install.sh build` 确保安装资源更新，再重启对应节点。
不把传感器真值或虚假 TF 作为定位输入。真机仍采用原 input_bridge 的平面 TF，
仿真中完整高度 TF 修复没有自动迁移至真机，起伏地形和障碍高度需单独实测。

## 4. 先验证狗的实际 DDS 消息，不控制运动

```bash
cd ~/outdoor_nav
bash deploy/nx/run.sh check
```

它检查 ROS 图并限时读取 `/lf/sportmodestate` 的真实 SportModeState。
必须收到消息，不能仅凭 topic list 判断成功。若固件只发布 `/sportmodestate`，
需核对真实话题，并修改 `go2w_driver.launch.py` 的 state_topic 后重新编译；不要绕过 ready 检查。
此检查不启动 Go2-W 驱动、不发布速度。

## 5. 启动真机导航

北京 ZNY 三话题仿真使用 `bash deploy/nx/run.sh hil`：导航发布 TF，仿真关闭重复定位 TF 发布，默认系统时间，
NX 不要求仿真项目源码或 /clock，公共 TF 由导航根据收到的里程计生成。
该模式不读取真机 config.env，也不启动真实执行器或传感器驱动。
变更、编译和原点设置见 [ZNY 导航对齐记录](../../docs/ZNY_NAV_ALIGNMENT.md)。

先确认狗的站立状态、PX4/GPS/雷达连接与现场条件，然后执行：

```bash
cd ~/outdoor_nav
bash deploy/nx/run.sh nav
```

**这条命令会启动真机执行器驱动；当前 go2w_driver 启动约 2 秒后会发送 BalanceStand 请求。**
它同时启动 MAVROS、MID360、定位中继、导航、twist_mux 和遥控转换，默认不启动 Foxglove。
需要远程查看时可加 `use_foxglove:=true`。不要同时运行原 launch_all_go2w.sh，以免重复节点。

另开终端检查真实传感器与导航就绪：

```bash
cd ~/outdoor_nav
source deploy/nx/env.sh
ns="$ROBOT_NAMESPACE"
timeout 12 ros2 topic echo "/$ns/mavros/state" mavros_msgs/msg/State --once
timeout 12 ros2 topic echo "/$ns/mavros/odom" nav_msgs/msg/Odometry --once --field header --qos-reliability best_effort
timeout 12 ros2 topic echo "/$ns/livox/lidar" sensor_msgs/msg/PointCloud2 --once --field header --qos-reliability best_effort
ros2 lifecycle get "/$ns/controller_server"
ros2 lifecycle get "/$ns/straight_planner"
timeout 6 ros2 run tf2_ros tf2_echo "$ns/map" "$ns/base_link"
```

MAVROS connected=true、收到新鲜 odom/点云、两个导航节点 active、TF 连通且持续更新后才能下发任务。
PX4 需要实际传感器、GPS、EKF 配置和有效原点，安装 ROS 不会自动完成飞控配置。
此项目依赖 PX4/MAVROS 定位，不是只接机器狗自身 IMU 就能直接导航。
若出现生命周期启动超时，需排查并记录；仿真专用的生命周期 relay/watchdog 没有随本部署迁移。

## 6. 任务服务及 MQTT

仅启动原任务服务和 GPS 队列（不会自动发送任务）：

```bash
cd ~/outdoor_nav
bash deploy/nx/run.sh tasks
```

通过 `/agent/robot_a/execute_task` 接收 ExecuteTask 请求。修改命名空间时同步修改 JSON agent_id。
队列原点默认从原项目默认参数读取，必须确认与导航原点一致；新任务追加，不会覆盖旧队列。
需要只改原点或核对完整参数时使用 `ros2 launch ... --show-args`，不要把仿真测试 YAML 的远距离坐标直接用于真机。

本部署会编译原 MQTT 后台 `backend_nodes`，但不自动启用 `mqtt:=true`：
原 `backend_nodes_launcher` 不只启动 MQTT，还启动 ProductGateway 和额外 MAVLink UDP/串口接口；
`multi_task_bridge` 还保留模拟 FCU 状态定时器。启用前应明确实际 broker、vehicle_id、
额外 MAVLink 接口用途，并处理模拟状态发布，不能把它当成真实 FCU 状态。
仿真里验证的 MQTT 协议可以复用，但该薄入口尚未作为独立真机部署组件迁移，
所以本脚本不宣称真机 MQTT 端到端已部署完成。不要将仿真整体加载到真机运行。

## 7. 停止与验证边界

任务队列退出不等于已下发的导航目标被取消；先通过真实的取消/停车链路确认实际停止，
再关闭任务、导航和传感器终端。狗自身急停机制独立保留。
此部署不启用开机自启动，不会在安装后自动执行航点，也不会强制绕过碰撞或 ready 检查。
真机驱动目前 vx_max=1.5 m/s，导航期望速度以真机 YAML 为准；初次运动应先调整为现场允许的速度。

官方参考：

- [ROS 2 Humble 安装](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Unitree ROS 2（Ubuntu 22.04 / Humble 与网卡配置）](https://github.com/unitreerobotics/unitree_ros2)
- [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)
- [Livox ROS Driver2](https://github.com/Livox-SDK/livox_ros_driver2)
