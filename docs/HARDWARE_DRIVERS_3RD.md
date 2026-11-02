# 真机驱动集中安装与加载

统一根目录：`/home/xiaofanfan/ww/3rd`，与已有 Unitree 环境并列。
本机为 Ubuntu 22.04 x86_64 / ROS 2 Humble；这些二进制不能复制到 ARM64 NX 直接执行。

| 路径（相对根目录） | 内容 |
|---|---|
| `unitree_ros2/cyclonedds_ws/install` | 已有 Unitree 消息 |
| `mavros_ws/src`、`build`、`install` | MAVLink、MAVROS、MAVROS Extras 和对应消息 |
| `Livox-SDK2` | SDK 源码、`build` 及独立 `install` |
| `livox_ws/src`、`build`、`install` | MID360 ROS 2 驱动 |
| `GeographicLib/geoids/egm96-5.pgm` | MAVROS 必需的 EGM96 高程数据 |
| `driver_deps` | APR 等补充依赖的 deb 文件及解压目录，不写系统目录 |
| `setup.bash` | 统一加载脚本，不启动节点，不设置 ROS 域或 DDS |
| `installation_logs` | 构建日志和源码 commit 记录 |
| `install_drivers.sh` | 独立的自动下载、编译安装与检查脚本 |

## 自动编译安装

脚本已放在 `/home/xiaofanfan/ww/3rd/install_drivers.sh`，项目内保留同一份源码
`deploy/host/install_drivers.sh`，不依赖 `/tmp` 下的临时文件。

```bash
# 使用本地源码：依赖准备、编译安装、环境生成、安装检查
bash /home/xiaofanfan/ww/3rd/install_drivers.sh

# 允许自动下载缺失的源码
bash /home/xiaofanfan/ww/3rd/install_drivers.sh --download-source

# 已有源码时只增量编译安装并检查
bash /home/xiaofanfan/ww/3rd/install_drivers.sh build

# 只检查安装，不启动硬件节点
bash /home/xiaofanfan/ww/3rd/install_drivers.sh check
```

`prepare` 模式只准备源码、APR 和高程数据。源码默认使用本地文件，不自动克隆；
可显式传 `--no-download-source`，或用 `--download-source` 允许克隆缺少的源码。
这两个开关只控制源码，不控制 APR/高程数据的下载。
重复执行支持增量编译，下载源码使用固定 commit；已有 Git 源码版本不同会报错，
不会强制重置用户文件。也接受完整但不含 `.git` 的源码目录。
不会删除旧工作区、启动设备、修改网卡或调用 sudo。
默认单线程编译以控制内存，可用 `DRIVER_BUILD_JOBS=2` 调整。
默认安装根目录为 `$HOME/ww/3rd`，可用 `DRIVER_THIRD_PARTY=/目标目录` 修改。

该脚本针对已有 ROS 2 Humble/PCL 开发环境的 Ubuntu 22.04 主机，不是空白系统的 ROS 安装器。
本机缺少的 APR 依赖下载为 deb 并解压到安装根目录使用；其他基础依赖需由主机提供。

## 上传源码到 NX 并本机编译

### NX 系统条件

支持 **ARM64 NX + Ubuntu 22.04 + ROS 2 Humble**，在板卡上原生编译。
先在板卡执行 `cat /etc/os-release`、`uname -m`、检查 `/opt/ros/humble/setup.bash`。
若板卡是 Ubuntu 20.04，此脚本会拒绝执行，需要先选择匹配的系统和 ROS 方案。
NVIDIA 的 [JetPack 6.0](https://developer.nvidia.com/embedded/jetpack-sdk-60) 面向 Orin 系列并使用 Ubuntu 22.04；
[JetPack 5.0.2](https://developer.nvidia.com/embedded/jetpack-sdk-502) 使用 Ubuntu 20.04 并包含 Xavier NX 支持。
因此不能仅凭“NX”型号名称判断是否满足条件。
默认单线程适合内存受限环境；构建前保证足够磁盘空间和可用内存，必要时配置 swap。
本次只在 x86_64 主机实际编译验证，尚未在真实 NX 板卡执行。

### 必须传的四份源码

以下路径相对本机 `/home/xiaofanfan/ww/3rd`：

| 本机源码目录 | NX 上的对应目录 | 用途 |
|---|---|---|
| `mavros_ws/src/mavros` | `~/ww/3rd/mavros_ws/src/mavros` | 包含 mavros、mavros_msgs、libmavconn、mavros_extras，需整个仓库 |
| `mavros_ws/src/mavlink` | `~/ww/3rd/mavros_ws/src/mavlink` | 包含 MAVLink 消息定义和 pymavlink 代码生成器，需整个目录 |
| `Livox-SDK2` | `~/ww/3rd/Livox-SDK2` | Livox SDK 源码，含其 3rdparty |
| `livox_ws/src/livox_ros_driver2` | `~/ww/3rd/livox_ws/src/livox_ros_driver2` | MID360 ROS 2 驱动，含 package_ROS2.xml、config、msg 等 |

此外传 `install_drivers.sh`。可选传 `GeographicLib` 数据目录，它与 CPU 架构无关，能免去板卡下载高程数据。
**不要传本机的 `build`、`install`、`log` 或 `driver_deps`**：它们含本机路径和 x86_64 构建/依赖产物，NX 需要自己的 ARM64 版本。

### 在当前电脑执行上传

先把示例中的 NX 用户名和 IP 改成实际值；这些命令只上传，不在板卡启动节点。

```bash
THIRD=/home/xiaofanfan/ww/3rd
NX=ubuntu@192.168.1.50  # 示例：修改为实际 NX 登录账号和 IP

ssh "$NX" 'mkdir -p "$HOME/ww/3rd/mavros_ws/src" "$HOME/ww/3rd/livox_ws/src"'

rsync -az --progress --exclude=build --exclude=install --exclude=log \
  "$THIRD/mavros_ws/src/mavros" "$THIRD/mavros_ws/src/mavlink" \
  "$NX:~/ww/3rd/mavros_ws/src/"

rsync -az --progress --exclude=build --exclude=install --exclude=log \
  "$THIRD/Livox-SDK2" "$NX:~/ww/3rd/"

rsync -az --progress --exclude=build --exclude=install --exclude=log \
  "$THIRD/livox_ws/src/livox_ros_driver2" "$NX:~/ww/3rd/livox_ws/src/"

rsync -az --progress "$THIRD/install_drivers.sh" "$THIRD/GeographicLib" "$NX:~/ww/3rd/"
```

不要加 `--delete`，避免覆盖或删除板卡上已有的其他驱动。上述命令保留 `.git` 元数据，
脚本可以核对源码版本；去掉 `.git` 也可以编译，但无法自动验证 commit。
若 NX 上已有旧源码，先核对其修改与版本，再决定上传位置。

### 在 NX 上准备基础依赖并编译

NX 需先安装 ROS 2 Humble。下列基础依赖操作需要板卡上的 sudo 权限；
它们属于系统开发依赖，驱动本身仍安装在 `~/ww/3rd`。

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git rsync pkg-config \
  python3-colcon-common-extensions python3-rosdep python3-lxml python3-future \
  geographiclib-tools libgeographiclib-dev libapr1-dev libaprutil1-dev

source /opt/ros/humble/setup.bash
# 仅在尚未初始化 rosdep 时执行一次：sudo rosdep init
rosdep update --rosdistro humble

# 先生成 ROS 2 manifest，以便 rosdep 正确识别 MID360 依赖
cp ~/ww/3rd/livox_ws/src/livox_ros_driver2/package_ROS2.xml \
  ~/ww/3rd/livox_ws/src/livox_ros_driver2/package.xml

rosdep install --from-paths \
  ~/ww/3rd/mavros_ws/src/mavlink \
  ~/ww/3rd/mavros_ws/src/mavros/libmavconn \
  ~/ww/3rd/mavros_ws/src/mavros/mavros_msgs \
  ~/ww/3rd/mavros_ws/src/mavros/mavros \
  ~/ww/3rd/mavros_ws/src/mavros/mavros_extras \
  ~/ww/3rd/livox_ws/src/livox_ros_driver2 \
  --ignore-src --rosdistro humble --dependency-types build buildtool exec -y

# 使用已上传源码，不克隆源码；首次需完整编译
bash ~/ww/3rd/install_drivers.sh all --no-download-source

source ~/ww/3rd/setup.bash
bash ~/ww/3rd/install_drivers.sh check
```

如果板卡缺少 APR 或高程数据，`all` 仍可能联网下载它们。
完全离线的板卡需事先准备 ARM64 系统依赖，并上传 `GeographicLib`。
`build` 模式跳过依赖准备，适合依赖齐全后的离线增量编译。

### 如果 NX 还需要 Go2-W 的 Unitree 消息

本脚本只编译 MAVROS 和 MID360，不负责构建 Unitree。
板卡尚无 Unitree 环境时，在电脑上额外上传：

```bash
rsync -az --progress --exclude=build --exclude=install --exclude=log \
  "$THIRD/unitree_ros2" "$NX:~/ww/3rd/"
```

然后在 NX 上单独编译：

```bash
source /opt/ros/humble/setup.bash
cd ~/ww/3rd/unitree_ros2/cyclonedds_ws
colcon build --packages-select unitree_go unitree_api \
  --executor sequential --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

导航项目 `outdoor_nav/src`、配置与启动脚本也需另行上传并在 NX 编译；
仅安装这两套驱动，不会自动安装本项目的导航和 Go2-W 控制节点。

## 加载环境

每个终端执行：

```bash
cd /mnt/d/shixi/south_lake_institution/outdoor_nav
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source setup_hardware.sh
source install/local_setup.bash
```

也可以直接 `source /home/xiaofanfan/ww/3rd/setup.bash` 加载驱动。
`setup_hardware.sh` 默认查找 `$HOME/ww/3rd/setup.bash`，可通过
`OUTDOOR_NAV_THIRD_PARTY` 指向其他根目录。

统一加载脚本设置 `GEOGRAPHICLIB_DATA` 和驱动所需库路径，保留已有
`ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION`、`CYCLONEDDS_URI`。
原真机入口 `setup_go2w.sh` 已接入此加载脚本，其原有的狗网口检查及
Cyclone DDS 设置仍用于真机启动。

确认包来自新目录：

```bash
ros2 pkg prefix mavros
ros2 pkg prefix mavros_extras
ros2 pkg prefix livox_ros_driver2
ros2 pkg executables mavros
ros2 pkg executables livox_ros_driver2
```

## 真机启动前需要填写的配置

- 狗网口：按真实硬件设置 `GO2W_NET_IFACE` 和可选 `GO2W_LAN_IFACE`。
- 飞控：确认串口可见、用户可访问，启动时通过 `fcu_url:=设备路径:波特率` 设置。
- MID360：修改 `livox_ws/src/livox_ros_driver2/config/MID360_config.json` 的主机 IP 和雷达 IP；上游示例是 `192.168.1.5` / `192.168.1.12`，不是本机已验证的硬件地址。项目导航启动文件自动查找驱动包中的这个 JSON。
- ROS 域、DDS 及真实设备网络必须与现场一致；安装驱动不能代替连接验证。
- Foxglove 是另一个可选包，本次不安装；未安装时完整真机启动需要 `use_foxglove:=false`。

本次不修改现有 `/usr/local` Livox SDK，也不删除旧的 `~/mavros_ws` 和 `~/Livox-SDK2`。加载新环境后使用 `/3rd` 下的驱动及 SDK。

安装依据：[MAVROS 上游](https://github.com/mavlink/mavros/tree/ros2)、[Livox ROS Driver 2 上游](https://github.com/Livox-SDK/livox_ros_driver2)。

## 本机验证记录（2026-11-01）

- MAVROS、MAVROS Extras、对应消息和 MAVLink 共 5 个包完成编译安装。
- Livox SDK2 的动态/静态库和 MID360 ROS 2 驱动完成编译安装。
- 正式脚本 `all --no-download-source` 已完整执行，退出码为 0；增量重跑没有下载源码。
- MAVROS/MID360 消息类型支持正常；核心库、标准插件、扩展插件及雷达驱动库通过 `ldd` 和动态加载检查。
- `GeoidEval` 可读取安装目录的 EGM96 数据。
- 脚本语法检查、ShellCheck、本地源码缺失时禁止克隆的检查均通过。
- `patrol_node` 已单独重新编译，Action 类型和 `send_patrol --dry-run` 正常。
- NX 部署入口 `deploy/nx/install.sh` 已将 `patrol_node` 纳入构建目标；项目根目录 `build.sh` 原本已包含该包。

这些检查不启动硬件节点。真实飞控、雷达和底盘通信仍需接线并使用现场网络配置验证；NX 原生编译尚未实测。
