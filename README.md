# outdoor_nav

Go2-W 的 ROS 2 Humble 导航工作区，支持独立仿真器与实机。任务入口、队列和四类基元执行器都由本仓库启动。

```text
MQTT robots/<namespace>/task ─→ mqtt_task_gateway ─┐
发送基元示例节点 ──────────────────────────────────┴→ /<namespace>/task (Task)
 → task_pipeline FIFO → PointGoal / MultiPoint / Patrol / AreaSearch Actions
 → GPS 转 ENU 目标点或区域探索路径 → Nav2 FollowPath → cmd_vel_nav → cmd_vel
```

## 构建与启动

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
bash build.sh
source install/local_setup.bash
# 独立启动仿真器，保持 ROS domain、命名空间、GPS 原点与本工作区一致
bash launch_sim.sh robot_namespace:=robot_1 use_mavros_bridge:=false
# 或连接实机（需 Go2-W SDK、MID360 和 MAVROS）
bash launch_real.sh robot_namespace:=robot_1
```

`use_mavros_bridge:=true` 时仿真入口可启动外部 UE 狗桥，默认仅启用 `dog`；导航工作区不启动仿真器。需要仿真时钟时传 `use_sim_time:=true`，并确认仿真器发布 `/clock`。不要同时启动仿真与实机导航入口。

开启 MQTT：增加 `mqtt:=true address:=tcp://<broker>:1883`；需要认证时传 `mqtt_username`、`mqtt_passwd`，无内置账号密码。消息是与 ROS Task 字段同构的 GPS JSON。状态回传 `robots/<namespace>/taskstatus`，任务事件回传 `robots/<namespace>/task_feedback`。

## 四类任务示例

```bash
bash run_primitive.sh point_goal --check
bash run_primitive.sh point_goal
bash run_primitive.sh multi_point
bash run_primitive.sh patrol
bash run_primitive.sh area_search
```

示例配置位于 `src/external_pkg/task_pipeline/config/`；支持 `--file`、`--namespace`、`--dry-run`、`--check`、`--cancel`。GPS 配置可离线预览；ENU 示例在发送前向运行中的 `nav2_input_bridge` 查询唯一导航原点，再转换为 GPS。`--check` 仅检查 topic 订阅者，不提交任务。示例等待自身 `task_feedback` 终态；Ctrl+C 请求取消。巡线通过统一入口默认一次往返。

完整消息定义、进度语义、仿真/实机数据链路及验证命令见 [任务接口与数据链路](docs/TASK_PIPELINE.md)。

## 目录

- `src/robot_launch/`：两个系统启动入口、导航参数与 MAVROS GPS 频率配置。
- `src/external_pkg/task_interfaces/`：Task、SearchArea、TaskStatus、MultiPoint 和 AreaSearch。
- `src/external_pkg/task_pipeline/`：topic/MQTT 入口、FIFO 调度器和多点执行器；示例发送及其专用工具集中在 `task_pipeline/examples/`。
- `src/external_pkg/patrol_node/`：PointGoal、Patrol Action 执行器。
- `src/external_pkg/co_search/co_search_nav/`：完整移植的原生 co-search 建图、搜索、路径与 B 样条优化，以及 AreaSearch Action 适配。
- `src/external_pkg/multi_goal_to_gp_goal/`：保留供基元复用的坐标转换和导航 Action 客户端；旧 topic 执行器已移除。
- `src/navigation2/`、`src/driver/`：导航栈和硬件驱动。
- `tests/`、`deploy/`：回归验证与设备部署辅助。

相机检测为可选功能，见 [YOLO](src/yolo_detector/README.md)；硬件配置见 [驱动安装](docs/HARDWARE_DRIVERS_3RD.md)。

检测类别文件、区域搜索期间由 task_pipeline 管理的仿真目标节点及 ROS 接口见 [检测与仿真标记](docs/SIMULATION_TARGETS.md)。
