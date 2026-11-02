# Gazebo 仿真导航与任务流程

两个工作空间分别构建、分别启动，通过 ROS 接口连接。

- `sim_ws`：Gazebo、场景、模型、传感器、PX4 仿真定位、底层控制；订阅 `/<namespace>/cmd_vel`。
- `outdoor_nav`：Nav2、RViz、速度仲裁、遥控取消和任务 pipeline；消费里程计和点云；发布公共定位与雷达 TF，仿真只发布私有模型 TF。

## 启动

```bash
# 终端 1：sim_ws/simulation 目录（从 sim_ws 根目录执行下面两条命令）
cd simulation
bash start.sh robot_namespace:=robot_1

# 终端 2：outdoor_nav 目录
bash launch_sim.sh robot_namespace:=robot_1 mqtt:=true address:=tcp://127.0.0.1:1883 mqtt_username:="" mqtt_passwd:=""
```

两端保持相同的 ROS_DOMAIN_ID 和命名空间；不设置通信域时使用 ROS 默认域 0，已有 RMW、DDS 配置保持不变。仿真脚本不加载导航环境，导航脚本不加载仿真环境。请用新终端启动，避免继承以前叠加的工作空间。

仿真参数 `world:=agriculture.world`、`gui:=false` 只传给仿真入口；导航参数 `rviz:=false`、`mqtt:=true`、`params_file:=/绝对路径/nav.yaml` 只传给导航入口。不再支持在仿真入口启动导航。

仿真统一使用 `src/robot_launch/config/go2w_nav2.yaml`，其中包含 Nav2、搜索边界和共享到点参数。仿真模型 profile 与导航参数文件分离。GPS 物理原点由 sim_ws 中的 localization 配置管理；其 reference 坐标必须与导航配置一致。原点变更后重启两端，导航参数变更只需重启导航。

## 检查就绪

启动日志正常后，在另一个终端 source 相同环境：

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source /mnt/d/shixi/south_lake_institution/outdoor_nav/install/setup.bash
timeout 10s ros2 topic echo /clock rosgraph_msgs/msg/Clock --once
timeout 10s ros2 topic echo /robot_1/odom nav_msgs/msg/Odometry --once
ros2 lifecycle get /robot_1/controller_server
ros2 lifecycle get /robot_1/straight_planner
ros2 service list | rg '/agent/robot_1/execute_task'
```

默认使用系统时间。只有传入 `use_sim_time:=true` 时才需要检查 `/clock`。确认里程计和 ExecuteTask 服务有实际响应，Nav2 lifecycle 为 `active` 后再下发任务。

## 导航基元直接验证

在 outdoor_nav 目录运行，直接调用 `/agent/{agent_id}/execute_task`，由导航的 pipeline_dispatcher 分发给基元执行器。该入口不需要 MQTT，导航可使用默认的 mqtt:=false。

```bash
bash run_primitive.sh point_goal --dry-run
bash run_primitive.sh point_goal --check
bash run_primitive.sh point_goal
bash run_primitive.sh patrol
bash run_primitive.sh area_search
# 自定义任务 YAML 或目标机器人
bash run_primitive.sh patrol --file /path/patrol.yaml --namespace robot_2
# 通过 pipeline 取消区域搜索
bash run_primitive.sh area_search --cancel
```

默认任务分别是 `src/external_pkg/task_pipeline/config/point_goal.yaml`、`patrol_goal.yaml`、`area_search.yaml`。在 YAML 修改任务参数、mission_id、agent_id 和 primitive_id；`--namespace` 会同步覆盖路由身份。`--check` 仅检查 pipeline 服务可用性；实际执行会输出反馈并等待终态，Ctrl+C 向 pipeline 请求取消。

也可在 source 工作区后直接使用 `ros2 run task_pipeline send_point_goal`、`send_patrol`、`send_area_search`。ENU 定点/巡线从运行中的 nav2_input_bridge 读取原点后提交 pipeline；接收响应不代表任务完成。

## MQTT 基元发布验证

MQTT 5 broker 由外部提供，导航入口的 mqtt:=true 启动接收网关和任务桥；导航还负责唯一的 ExecuteTask 调度器。sim_ws/commands 只加载 YAML、通过 MQTT 发布和确认接收，不启动 ROS 节点、broker 或任务执行器。

从 sim_ws 根目录执行，按需编辑 commands/config/ 中对应 YAML：

```bash
bash commands/setup_mqtt.sh
bash commands/send.sh point_goal --dry-run
bash commands/send.sh point_goal
bash commands/send.sh patrol
bash commands/send.sh area_search
```

连接参数位于 commands/config/mqtt/client.yaml，必须与导航的 broker、vehicle_id、认证信息一致。默认等待导航接收 ACK；--publish-only 只等待 broker PUBACK。接收 ACK 不代表任务完成。定点和巡线的 ENU 坐标由导航接收端使用活动 nav2_input_bridge 的原点转换；区域搜索算法配置保留在导航项目。

## TF 与雷达外参

导航统一发布 `map → odom → base_link → livox_frame`；`odom → base_link` 仅在收到有效里程计时发布。仿真只发布私有模型/关节 TF，不发布公共定位和雷达外参。所有仿真使用同一个导航入口，不提供仿真类型选择。

雷达外参配置在 `src/robot_launch/config/go2w_nav2.yaml` 的 `lidar_mount.ros__parameters` 中：`xyz` 为相对 base_link 的位置（米），`rpy` 为滚转、俯仰、偏航（弧度）。修改后重启导航。自定义 `params_file` 也必须包含此段。
