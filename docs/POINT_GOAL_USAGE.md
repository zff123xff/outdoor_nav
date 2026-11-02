# YAML 单点导航

`send_point_goal` 是手动 pipeline 客户端：它从 YAML 读取 GPS/ENU 目标，调用统一 `ExecuteTask` 服务。pipeline 根据单点 `goto_waypoints` 任务调用 PointGoal 执行器 Action，并将执行反馈汇总到统一反馈话题：

```text
send_point_goal ─┐
MQTT → MultiTaskBridge ─┴→ /agent/<robot>/execute_task
  → task_pipeline 调度 → /<robot>/point_goal（PointGoal Action）
  → point_goal_executor（feedback + GPS 目标转换）
  → /<robot>/navigate_to_pose → straight_planner → FollowPath → controller_server
```

先使用 `bash launch_sim.sh` 或 `bash launch_real.sh` 启动完整系统；两种入口都会同时启动导航栈、pipeline、两个 Action 执行器及 `multi_goal_to_gp_goal`。确认没有其他任务占用导航后，编辑配置：

```text
src/external_pkg/task_pipeline/config/point_goal.yaml
```

修改 `robot_namespace`、`mission_id` 和 `coordinate_frame`。选择 `gps` 时填写 `gps.lat/lon/alt`；选择 `enu` 时填写 `enu.x/y/z`，单位米，轴向为东/北/天。ENU 模式下发送器会从运行中的 `/<robot>/nav2_input_bridge` 读取参考原点再转 GPS，不需要在任务 YAML 重复维护原点；因此下发 ENU 任务前必须启动导航栈。默认坐标仅为示例，执行前请替换为现场确认可通行的点。

```bash
ros2 run task_pipeline send_point_goal --dry-run
ros2 run task_pipeline send_point_goal --check
ros2 run task_pipeline send_point_goal
```

运行中会显示 pipeline 转发的执行反馈，到达或失败后显示 Action result。按 Ctrl+C 会通过 `ExecuteTask` 请求取消本任务。支持临时改命名空间和指定配置文件：

```bash
ros2 run task_pipeline send_point_goal --namespace robot_1 --check
ros2 run task_pipeline send_point_goal --file /path/to/point_goal.yaml --dry-run
```

底层 PointGoal Action 可用 ROS 2 CLI 调试（该命令会绕过 pipeline；正常手动任务请使用上面的 YAML 客户端）：

```bash
ros2 action send_goal /robot_1/point_goal patrol_node/action/PointGoal \
  "{mission_id: point_cli, agent_id: robot_1, capability_id: goto_waypoints, primitive_id: point_cli_001, target: {latitude: 40.9446353, longitude: 113.172819, altitude: 1298.734571}}" \
  --feedback
```

MQTT 与手动 YAML 工具都调用 `ExecuteTask`。pipeline 把单点任务调度到 PointGoal Action，并把 feedback/result 发布到 `/<robot>/task_feedback`。MQTT 模式下，`swarms_backend` 转发 JSON 到 `/mqtt/task_deploy/execution_feedback/<vehicle_id>`；该 topic 独立于旧的 UInt8 `stream_feedback`。

可查看 Action 与导航目标状态：

```bash
ros2 action info /robot_1/point_goal
ros2 action info /robot_1/navigate_to_pose
ros2 topic echo /robot_1/task_feedback std_msgs/msg/String
ros2 topic echo /robot_1/goal_pose geometry_msgs/msg/PoseStamped --once
```

RViz 的 **2D Goal Pose** 直接发布本地坐标到 `/robot_1/goal_pose`，不经过 GPS Action 适配链。
