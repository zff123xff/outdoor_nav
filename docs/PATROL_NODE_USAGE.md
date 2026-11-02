# 两点往返巡线

巡线使用 Patrol Action 执行器，单点导航使用 PointGoal Action 执行器。MQTT 与两个手动入口统一调用 `ExecuteTask`；pipeline 按基元类型分发到对应 Action 执行器，并汇总 feedback/result。执行器将 GPS 目标转换为局部坐标后调用 `navigate_to_pose` Action，依据对应目标的结果完成定点或切换巡线端点。

```text
send_patrol ─┐
MQTT → MultiTaskBridge ─┴→ /agent/robot_1/execute_task
  → task_pipeline 调度 → /robot_1/patrol（Patrol Action）
  → patrol_node（负责往返调度与 feedback）
  → navigate_to_pose Action → straight_planner → FollowPath → controller_server
```

巡线启动时使用新鲜里程计与 map→base_link TF 比较两个端点的水平距离，等距时选 A；缺少新鲜定位时拒绝任务，不默认前往 A。到达近端点的接近路程不计为一次往返。每段导航的目标朝向采用该段行进方向，不额外要求端点朝东；进入位置容差后由导航 goal checker 判定到达，不等待末端朝向对齐。

定点和巡线共用 `navigate_to_pose` 导航 Action；巡线循环由 Action 执行器管理。启动巡线前，确认没有其他导航任务正在占用队列。

## 启动与编译

```bash
cd /mnt/d/shixi/south_lake_institution/outdoor_nav
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source install/setup.bash
```

缺少新接口或程序时，在工作区根目录编译：

```bash
colcon build --packages-up-to task_pipeline --symlink-install
source install/setup.bash
```

启动对应机器人的定位、里程计和 Nav2，再启动任务/导航适配节点：

```bash
ros2 launch robot_launch simulation.launch.py robot_namespace:=robot_1
```

该 launch 启动 `task_pipeline` dispatcher（提供 ExecuteTask）、`patrol_node`（提供 Patrol 和 PointGoal 两个执行 Action）及 `multi_goal_to_gp_goal`。MQTT、手动定点、手动巡线都经过同一个 dispatcher；不要另启旧的 `execute_task_server`，否则会造成 ExecuteTask 服务重名。也不要同时运行另一项导航任务。

定点、巡线和通用航点队列只根据对应导航 Action 的成功结果推进；位置容差统一由 `go2w_nav2.yaml` 的 `controller_server.general_goal_checker` 管理，默认使用 PositionGoalChecker，不要求终点朝向；任务层依据成功结果切换目标，不重复做距离计数。失败或取消会中止本次任务，旧目标结果不会完成新目标。自定义导航 YAML 传给总入口：

```bash
bash launch_sim.sh params_file:=/path/to/nav.yaml
```

## YAML 下发

编辑 [patrol_goal.yaml](../src/external_pkg/task_pipeline/config/patrol_goal.yaml)。`coordinate_frame` 选 `gps` 或 `enu`，手动调用节点只使用每个端点中被选中的坐标组。GPS 使用 `lat/lon/alt`；ENU 使用东、北、天方向的 `x/y/z` 米制坐标。ENU 模式下发送器从运行中的 `/<robot>/nav2_input_bridge` 读取参考原点后转为 GPS，不在任务 YAML 重复维护原点；下发 ENU 巡线前必须启动导航栈。

`max_laps: 1` 表示先到距离机器人较近的端点，再到另一端并返回近端后结束（A→B→A 或 B→A→B），`0` 表示无限往返；`hold_time_at_endpoint_s` 是每次到达端点后等待下一目标的秒数，默认 0 表示立即切换。ExecuteTask 服务名由 `robot_namespace` 自动组成 `/agent/<robot_namespace>/execute_task`；pipeline 再调用 `/<robot_namespace>/patrol`。

```bash
# 校验 YAML 并预览，不连接 ROS
ros2 run task_pipeline send_patrol --dry-run

# 只检查 ExecuteTask pipeline 服务是否在线，不提交任务
ros2 run task_pipeline send_patrol --check

# 下发 YAML 中的巡线任务
ros2 run task_pipeline send_patrol
```

也可以传入源码 YAML 或自定义配置路径：

```bash
ros2 run task_pipeline send_patrol src/external_pkg/task_pipeline/config/patrol_goal.yaml
ros2 run task_pipeline send_patrol my_patrol.yaml
```

脚本调用 ExecuteTask 后持续打印 pipeline 转发的 Action feedback。feedback 包含当前阶段、累计到达点数、完整往返趟数及距当前目标的距离；最终消息给出完成/取消状态和最终计数。统一 feedback 话题为 `/<robot>/task_feedback`。也可以直接观察适配器输入与导航目标：

```bash
ros2 action info /robot_1/navigate_to_pose
ros2 topic echo /robot_1/task_feedback std_msgs/msg/String
ros2 topic echo /robot_1/goal_pose geometry_msgs/msg/PoseStamped --once
```

底层 Patrol Action 可用 ROS 2 CLI 调试（会绕过 pipeline；正常手动任务请使用 `send_patrol`）：

```bash
ros2 action send_goal /robot_1/patrol patrol_node/action/Patrol \
  "{mission_id: patrol_cli, agent_id: robot_1, capability_id: patrol_line, primitive_id: patrol_cli_001, start_point: {latitude: 40.9445353, longitude: 113.172819, altitude: 1298.734571}, end_point: {latitude: 40.9446353, longitude: 113.172819, altitude: 1298.734571}, max_laps: 1, hold_time_at_endpoint_s: 1.0, keep_moving_on_end: false}" \
  --feedback
```

## MQTT/通用任务入口

MQTT、定点和巡线手动工具都调用同一个 ExecuteTask 服务。`capability_id=patrol_line` 会调度到 Patrol Action，feedback 经 `task_feedback` 回传；MQTT 模式下转发到 `/mqtt/task_deploy/execution_feedback/<vehicle_id>`。下面是 GPS 格式示例：

MQTT `MultiTask` 中，当前按 `task_type=11` 作为两端点巡线任务处理，要求 `arrive_path.waypoints` 恰有两个 GPS 端点；`task_type=7` 作为普通线性/定点任务处理，单点会分发到 PointGoal。

```bash
ros2 service call /agent/robot_1/execute_task execute_task_server/srv/ExecuteTask \
  '{request_json: "{\"mission_id\":\"patrol_cli\",\"agent_id\":\"robot_1\",\"capability_id\":\"patrol_line\",\"params\":{\"waypoint_frame\":\"gps\",\"waypoints\":[{\"lat\":40.9445353,\"lon\":113.172819,\"alt\":1298.7345711102703},{\"lat\":40.9446353,\"lon\":113.172819,\"alt\":1298.7345711102703}],\"max_laps\":1,\"hold_time_at_endpoint_s\":1.0}}"}'
```

## 取消巡线

运行中的 `send_patrol` 按 Ctrl+C 会通过 ExecuteTask 请求取消当前巡线；执行器停止后 pipeline 回传 canceled result。`max_laps: 0` 会持续往返。

也可以使用 ExecuteTask 发送 `cancel_patrol`：

```bash
ros2 service call /agent/robot_1/execute_task execute_task_server/srv/ExecuteTask \
  '{request_json: "{\"mission_id\":\"cancel_patrol\",\"agent_id\":\"robot_1\",\"capability_id\":\"cancel_patrol\"}"}'
```

取消由 pipeline 转发到 Patrol Action；确认实际速度归零后再启动其他任务。
