# patrol_node

手动定点、手动巡线和 MQTT 任务统一进入 `/agent/<robot>/execute_task`。`task_pipeline` 根据 `capability_id` 将单点任务调度到 `point_goal_executor`，将 `patrol_line` 调度到 `patrol_node`；执行器将 GPS 转为 ENU 并逐点调用 `/<robot>/navigate_to_pose`；仅收到对应目标的成功结果才完成定点或切换巡线端点。Action feedback/result 由 pipeline 汇总到 `/<robot>/task_feedback`，MQTT 模式下转发到 `/mqtt/task_deploy/execution_feedback/<vehicle_id>`。

```bash
ros2 run task_pipeline send_patrol
```

`patrol_node/action/Patrol` 与 `patrol_node/action/PointGoal` 是 pipeline 内部的执行器接口。手动任务请使用 `ros2 run task_pipeline send_patrol` 或 `send_point_goal`，避免绕过调度层。不要与其他导航任务同时控制同一机器人。
