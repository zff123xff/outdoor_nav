# GPS 航点队列

保留历史包名 `multi_goal_to_gp_goal`。订阅命名空间内 `waypoint_goals`（MultiGoal），按导航 YAML 的 GPS 原点转换目标，逐点调用 `navigate_to_pose` Action。

队列仅在当前目标返回 SUCCEEDED 后推进；失败或取消清空剩余队列并停止。没有基于距离、里程计采样次数的独立到点判断。巡线队列仍支持端点停留和有限/无限往返。

定点和 Patrol 执行器直接使用同一导航 Action，不再绕经此队列。外部单 GPS 目标仍可通过 `gp_goal → nav2_input_bridge → goal_pose` 接入。

导航完成由 controller_server 的 goal_checker 判断，经 FollowPath 结果传到 NavigateToPose。所有消费者使用目标专属的 Action 结果，过期结果不会推进新目标。导航参数只读取 robot_launch/config/go2w_nav2.yaml 或总入口指定的 params_file。
