# GPS 定点与两点巡线

`task_pipeline` 从 Task topic 接收任务，分别调用 `point_goal`（PointGoal）和 `patrol`（Patrol）Action。执行器将 GPS 转 ENU，逐点调用 `navigate_to_pose`，以匹配目标的导航 Action result 判断完成。统一入口巡线默认一次往返；直接调用 Patrol 可设置 max_laps。

接口、启动及示例见 [任务链路](../../../docs/TASK_PIPELINE.md)。

巡线使用最新 `map → base_link` TF 选择最近端点，不依赖执行器独立收到 odom。
TF 新鲜度按本机单调时钟监测：收到动态 TF 且该变换链的最新公共时间戳推进时更新；
超过 2 秒未推进则拒绝定位。上游时间戳原样保留，不要求它与执行器主机的墙上时钟一致。
