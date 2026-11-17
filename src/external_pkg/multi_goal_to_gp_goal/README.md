# 共享导航客户端

保留历史包名，供 PointGoal 和 Patrol 复用 ENU 坐标转换与 NavigateToPose Action 客户端。旧 MultiGoal topic 执行器已删除。多点任务由 task_pipeline/multi_point_executor 的 MultiPoint Action 顺序调用 PointGoal。
