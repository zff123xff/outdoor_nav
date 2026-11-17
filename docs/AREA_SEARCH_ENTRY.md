# 区域搜索入场、进度与完成条件

权威实现见 [开发交接总览](DEVELOPER_HANDOVER.md)，配置位于
`src/external_pkg/co_search/co_search_nav/config/area_search.yaml`。

## 入场选择

机器人在任务矩形外时，native 引擎先生成入场点：将矩形边界按 `entry.inset_m` 内缩，在四条边上按
`sample_spacing_m` 采样，并加入机器人在内缩矩形内的投影点；按与当前位置距离排序，单周期最多尝试
`max_plan_attempts` 个候选。候选点及优化后的整段轨迹都按 `clearance_m` 检查原生占据柱；失败候选进入
`retry_cooldown_s` 冷却。进入区域后还要抵达已选入口附近约 0.25 m，才转入区域策略。

连续无法生成入口计划超过 `timeout_s` 会进入 `entry_failed`。入场 FollowPath 执行失败会通知 native 引擎
拒绝当前入口并换点，达到 `max_execution_failures` 后失败。这里复用原生轨迹优化；未知区域仍遵循原 planner
策略，不能把未知当作已验证安全。Nav2 footprint 和最后碰撞检查不变。

板卡当前值：`inset_m=0.8`、`sample_spacing_m=0.5`、`clearance_m=0.7`、`max_plan_attempts=4`、
`retry_cooldown_s=5`、`timeout_s=30`、`max_execution_failures=4`。`inset_m` 必须大于 `clearance_m`。

## 覆盖进度和终止条件

Python 执行器在 native occupancy grid 中裁取任务矩形，已知格定义为栅格值 `>=0`，并按任务格子数计算
`observed_ratio`。当前 Action feedback 和公共 TaskStatus progress 都是 `observed_ratio × 100`（未终态上限 99），
不是距离、轨迹长度、已走扫线长度或 YOLO 检测数。

当前板卡配置 `coverage_threshold=0.99`、`timeout_coefficient_s_per_m2=1.0`。成功需同时满足：

- 区域已知格比例 `observed_ratio >= coverage_threshold`；
- 从 native map ready 并启动搜索计时后，搜索时间严格大于 `区域长 × 区域宽 × timeout_coefficient_s_per_m2`。

例如 10×10 m 区域须搜索超过 100 秒且已知格达到 99%。显式暂停期间不检查该完成条件。

将 `coverage_threshold` 设为 0 会关闭基于覆盖率的完成判断，改为仅当 native `co_search` 报告可达前沿耗尽并
连续达到 `exhaustion_confirmations` 后结束。parallel_lines 已经扫完端点时会报告 `exhausted`，但当前执行器
仍未用端点比作为进度；高覆盖阈值未达标时，扫线耗尽不保证任务成功。见 handover 中的待处理项。

`area_search/status` 可观察 `searched_area_m2`、`assigned_area_m2`、`observed_ratio`、
`coverage_threshold`、`search_elapsed_s` 和 `search_time_limit_s`。`area_search/map` 只发布任务矩形栅格。

## 关键判停条件

- 搜索启动后，有效点云超过 `options.sensor_timeout_s=3.0` 未更新：native phase 为 `waiting_sensor`；使能状态下
  执行器立即 `paused`。
- 导航 readiness：FollowPath Action 可用，controller lifecycle state 与 navigation manager 响应都须在 3 秒内更新。
- native 引擎心跳超过 15 秒：执行器暂停任务。
- 搜索启动等待超时默认 60 秒；这是未使能时的启动超时，不是整个任务的时限。

## parallel_lines 当前行为

策略从配置 `search_strategy` 读取，板卡当前为 `parallel_lines`。它按矩形较长边作为扫线方向，结合
`lane_spacing_m`、`boundary_margin_m` 生成蛇形端点，选距机器人最近的路线段作起始投影，之后按端点到达或
越过阈值推进。发出的路径仍由现有 FollowPath 控制器跟踪。

板卡当前 `lane_spacing_m=10.0`、`boundary_margin_m=0.8`、`waypoint_tolerance_m=0.5`。端点统计字段/进度算法
尚未实现；现在的用户可见进度仍是 occupancy grid 覆盖率。不要在交接或验收中把两者说成已经一致。
