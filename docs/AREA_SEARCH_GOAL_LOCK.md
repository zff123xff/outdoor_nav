# 区域搜索目标承诺（视点锁）

配置：`src/external_pkg/co_search/co_search_nav/config/area_search.yaml` 的 `co_search.parameters`。
实现：`src/external_pkg/co_search/upstream/exploration_manager/src/fast_exploration_manager.cpp`
的 `planExploreMotion`（目标确定之后、`admitTarget` 之前）。

## 问题

单机 co_search 每 `adapter/plan_period_s`（板卡当前 5.0 s）执行一次完整重选：网格 tour、按距离取前
`refined_num` 个前沿、`getViewpointsInfo` 生成候选视点、逐点取最小代价。代价函数
`ViewNode::computeCost` 只有路径长度、速度方向偏角和航向差，没有信息增益项，因此两个代价接近的
候选完全由位姿和地图的微小变化决定胜负。执行端每收到一条 path 就重新下发一次 `FollowPath`，
规划层的目标抖动会直接变成车体在两个前沿之间反复掉头，里程和耗时都被浪费在来回路上。

## 机制

板卡当前 `search_strategy` 为 `parallel_lines`，所以本机制用于切换到 `co_search` 时的前沿策略；不会控制平行扫线端点。
当前 YAML 的 `plan_period_s` 为 5.0 秒（不是旧版本的 1.5 秒）。`planExploreMotion` 在最终目标 `next_pos` 确定后加一层承诺：

1. 承诺窗口内（`viewpoint_lock_sec`）先检查旧目标是否仍然有效：距当前位置大于到达阈值，且
   `2 m` 内仍存在真实前沿质心（`ed_->points_`），否则立即释放。
2. 仍有效时比较两者代价。新候选代价必须低于旧目标的 `(1 - switch_improve_ratio)` 才允许换目标，
   否则继续执行旧目标。
3. 没有保持承诺时写入新目标并刷新窗口；窗口到期后强制重做一次完整决策，避免长期锁死在低价值目标。

目标被 `admitTarget` 拒绝，或 `planTrajToView` 规划失败时立即清除承诺，保证不会反复重试同一个
不可达目标（原有隔离表仍然生效）。

## 参数

| 参数 | 默认（代码） | 当前配置 | 含义 |
|---|---:|---:|---|
| `exploration/viewpoint_lock_sec` | 0.0 | 4.0 | 承诺窗口秒数；0 关闭该机制 |
| `exploration/viewpoint_lock_max_shift_m` | 2.0 | 2.0 | 活性判定半径：旧目标附近多远内还有前沿才算未探索完 |
| `exploration/viewpoint_lock_switch_improve_ratio` | 0.25 | 0.25 | 提前换目标所需的代价改善比例 |
| `exploration/viewpoint_lock_max_dist_m` | 4.0 | 4.0 | 与旧目标的距离参数；以板卡 C++ 实现是否读取为准，当前逻辑中未参与有效性判定 |

抖动仍然频繁就增大 `viewpoint_lock_sec` 或 `switch_improve_ratio`；发现车体不肯离开某个前沿、
反复在同一个低价值目标上打转，就减小两者。窗口应显著小于一段 `local_traj_radius`（当前 6 m）弧长
的执行时间，否则承诺会跨段生效，抵消重规划的意义。

## 日志

engine 日志按周期输出三类标记，可直接统计换目标频次：

```text
[goal_lock][d1] hold target [2.10, -1.30] over [4.80, 2.40] (cost 3.42 vs 3.51, shift 4.53m)
[goal_lock][d1] switch [2.10, -1.30] -> [4.80, 2.40] (cost 3.42 -> 2.10, better than 25%)
[goal_lock][d1] release locked target [2.10, -1.30] (alive=0 dist=4.12m live_gap=6.71m)
```

`hold` 表示压住了一次换目标；`switch` 是新目标达到改善比例；`release` 是旧目标失效或已到达。

## upstream 边界

`upstream/` 是 co-search 原版副本，`UPSTREAM_SHA256.json` 的 545 个文件逐字节校验。
`fast_exploration_manager.cpp` 与 `expl_data.h` 是唯一的算法层有意 fork（视点锁），
已登记在 `test_configuration.py::test_vendored_upstream_sources_match_manifest_except_documented_forks`
的白名单里。manifest 本身不随 fork 改写，便于将来对照原版重新同步；其余算法文件仍要求逐字节一致。

修改后编译 `co_search_nav`（该包把 `fast_exploration_manager.cpp` 编进 `search_engine`）并重启导航生效，
配置是 symlink-install，只改 yaml 不需要重新编译。
