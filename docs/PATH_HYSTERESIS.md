# 绕障路径选择滞回

当前定点/巡线导航由 `nav2_straight_planner::StraightPlanner` 发送直线引导路径，
`RegulatedPurePursuitController::getLocalPlan()` 每个控制周期运行 Theta*，平滑和插值后跟踪。
Theta* 每次产生一条提案，没有可复用的候选数组或拓扑类别 ID。
左右绕障代价接近时，点云和代价地图微小变化会使每次的新提案换边；原先没有保存绕障路线。
区域搜索的 co_search 另有 A*/B-spline 管线，但配置关闭拓扑规划；本次没有修改其优化目标。
现有转向模式滞回、搜索区域分配滞回也不等同于绕障路线选择。

## 插入位置和安全

`getLocalPlan()` 完成原来的 Theta*、平滑和插值后，调用 `selectLocalPath()`。
新增 `PathHysteresis` 比较新提案与上一条路线的剩余部分；不修改 Theta* 搜索或优化目标。
缓存路线按机器人当前进度裁剪，局部终点缩短时同步裁剪尾部；端点延伸连接同样需要通过检查。
新旧路径在当前地图上统一重新评分，采用 Theta* 的欧氏距离/障碍物权重和代价变换：

```text
J = integral [w_euc + w_traversal * ((26 + 0.9 * map_cost) / 252)^2] ds
```

单位为加权米，不随离散点数量或地图分辨率改变。
当前默认权重 2、1 下，4 米空闲路径约 8.0426；惩罚 1.0 相当于约 0.497 米空闲路程。
仅当 `J_new + switch_penalty < J_current` 且达到正常切换间隔时才换路线。
同路线的新形状直接更新，保留 ID 和原切换时间。
路线关联使用双向几何投影、顺序检查和跨路线连接碰撞检查，允许小幅抖动/重采样，
避免把薄墙两侧混为一条路线；它是简单的走廊关联，并非完整拓扑分类器。

每次重新检查缓存路线的地图覆盖、致命障碍和完整机器人 footprint。
缓存失效、机器人偏离、目标/坐标系/目标朝向改变、规划失败或原控制器触发碰撞/后退处理时，
取消缓存或立即允许原规划结果接管，不受惩罚和时间锁定限制。
规划失败返回原来的空结果；命令级碰撞检测、停止/恢复和速度限制保持原有行为。
没有安全提案时不会为了保持路线而复活旧缓存。

## 参数与日志

在 `src/robot_launch/config/go2w_nav2.yaml` 的 FollowPath 配置中增加；FollowTarget 继承同一配置。

| 参数 | 初始值 | 含义 |
|---|---|---|
| enable_path_hysteresis | true | 关闭后直接采用原规划提案并清除缓存 |
| switch_penalty | 300.0 | 最终比较时的换路惩罚，加权代价单位 |
| min_switch_interval | 2.0 | 正常换路间隔，秒，使用单调时钟 |
| log_path_switch | true | 输出节流后的路线选择日志 |

支持 controller_server 动态参数，负数、非有限数或错误类型会被拒绝。
如果调整 Theta* 权重，需按新权重重新标定惩罚。

```bash
ros2 param set /robot_1/controller_server FollowPath.log_path_switch true
ros2 param set /robot_1/controller_server FollowPath.enable_path_hysteresis false
```

日志 `path_selection` 包含 current/candidate ID、两个原始 J、惩罚、有效性、是否切换和原因。
`same_route` 表示关联成功；`switch_penalty`/`min_switch_interval` 表示保持；
`current_invalid` 表示立即放开旧路线；`cost_improved` 表示正常换路。
candidate ID 在尚未提交时只是预分配标识；不依赖候选数组下标或对象地址。
日志节流可能省略瞬时事件，主要用于观察持续选择状态。

## 验证

纯逻辑测试覆盖近似/略优/明显更优代价、时间间隔、安全失效绕过锁定、目标变化、
同路线重采样、薄墙隔离、裁剪、动态参数及关闭功能。
真实 Theta* 加合成墙地图的控制器集成测试对比开启/关闭选择滞回。

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source install/local_setup.bash
colcon build --symlink-install --packages-select nav2_regulated_pure_pursuit_controller \
  --executor sequential --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/nav2_regulated_pure_pursuit_controller --output-on-failure \
  -R '^(test_local_plan|test_path_hysteresis|test_regulated_pp|test_path_utils)$'
```

UE 仿真中放置左右均可绕过的墙，让起点和终点位于墙两侧且接近中线。
保持同一目标，先关闭滞回观察提案换边，再打开并记录 current ID 和保持原因。
略微改变左右代价应保持；明显增加当前侧代价且超过两秒应允许换路。
再封堵当前侧，验证无需等待两秒即可放开旧路；两侧都堵住应沿用原停止/恢复行为。
换目标后应显示 initial；恢复相同场景验证关闭后恢复原行为。
本地自动测试使用合成地图和模拟任务服务，不代表已完成 UE 场景实跑。

## Theta* 权重配置

`controller_server.ros__parameters.FollowPath` 支持以下参数，FollowTarget 默认通过 YAML 锚点继承：

板卡当前 FollowPath 配置为 `w_traversal_cost: 5.0`、`w_euc_cost: 2.0`；FollowTarget 通过 YAML 锚点继承。

前者提高经过高代价格子的惩罚，后者提高路径长度惩罚。两者不改变致命障碍的碰撞阈值。
配置加载时和运行中均检查：前者必须为有限非负 double，后者必须为有限正 double。
路径滞回的新旧路线评分读取同一组权重；运行中调整权重会清空旧路线选择缓存。
`switch_penalty` 是独立的绝对惩罚，不会随权重自动缩放；上述权重下，空闲路线
每米评分约为 2.042，应按新的代价量级重新评估切换惩罚。

```bash
ros2 param set /robot_1/controller_server FollowPath.w_traversal_cost 5.0
ros2 param set /robot_1/controller_server FollowPath.w_euc_cost 2.0
```

动态修改只作用于指定控制器实例；要同时修改 FollowTarget，需设置其同名参数。
运行中修改不写回 YAML。`prefer_supplied_path` 保留已有安全引导路径时不会调用 Theta*，
因此权重主要影响实际触发 Theta* 局部规划/绕障修复的阶段，不影响 co_search 自身的搜索规划器。
