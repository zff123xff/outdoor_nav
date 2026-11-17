# 里程计断流恢复

控制器在每个控制周期检查局部代价地图可用性和机器人位姿的新鲜度。
位姿时间戳与当前 ROS 时钟的差值超过 local_costmap.transform_tolerance 时，
即使 TF 缓存仍能返回旧位姿，也视为暂不可用。

暂不可用时持续发布零速度，保留当前 FollowPath 目标，暂停进展超时计时。
恢复有效位姿且代价地图可用后，重置进展检查基线，再执行原有规划和碰撞检查。
等待期间仍可取消任务，或停止导航栈。等待不设超时，不会自行重发已经失败的任务。
机器人恢复后若仍无法正常移动，原有进展检查会重新超时。

日志仅在进入等待和恢复时各输出一次：

- `Navigation data unavailable: stopping and waiting for fresh robot pose/costmap.`
- `Navigation data recovered: resuming current goal with a fresh progress baseline.`

所有消息时间戳和 ROS 时钟必须使用同一时间基准。若 UE 重置时间而 ROS 未同步，
或另有发布者持续发送同一 base_link 的未来 TF，必须修复源头时间/重复发布问题；
本机制不会伪造 TF 时间戳，也不会重定位已有 ENU 任务。

回归测试（无需机器人；独立 ROS domain，合成 TF，不接真实驱动）：

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source install/local_setup.bash
ROS_DOMAIN_ID=186 PYTHONNOUSERSITE=1 python3 tests/odom_recovery_smoke.py
```

测试启动真实 controller_server，验证断流后零速度、等待超过进展超时仍保留目标、
数据恢复后重新输出控制指令，以及再次断流时能取消任务。

## 区域搜索停在 starting / waiting_odom

若 `co_search/engine_state` 显示点云数量持续增加、`odom_stamp=0`，并报
`lookupTransform argument source_frame ... cannot be empty`，说明里程计消息可能正在到达，
但它的 `header.frame_id` 为空，搜索引擎无法转换位姿。这不是“没有收到 odom”。

co_search 在里程计 frame_id 为空时使用配置的 `odom_frame`，默认
`{namespace}/odom`，生成 native 参数 `adapter/odom_frame`；非空消息头仍按其真实
坐标系转换。该默认值与本项目 nav2_input_bridge 使用的 odom 坐标系一致。
TF 仍须实际存在，转换使用原消息时间戳；不补造位姿或时间，不跳过 TF 检查。
首次发生会输出 fallback 提示，重复日志每 10 秒最多一条。

导航 readiness 丢失是另一条检查：FollowPath Action 必须可用，controller lifecycle state 和 navigation
manager 的服务响应时间都必须小于 3 秒。状态年龄超限会暂停区域搜索，即使日志仍显示 ready=true。
native 引擎心跳则有独立 15 秒 watchdog。有效点云超过 area_search 配置的 3 秒传感器时限会使 native
引擎进入 `waiting_sensor`；搜索已使能时执行器立即暂停，不必等到 15 秒心跳超时。

Nav2 costmap 若报 `earlier than all the data in the transform cache`，表示点云时间戳早于可用 TF 历史；
co-search 的 `tf_lookup_timeout_s` 只延长 native 对精确时间戳的等待，不影响 Nav2 message filter 缓存。

### 搜索入口过滤配置

编辑 `src/external_pkg/co_search/co_search_nav/config/area_search.yaml`：

```yaml
options:
  sensor_timeout_s: 3.0
co_search:
  cloud_filter:
    enable_x_filter: false
    min_x_m: 0.05
    max_x_m: 40.0
    voxel_size_m: 0.2
    accumulation_window_s: 1.0
    min_accumulated_points: 1
    max_future_offset_s: 0.5
    tf_lookup_timeout_s: 2.0
```

x 范围在点云原始坐标系中判断，不是半径。板卡当前关闭 enable_x_filter，故保留各方向点；
TF、时间检查和建图阶段的障碍高度/射线范围约束仍生效。
voxel_size_m 必须正数，min_x_m 必须小于 max_x_m，未来容差不得为负，所有数值必须有限。
参数在每次搜索任务创建引擎时加载；修改 YAML 后重启搜索执行器/导航栈才能读取新文件。
这些不是本次新增的在线动态参数。
