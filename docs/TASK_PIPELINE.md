# 机器狗任务接口与完整数据链路

四类基元的完整 ROS 发布命令、示例脚本及 ENU 用法见 [基元下发示例](PRIMITIVE_ROS_EXAMPLES.md)。

## 入口接口

示例节点和 MQTT gateway 都向 `/<namespace>/task` 发布 `task_interfaces/msg/Task`。默认命名空间 `robot_1`，可靠传输、volatile、depth 10；下发前等待 dispatcher 订阅者发现。任务到达顺序即 FIFO 执行顺序，不抢占当前任务，暂不处理 task_id 冲突。

```text
string task_id
string task_type
bool cancel
string[] targets
sensor_msgs/NavSatFix[] waypoints
task_interfaces/SearchArea area
```

`targets` 是检测类别列表（模型类别名称或类别编号字符串）；空列表表示全部类别。dispatcher 仅在任务从队列取出成为当前任务时，原子改写 `/tmp/outdoor_nav/<namespace>/detection_targets.yaml`，不让后续排队任务提前覆盖类别。YOLO 每秒读取该文件。见 [检测与仿真标记接口](SIMULATION_TARGETS.md)。

`task_type` 为下列四个字符串之一（消息中也提供同名常量）：

| task_type | 解析规则 | 下游 Action |
| --- | --- | --- |
| `point_goal` | waypoints 至少一个点，仅取第一个 | `/<ns>/point_goal`，`patrol_node/action/PointGoal` |
| `multi_point` | waypoints 非空，按顺序全部执行 | `/<ns>/multi_point`，`task_interfaces/action/MultiPoint` |
| `patrol_line` | waypoints 恰好两个点，近端→远端→近端，一次往返 | `/<ns>/patrol`，`patrol_node/action/Patrol` |
| `area_search` | 解析 area，waypoints 可空 | `/<ns>/area_search`，`task_interfaces/action/AreaSearch` |

`SearchArea`：

```text
sensor_msgs/NavSatFix center
float64 length_m
float64 width_m
float64 yaw_deg
```

`center.latitude/longitude` 是 GPS 经纬度（度），`center.altitude` 为米；长宽单位米、至少 1 米。`yaw_deg` 是矩形长度轴相对 ENU 东向逆时针的角度：0° 长轴向东，90° 长轴向北。waypoints 也用 latitude/longitude/altitude。字段必须为有限数值；纬度 ±90°、经度 ±180°。NavSatFix 的 header/status/covariance 不用于目标坐标解析。非区域任务忽略 area；区域任务只使用 area。

GPS topic 下发示例：

```bash
ros2 topic pub --once /robot_1/task task_interfaces/msg/Task \
  "{task_id: point_001, task_type: point_goal, waypoints: [{latitude: 40.9446353, longitude: 113.172819, altitude: 1298.73}]}"
ros2 topic pub --once /robot_1/task task_interfaces/msg/Task \
  "{task_id: search_001, task_type: area_search, waypoints: [], area: {center: {latitude: 40.9446353, longitude: 113.172819, altitude: 1298.73}, length_m: 20.0, width_m: 10.0, yaw_deg: 30.0}}"
```

MQTT 使用相同字段的 JSON，例如：

```json
{"task_id":"point_001","task_type":"point_goal","waypoints":[{"latitude":40.9446353,"longitude":113.172819,"altitude":1298.73}],"area":{}}
```

发布到 `robots/robot_1/task`，QoS 1、retain=false。网关忽略 retained 运动指令；不再支持旧集群 MultiTask/ExecuteTask 协议。取消与普通任务共用 `/<ns>/task`：`cancel` 默认 false，为 true 时只需 `task_id`，其余字段忽略。取消排队项会移出队列，取消执行项等待 Action 终态后释放导航。不存在该 ID 时不影响其他任务。不再使用单独的 `task/cancel` topic。

取消命令示例（不需要 task_type、waypoints 或 area）：

```bash
ros2 topic pub --once -w 1 --qos-reliability reliable --qos-durability volatile \
  /robot_1/task task_interfaces/msg/Task '{task_id: point_001, cancel: true}'
```

MQTT 也使用同一 JSON：`{"task_id":"point_001","cancel":true}`。新增消息字段后，外部 ROS 发送端需要重新构建并 source 新版 `task_interfaces`，然后重启使用旧消息类型的节点。

## 调度与任务状态

调度器只允许一个顶层基元执行；上一 Action 返回成功、失败或取消后，再发下一项。等待 Action 服务的默认期限 30 秒，超时按失败结束；Action 拒绝也结束该项，避免阻塞整个队列。输入非法任务发布 rejected 事件，不进入执行队列。

对外 `/<ns>/robot_task_status`，类型 `task_interfaces/msg/TaskStatus`：

```text
string task_id
int8 progress
int32 failures
float64 param1
float64 param2
float64 param3
```

仅对当前任务以 1 Hz 发布；无任务时不发布。Action 结束时立即额外发布 progress=100，无论成功或失败，然后切换任务。故障数与三个预留参数目前全部为 0。**100 表示任务已结束，不代表成功**；成功与失败查看 `/<ns>/task_feedback` JSON 事件 `completed/failed`。

- 单点：优先使用新鲜有效 GPS，计算起点到当前位置 / 起点到目标的水平距离。GPS 无效或超过 2 秒未更新时，使用 PointGoal 执行器从新鲜 odom 和地图 TF 得到的 ENU 位姿，按同一距离比例反馈；GPS 与 ENU 都不可用时保持已有进度。每种坐标来源分别记录起点，切换时进度不回退。
- 多点：已到达点数加当前路段的接近比例，再除以总点数。
- 巡线：已到达端点数 / 3，初始端点也计一次。
- 区域：矩形内实时点云实际观测单元数 / 矩形单元总数；障碍单元也是已观测区域。

执行中进度单调、上限 99，终态为 100。区域探索在新点云连续确认没有可达边界后结束，可以低于全区域覆盖率；实际覆盖保留在 `area_search/status`。不虚构不可达或遮挡区域的覆盖。

## 仿真链路

`launch_sim.sh` → `simulation.launch.py` → `external` 导航组件 + 公共 pipeline。

1. 外部仿真器或 UE dog 桥提供 `mavros/odom`、`livox/lidar` 和 TF；使用仿真时钟时另提供 `/clock`。本仓库仿真入口不启动 MAVROS，也不发布 GPS 定位；若外部提供 `mavros/global_position/global`，调度器会用它计算单点进度，没有有效 GPS 时用新鲜 odom/TF 的 ENU 反馈计算单点进度；二者均不可用时保持已有进度，Action 结束后为 100。任务 topic 本身仍携带 GPS 目标坐标。
2. `mavros_odom_relay` 将 BEST_EFFORT 里程计转为 `odom` RELIABLE；`nav2_input_bridge` 建立地图、里程计与机器人 TF，并转换雷达输入。
3. `task` → dispatcher → 四种 Action。单点/巡线将 GPS 按导航 YAML 原点投影为 ENU，调用 `navigate_to_pose`；多点 Action 顺序调用 PointGoal。
4. `straight_planner` 将单个 ENU 目标组织为路径并调用 `follow_path`。区域搜索持有导航独占权，只调用完整移植的原生 co-search 建图、边界/视点选择、SNN/A* 路径规划、多项式初始化和 B 样条优化；将原生优化曲线采样为地图路径，直接调用同一 `follow_path`。简化 Python 搜索已移除。
5. `controller_server` 的 RPP 控制器与 costmap 完成跟踪/碰撞检查，`cmd_vel_nav` → 速度仲裁 → `cmd_vel` → 仿真器。
6. 导航 Action result 逐级回传基元与 dispatcher；发布目标本身不判定任务完成。

## 实机链路与 30 Hz GPS

`launch_real.sh` → `real.launch.py`：Go2-W 驱动、MAVROS、MID360、里程计 relay、Nav2、速度仲裁和相同 pipeline。任务处理流程与仿真一致；`cmd_vel` 最终由真实 Go2-W 驱动执行，FCU/MID360 提供真实里程计与点云。

MAVROS 启用 `global_position`、`sys_status`、`command` 插件。`mavros_global_rate` 在连接后调用 `set_message_interval`，请求 MAVLink **GLOBAL_POSITION_INT（33）=30 Hz**；该消息对应融合 GPS `NavSatFix`。对旧 MAVROS 扁平 topic `mavros/global` 重映射至 `mavros/global_position/global`，标准版本直接使用规范 topic。节点监控实际消息收取速率、在断线重连或速率不达标时重试，绝不重复旧 GPS 消息制造 30 Hz。

规范 ROS 2 topic 是 `/robot_1/mavros/global_position/global`；`/robot_1//mavros/...` 含空路径段，不能作为合法 ROS 2 topic。

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source install/local_setup.bash
ros2 topic list --no-daemon --spin-time 5
# 先验证真实消息，再测频率；bounded timeout 到时退出是预期行为
 timeout 8s ros2 topic echo /robot_1/mavros/global_position/global sensor_msgs/msg/NavSatFix --once --qos-reliability best_effort
 timeout 15s ros2 topic hz /robot_1/mavros/global_position/global
```

飞控必须支持该消息速率、有可用全球定位且通信带宽足够；服务 ACK 本身不能证明 30 Hz，需看实际 hz。当前开发环境缺少 MAVROS 节点和实机，实机频率须部署后按以上命令验收。

MAVROS 源码依据：[GPS 插件](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/global_position.cpp)、[消息频率服务](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/sys_status.cpp)。

## 验证

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source install/local_setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/external_pkg/task_pipeline/test src/external_pkg/co_search/co_search_nav/test tests/test_navigation_completion.py tests/test_area_search_backends.py tests/test_simulation_namespace.py
ROS_DOMAIN_ID=173 timeout 35s python3 tests/task_pipeline_smoke.py
```

运行 `ROS_DOMAIN_ID=177 timeout 30s python3 tests/mqtt_ingress_smoke.py` 验证真实 MQTT TCP 消息、网关、调度器以及实际示例发送进程。

另外可运行 `ROS_DOMAIN_ID=174 timeout 40s python3 tests/native_area_search_smoke.py` 验证真实区域搜索 Action、点云/TF、子进程、地图和 FollowPath；运行 `ROS_DOMAIN_ID=175 timeout 18s python3 tests/mavros_rate_smoke.py` 验证消息 33 的速率请求和收取频率监测（模拟 FCU）。

GPS 示例集中在 `task_pipeline/config/examples/gps/`；ENU 示例集中在 `config/examples/enu/`。每组均为 `point_goal.yaml`、`multi_point.yaml`、`patrol_line.yaml`、`area_search.yaml`，用 `--file` 传给对应发送节点。唯一原生算法运行配置为 `co_search_nav/config/area_search.yaml`，任务示例只提供任务几何及类别。

仿真入口启用 dispatcher 参数 `simulation_targets`：仅在当前区域搜索实际发送 Action 时，由 dispatcher 启动 `simulation_target_marker` 子进程；任务终态、取消完成或 dispatcher 退出时停止。子进程异常退出会取消区域搜索并报告失败。其他基元不启动此节点；实机入口默认关闭。YOLO 推理节点仍由其独立启动文件运行。

隔离测试只使用模拟 Action 服务，不连接或控制机器人。旧集群 MQTT backend、任务桥和 ExecuteTask 不恢复；完整原生区域搜索所需计算依赖保留，原集群/飞行入口不构建、不启动。

`timeout 45s python3 tests/robot_task_status.py` 只使用对外 Task/TaskStatus 接口：下发 point_goal，由假执行器在 20 s 内把进度从 0 递增到 99 后成功，检查 TaskStatus 单调递增、约 1 Hz、终态立即补发 100，且总耗时不超过 20 s 预算。

单点 ENU 回退可用 `ROS_DOMAIN_ID=178 timeout 25s python3 tests/point_progress_smoke.py` 验证：测试完全不发布 GPS 定位，只使用隔离的 odom/TF 和模拟导航 Action。内部 `PointGoal.Feedback.progress` 为 ENU 进度，范围 0–99；没有新鲜位姿时为 -1。对外 Task、TaskStatus 接口保持一致。
