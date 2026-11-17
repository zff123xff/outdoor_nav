# outdoor_nav 开发交接总览

## 交接基线

本页按 2026-11-10 通过 SSH 检查的板卡源码整理，路径为
`/home/nhy/outdoor_nav_1105`。板卡 Git 分支为 `go2w`，HEAD 为 `8a61d7e`；工作区有大量
未提交修改和未跟踪文件，HEAD 不能代表实际源码状态。请在板卡工作区继续开发前先执行
`git status --short` 并保留这些改动，不要用 reset、clean 或整仓覆盖来“同步”。

本次核对读取了板卡 `src/`、配置和 `docs/`，没有检查正在运行的进程、安装空间是否由这份源码构建，
也没有在板卡编译或实机运行。下面描述的是板卡源码当前实现，不等同于部署二进制已经生效。

## 系统边界与启动入口

这是 ROS 2 Humble 的 Go2-W 导航工作区。主要源码位于：

| 目录 | 职责 |
|---|---|
| `src/robot_launch` | 真机/仿真导航入口、Nav2 参数、TF 安装外参和组件组合 |
| `src/navigation2` | Nav2 控制器、局部代价地图、输入桥和路径跟踪改动 |
| `src/external_pkg/task_interfaces` | 公共 `Task`、状态消息和任务 Action |
| `src/external_pkg/task_pipeline` | ROS/MQTT 任务入口、单任务接收、各基元执行器、目标交互 |
| `src/external_pkg/co_search` | 原生区域搜索算法、ROS 适配和配置 |
| `src/external_pkg/patrol_node` | 定点导航、巡线、目标接近及实时 TF 缓冲 |
| `src/yolo_detector` | YOLO 二维检测、雷达点云关联、三维 Marker 和仿真目标辅助节点 |
| `src/external_pkg/rtmp_image_bridge` | 视频流解码及原始/小尺寸图像输出 |
| `src/external_pkg/nav2_input_bridge`、`mavros_odom_relay` | 导航目标输入、里程计 QoS 转发和 GPS 原点转换 |

模拟器属于另一工作空间 `sim_ws`。`launch_sim.sh` 连接已经运行的 UE/仿真输入，不启动 Gazebo/UE；
仿真传感器和速度 UDP 通常由板外的 `~/MavrosBridge` 提供。真机入口为 `launch_real.sh`。
板卡 `go2w_nav2.yaml` 当前 GPS/ENU 参考原点为纬度 40.9445353、经度 113.172819、高度 1298.734571 m。
仿真可由 UE 发 `simulation/gps_origin` 在运行时更新后续 GPS↔ENU 转换；已转换的 ENU 任务和地图不会被重定位。

| 入口 | 实际作用 | 重要默认值/边界 |
|---|---|---|
| `bash launch_real.sh robot_namespace:=robot_1` | 启动 Unitree 驱动、MAVROS、里程计 relay、Nav2、pipeline、twist mux 和原点设置 | 可用 `use_go2w_driver:=false`、`use_mavros:=false` 关闭硬件连接部分；YOLO/视频不是此入口的默认组成 |
| `bash launch_sim.sh robot_namespace:=robot_1` | 启动 Nav2、外部传感器/速度桥、任务链、可选 YOLO/视频、仿真原点接收 | `use_sim_time` 默认 false；`video_stream` 和 `yolo` 默认 true；此入口不启动模拟器 |
| `bash run_primitive.sh <name> --check` | 校验/预览任务请求，不执行运动 | 任务实际执行需显式不带 `--check`/`--dry-run` |

编译前在同一个 shell source ROS 环境和工作区：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

板卡文件应由 `nhy` 拥有。以 root 登录时，使用 `sudo -u nhy ...` 构建，避免 install/build 中出现
root 所有文件。完整构建入口是 `bash build.sh`；脚本默认跳过 `fast_lio` 和 `go2w_driver`。

```bash
sudo -u nhy bash -lc 'cd /home/nhy/outdoor_nav_1105 && source /opt/ros/humble/setup.bash && bash build.sh --executor sequential'
```

只改一个包时可在 `build.sh` 后加 `--packages-select <package>`；消息定义修改需要同时构建
`task_interfaces` 及使用它的包。编译后启动新终端重新 source `install/local_setup.bash`，再重启相关进程。

## 任务协议与执行链

ROS 公共入口为 `/<namespace>/task`，类型 `task_interfaces/msg/Task`。协议当前只保留 `task_id`，不含
`task_type`。固定任务编号如下：

| ID | 名称 | 当前执行方式 |
|---:|---|---|
| 140 | `point_goal` | 一个 GPS 三元组，调用 PointGoal/NavigateToPose |
| 141 | `patrol_line` | 两个 GPS 点，选近端出发、到另一端再返回，一轮 |
| 142 | `area_search` | 中心 GPS、长宽和角度；调用 `area_search` Action，由原生 co-search 规划并用 Nav2 FollowPath 执行 |
| 143 | `target_inspection` | 目标观察巡检。支持分配目标模式和区域搜索巡检模式；仿真入口当前有模式选择冲突，见下文 |
| 145 | `multi_point` | 一个或多个 GPS 三元组，按输入顺序串行执行 |

主要字段：`uint16 task_id`、`bool cancel`、`string[] targets`、`float32[] waypoints`、
`float32[] center`、`float64 length_m/width_m/yaw_deg`，以及预留的 `int16 gait_speed/air_route`。
waypoints 是重复的 `[纬度, 经度, 高度]` 三元组；center 是单个同格式三元组；角度从 ENU 东向逆时针，
东为 0°、北为 90°。GPS 输入为 float32，长距离/高精度需求要考虑量化误差。
`gait_speed`、`air_route` 当前只校验和传递，不改变控制。

Dispatcher 只保留一个当前任务。普通任务在忙碌时直接拒收，空闲时才校验参数并执行；只有同编号取消消息
才能取消当前任务。取消当前任务会等待 Action 进入终态后再释放通道。接口/响应故障导致无法确认子任务
是否停止时，dispatcher 会保留通道，不把不确定的控制请求交给下一任务。

反馈：`/<namespace>/robot_task_status` (`TaskStatus`) 约 1 Hz；空闲 `task_id="0"`，活动任务进度封顶 99，
终态 100，`failures` 表示失败。`/<namespace>/task_feedback` (`std_msgs/String`) 是 JSON 事件，区分
`accepted`、`executing`、`completed`、`failed`、`rejected` 等。应同时看这两个话题，不能只凭 `progress=100`
判断成功。MQTT 网关默认关闭，启用后入口为 `robots/<namespace>/task`。

142/143 在真正派发 Action 前，将 `Task.targets` 原子写入
`/home/nhy/outdoor_nav_1105/src/yolo_detector/config/detection_targets.yaml`。140/141/145 不写该文件；
142/143 结束后也不清空，所以下次任务可覆盖，空数组让检测端待机。文件写入失败会阻止任务派发。

## 区域搜索 142

执行器 `co_search_executor` 负责任务和 Nav2 Action 生命周期；独立 C++ `search_engine` 负责点云入图、地图、
入场路径、搜索路径和优化。它通过 `co_search/path` 把路径交给现有 `FollowPath`，不会另起一套运动控制器。

当前板卡 `config/area_search.yaml` 的活动策略为 `search_strategy: parallel_lines`：在矩形长边方向生成往返
平行线，按机器人当前位置选最近线段并投影接入；后续段交替方向。路线由 `parallel_lines` 一次生成，之后
按端点到达/越过阈值推进。`search_strategy: co_search` 则调用原生 frontier/viewpoint/SNN/LKH/B-spline 搜索。
选择由 YAML 决定，不由任务消息字段决定。入场阶段两种策略共用候选点和安全检查逻辑。

co_search 地图层按各帧采集时刻 TF 转到任务局部坐标；点云先用 base_link 高度带过滤，再通过射线命中/未命中
更新占据概率。原生规划从已知/未知边界提取 frontier，聚类并限额采样候选视点，再用路径长度、方向与路径复杂度
打分，路线由 SNN/A* 生成并经 B-spline 优化。最终路径交给同一 Nav2 FollowPath；当前选择 parallel_lines 时
不会走 frontier/viewpoint 评分和 co_search 的路径复杂度代价。

**当前进度与终止差异：** Action `progress` 和 TaskStatus 仍以 `observed_ratio`（区域内已知栅格占比）为依据；
板卡 `search_engine.cpp` 没有发布 `parallel_endpoints_reached/total`，执行器也没有用扫线端点比计算进度。
因此“已到达扫线端点数/总端点数作为 parallel_lines progress”尚未出现在板卡源码中。
配置 `coverage_threshold: 0.99` 且 `timeout_coefficient_s_per_m2: 1.0` 时，完成需同时满足已知格比例至少
99% 且有效搜索时间严格大于 `1 秒/平方米 × 区域面积`。若 `coverage_threshold <= 0`，执行器改用原生
co-search `exhausted` 连续确认逻辑；对 parallel_lines 的路线耗尽状态是否能满足当前终止条件需实测，不能据此
声称“端点走完一定成功”。

入场：机器人在区域外时，在向内缩 `entry.inset_m` 的四边采样候选并加入机器人投影点，按距离尝试；点位及
优化轨迹都检查原生占据柱和膨胀净空，失败入口进入冷却。默认连续无法生成入场规划 30 秒失败；执行端
FollowPath 中止会报告重选入口，最大执行失败数 4。

### 区域搜索配置（板卡当前值）

来源：`src/external_pkg/co_search/co_search_nav/config/area_search.yaml`。任务级 `co_search` 设置可覆盖相应值。

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `search_strategy` | `parallel_lines` | `parallel_lines` 或 `co_search` |
| `parallel_lines.lane_spacing_m` | 10.0 m | 相邻扫线目标间距；受区域边界尺寸和 margin 限制 |
| `parallel_lines.boundary_margin_m` | 0.8 m | 扫线离区域边界距离 |
| `parallel_lines.waypoint_tolerance_m` | 0.5 m | 端点到达容差 |
| `plan_period_s` | 5.0 s | 原生规划/路径发布周期 |
| `map_period_s` | 0.2 s | 点云窗口地图处理节流周期 |
| `options.sensor_timeout_s` | 3.0 s | 点云/里程计有效性时间门槛 |
| `startup_timeout_s` | 60 s | 未进入可搜索状态前的启动超时 |
| `path_spacing_m` | 0.1 m | 发给 FollowPath 的路径采样间隔下限 |
| `coverage_threshold` | 0.99 | 区域内已知栅格占比完成阈值 |
| `timeout_coefficient_s_per_m2` | 1.0 | 完成前最短有效搜索时长系数 |
| `entry.inset_m / sample_spacing_m / clearance_m` | 0.8 / 0.5 / 0.7 m | 入场边界、采样和净空 |
| `cloud_filter.enable_x_filter` | false | 当前保留各方向点，不启用原始点云 x 裁剪 |
| `cloud_filter.voxel_size_m` | 0.2 m | 输入及窗口合并体素降采样尺寸 |
| `cloud_filter.accumulation_window_s` | 1.0 s | 搜索建图滚动点云窗口 |
| `cloud_filter.min_accumulated_points` | 1 | 窗口内不同体素最少点数 |
| `cloud_filter.max_future_offset_s` | 0.5 s | 点云允许领先节点 ROS 时间的上限 |
| `cloud_filter.tf_lookup_timeout_s` | 2.0 s | 等点云采集时间对应的 TF，不用陈旧 TF 替代 |
| `sdf_map.resolution` | 0.2 m | 原生搜索地图 XY 栅格尺寸 |
| `sdf_map.ground_height` | 0.31 m | 代码以 `base_point.z - ground_height` 计算相对高度基准 |
| `sdf_map.min/max_obstacle_height` | 0.8 / 3.0 m | co-search 点云高度带，独立于 Nav2 costmap |
| `sdf_map.p_hit/p_miss/p_occ` | 0.8 / 0.1 / 0.8 | 占据概率更新和占据阈值 |
| `exploration/path_turn_penalty_s_per_pi` | 100.0 | co_search 前沿路径每累计转角 π 增加的等效秒代价 |
| `exploration/path_corner_penalty_s` | 100.0 | co_search 路径每个 ≥30° 拐弯代价 |
| `exploration/path_reverse_penalty_s` | 100.0 | co_search 路径每个 ≥135° 急转/折返代价 |
| `exploration/path_detour_penalty_s_per_ratio` | 0.01 | co_search 绕行比例代价；绕行比例增加 1.0 时加 0.01 s |
| `exploration/min_path_length_m` | 10.0 | co_search 可达候选路径偏好的最小长度；候选不足时保留可用路径 |
| `frontier/max_candidate_samples` | 24 | co_search 每个前沿最多评估的候选视点样本 |
| `exploration/snn_traj_max_waypoints` | 12 | co_search 送给 B-spline 的 SNN 轨迹最大路点数 |
| `exploration/viewpoint_lock_sec` | 4.0 s | co_search 目标承诺窗口，减少近似等价目标间切换 |
| `exploration/viewpoint_lock_switch_improve_ratio` | 0.25 | 提前切换锁定目标所需的代价改善比例 |

`ground_height` 的正号和 Nav2 的 `height_reference_ground_z: -0.31` 表示方式不同；代码/配置要一起核对，
不要把两处高度带当成共享参数。板卡当前值可能不适用于所有实机安装姿态，需用实际 TF 和地面高度标定。

上述路线复杂度、最小路径长度、候选视点上限和视点锁属于 `co_search` 的 frontier/SNN 规划分支；当前 YAML
选择 `parallel_lines` 时，平行线几何路径不使用这些前沿候选代价。当前 10×10 m 示例区域、margin 0.8 m、
lane spacing 10 m 会产生 2 条扫线，实际中心线间距约 8.4 m；小区域覆盖分辨率要通过 `lane_spacing_m` 调整。

**传感器掉线行为：** 引擎只在成功完成点云坐标转换并入缓存后更新 cloud 接收时刻。点云超过
`sensor_timeout_s` 没有有效更新时，引擎进入 `waiting_sensor`；执行器在搜索已使能状态下收到该 phase 会立即
把任务设为 `paused`。因此点云有效数据掉线约 3 秒可能使 142 暂停，并非要等 15 秒 native heartbeat。
15 秒是另一条引擎心跳 watchdog。导航 readiness 是第三条独立检查：FollowPath Action 可用，且 controller 和
navigation manager 的最新状态响应年龄均小于硬编码 3 秒。

## Nav2 与点云避障

区域搜索的 native 地图高度过滤与 Nav2 local costmap 高度过滤是两套配置：

| 子系统 | 当前配置 | 作用 |
|---|---|---|
| Nav2 local costmap | `height_reference_frame: base_link`，`height_reference_ground_z: -0.31`，障碍带 0.2–1.5 m | 点云障碍标记/清除和机器人局部避障 |
| Nav2 点数累计 | 每平面格最少 5 点，窗口 3.0 s；源 observation persistence 3.0 s | 降低单点噪声的障碍标记 |
| Nav2 地图 | 0.1 m 分辨率、20 Hz 更新、50 m × 50 m 滚动区域 | 控制器局部碰撞检查 |
| 膨胀层 | 半径 2.0 m、cost scaling 1.0 | 机器人与已标障碍保持代价距离 |
| FollowPath/FollowTarget | RPP，`desired_linear_vel=1.5` m/s，`lookahead_dist=0.6` m，`allow_backward=false` | 搜索/任务路径跟踪 |
| progress checker | 15 s 内平移 0.1 m 或转动 0.2 rad 算有进展 | 防止控制器长时间原地不动 |

costmap 观察缓存消息因 TF 过旧而被 message filter 丢弃，与 co-search 用 2 秒等待精确 TF 的逻辑不同；
前者不能靠调整 `cloud_filter.tf_lookup_timeout_s` 修复。日志若为 “earlier than all data in transform cache”，
应检查点云延迟、时间基准和 TF 发布/缓存历史；若为 extrapolation into the future，则检查点云和 TF 时间戳
是否同步，并确认 co-search 的 timeout 配置已被实际构建加载。

## YOLO 与目标三维坐标

仿真 launch 默认启动 RTMP 视频桥与 YOLO。YOLO 订阅 `/<ns>/front_camera/image_small`（640×360），
发布 `/<ns>/yolo/detections` 二维检测和调试图；
RTMP 桥保留 1920×1080 原始图，YOLO 按配置的 `expected_width/height` 还原检测坐标。

`yolo_detector/config/yolo.yaml` 是 YOLO 的内参与点云参数来源；相机内参采用 YAML 数值，默认标定尺寸
1920×1080，fx/fy=960 px、cx/cy=(960,540)，畸变系数当前为零。模型输入 640×480；模型 letterbox 与
相机原始标定尺寸不是同一概念。相机外参来自 TF：`go2w_nav2.yaml` 的 `camera_mount` 默认开启静态 TF，
位置 `[0.4, 0, 0.18]` m，旋转 `[-π/2,0,-π/2]` rad 用于水平朝前 optical frame 轴变换。
位置是否符合真实安装尚需硬件确认；必须与图像 optical frame 一致，不能把相机内参当外参。

三维投影流程：对每个图像检测时刻，选取时间窗内 Livox 点云；每帧按自己的 TF 时间戳分别变换到 map 和
检测图像时刻相机坐标，投影到图像检测框，在框内做深度筛选和稳健位置估计，最后发
`/<ns>/targets/markers` (`visualization_msgs/MarkerArray`)；目标位置在每个 Marker 的 `pose.position`，单位 m，
坐标系为 `/<ns>/map`。默认点云窗 1.0 s、图像/最近云时间同步容差 0.15 s、有效传感器时效 1.5 s、至少
3 个雷达点、深度离散上限 0.5 m、同类跟踪 ID 距离 1 m。

仿真 launch 同时启动 `yolo_detector`（配置 `publish_target_markers=true`）和
`simulation_target_marker`；两者源码都向 `targets/markers` 发布 MarkerArray。可能出现同一目标的重复 Marker，
需确认是否有意保留双投影器，或关闭其中一个发布器。`simulation_target_marker` 另会发
`targets/found`（`task_interfaces/TargetEvent`，位置字段为 GPS/NavSatFix）；
`targets/request` 是目标辅助交互请求。`targets/markers` 不是 `/targets/found` 的别名。
目标类别文件为空时 YOLO 暂停推理；节点每秒重读，配置文件缺失会初始化为空目标集。

## 143 模式：实现与 launch 配置冲突

代码中存在两条 143 路径：

1. `target_inspection`：消费 `targets/assigned` 目标批次，按最近目标执行接近、对准、停车确认，发布
   `targets/inspection_arrived`，停留后继续。配置在 `task_pipeline/config/inspection.yaml`。
2. `search_inspection`：另一个 Action，以区域搜索 142 的区域/策略启动搜索；`MarkerArray` (`targets/markers`)
   到达时暂停 `area_search`，通过 `targets/search_assigned` 交给观察巡检子 Action，完成后恢复同一搜索。
   参数同样在 `inspection.yaml`：marker 最大时效 1.5 s、位置去重距离 1.5 m、搜索/暂停/恢复服务超时 15 s。

板卡 `simulation.launch.py` 当前无条件把 `simulation_targets=true` 和 `target_interaction_enabled=true` 传给
pipeline；`dispatcher.py` 先计算 `simulation_observation`，当它为真时优先选择 `area_search` + 旧
`TargetInteraction`，只有它为假时才选择 `inspection_search_mode` 的 `search_inspection`。因此在该仿真入口
下仅设置 `inspection_search_mode:=true` 不足以启用新扫线巡检流程；需修正 launch/dispatcher 的模式优先级并
验证后再把此能力视为可用。这里是源码核对发现的待处理项，不是推测运行结果。

`simulation_target_marker` 是仿真视觉辅助节点，订阅 YOLO 二维框、CameraInfo、点云；有效三维观测发 Marker
及 GPS `targets/found`。其 `TargetApproach` 用于观察接近/对准确认，不是武器控制。

## 启动、诊断和构建

板卡常用检查（先 source 与启动进程相同 ROS domain 的环境）：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 topic echo /robot_1/robot_task_status task_interfaces/msg/TaskStatus
ros2 topic echo /robot_1/task_feedback std_msgs/msg/String
ros2 topic echo /robot_1/area_search/status std_msgs/msg/String
ros2 topic echo /robot_1/co_search/engine_state std_msgs/msg/String
ros2 topic hz /robot_1/livox/lidar
ros2 run tf2_ros tf2_echo robot_1/map robot_1/livox_frame
```

如果从 `/robot_17` 执行任务，上述话题和 TF 前缀都换成 `robot_17`。诊断优先次序：

1. `task_feedback` 是否 `executing`，Action 是否接受；
2. `area_search/status` 的 `state/reason`、`engine_phase`、`search_elapsed_s` 和 `observed_ratio`；
3. `co_search/engine_state` 的 `phase/reason`、`cloud_frames/cloud_window_points` 和最近 TF 错误；
4. `navigation readiness lost` 中 FollowPath Action 与 controller/manager 状态年龄；
5. local costmap message filter 的时间错误。点云 TF 错误会让相应帧无法转换，不要用旧 TF 强行代替；
   `earlier than all data` 与 `extrapolation into the future` 是不同方向的问题。

源码支持的低风险检查：

```bash
bash run_primitive.sh area_search --coordinate-frame enu --check
ros2 topic info /robot_1/targets/markers
ros2 topic echo /robot_1/targets/markers visualization_msgs/msg/MarkerArray
```

构建时以 `nhy` 用户执行并只构建受影响包。任务接口改动需把 `task_interfaces` 和消费者一起构建；区域搜索
改动构建 `co_search_nav`；Nav2 costmap/controller 改动构建相应 Nav2 包及 `robot_launch`；YOLO 配置/实现改动
构建 `yolo_detector`。构建后 `source install/local_setup.bash` 并重启目标节点。不要把 build 成功当作板卡
运动/仿真验证成功。

## 源码与文档维护准则

- 先看板卡 `git status --short`：该工作区包含大量未提交实现，所有者此前要求文件不归 root。
- 配置查实际入口传入的 YAML，不只查默认值。仿真、真机、co-search、Nav2 和 YOLO 各有独立配置。
- 修改消息定义后同步构建所有通信端；禁止混用旧 install 中生成的 type support。
- 新增 `parallel_lines` 端点进度、修正 143 launch 模式冲突、统一/验证两套障碍高度基准，都是明确待处理事项。
- 仿真/硬件联调结果应写入日期化 validation 记录；现存 `docs/validation/` 文件是历史记录，不自动代表本次板卡状态。

专题文档索引见 [README](README.md)。
