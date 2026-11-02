# Pipeline 矩形区域搜索

## 功能与接口

区域搜索通过和定点/巡线相同的调度链路执行，相关程序位于 `src/external_pkg/task_pipeline`：

- `send_area_search`：读取 YAML，通过 `/agent/<ns>/execute_task` 提交任务。
- `area_search`：由 `robot_launch.bringup.pipeline` 启动，默认空闲，收到任务才搜索。

任务链路：

```text
area_search.yaml → send_area_search → ExecuteTask(capability_id=area_search)
 → task_pipeline dispatcher → /<ns>/area_search/request → area_search
 → /<ns>/follow_path → controller_server → cmd_vel_nav → 现有 twist_mux
执行反馈从 area_search 回到 task_feedback，再回到手动客户端或 MQTT 上游。
```

算法参照 co-search 的前沿探索流程：持久观测地图 → 已知空闲/未知交界聚类 → 候选观察点 → 可见性与路径代价评分 → 带重复经过惩罚的八邻域 Dijkstra（禁止斜穿障碍角）→ 局部路径段 → 现有带 Theta* 的纯追踪控制器。没有复制 co-search 的 LKH、多机轨迹广播或 B 样条/MPC。

## 配置

修改 `src/external_pkg/task_pipeline/config/area_search.yaml`。该文件不是 ROS 参数文件，供发送器和 pipeline 的 `area_search_config` 同时使用：

```yaml
robot_namespace: robot_1
mission_id: rectangle_search_demo
coordinate_mode: enu # gps 或 enu
center_enu: {east: 0.0, north: 0.0}
center_gps: {latitude: 0.0, longitude: 0.0, altitude: 0.0}
rectangle: {length_m: 20.0, width_m: 16.0, yaw_deg: 0.0}
```

`length_m` 是矩形局部 X 长度，`width_m` 是局部 Y 宽度，单位米；`yaw_deg=0` 时长度沿东向，正角度逆时针旋转。只读取 coordinate_mode 选中的中心坐标。其余地图分辨率、车体安全半径、传感器范围/地面高度、目标距离、超时和覆盖门槛都在同一 YAML 的 `options` 中。

GPS→ENU 使用与 `nav2_input_bridge` 一致的球面投影。原点从本命名空间**运行中的** `nav2_input_bridge` 参数服务读取，不另外配置一套原点。所有参与机器人必须使用同一 GPS 原点、同一 ENU 方向和时钟基准；任务和分区签名不一致时不会出发。原点/时间基准变更应重启导航和 pipeline，再重新提交任务。

默认 TF 为 `<ns>/map`、`<ns>/base_link`；点云 `livox/lidar` 和 action `follow_path` 使用相对名称。当前支持单层机器人命名空间，例如 robot、robot_1、robot_2。

## nav_run 与分区

`nav2_input_bridge` 随导航栈启动立即发布 `<ns>/nav_run`，类型为 `std_msgs/Bool`，值 true；之后每秒按墙钟重发，QoS 为 reliable + transient_local。它仅表示导航进程存在，不表示 EKF 或控制器就绪。订阅方还必须检查数据超时和发布者是否存在，不能凭历史 true 永久计在线。

搜索节点动态发现同一 ROS domain 内的 `/<ns>/nav_run`，要求收到新鲜 true。列表稳定后按自然顺序排序（robot_2 排在 robot_10 之前），沿矩形长度方向分成等宽条带。每台机器人取与自己排序位置对应的区域。

例如 20×16 m、两台机器人、中心 (0,0)、yaw=0：

- robot_1：X=[-10,0]，Y=[-8,8]。
- robot_2：X=[0,10]，Y=[-8,8]。

所有 nav_run 参与者都要收到同一个搜索任务，交换一致的区域、选项、任务 ID、ENU 原点和名单签名后才执行。每个机器人通过自己命名空间的 pipeline 提交；不要只给 robot_1 提交、却期望 robot_2 自动开始。未提交的参与者会使其他车等待，超时后明确失败。

本次任务固定分区；成员加入/掉线会暂停，需按新名单重新提交。没有执行中自动接管。机器人可在总矩形内穿过别人的条带以进入自己的区域；搜索目标主要位于自己的任务区。各车地图目前独立，不包含 co-search 完整的地图共享和 ID 会车仲裁；近距离安全依赖现有局部感知和碰撞检查。

## 构建

```bash
cd /mnt/d/shixi/south_lake_institution/outdoor_nav
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source install/local_setup.bash
colcon build --packages-select nav2_input_bridge \
  nav2_straight_planner execute_task_server task_pipeline robot_launch \
  --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF
```

## 在现有仿真中运行

不新增或替换场景。保持正在使用的 sim_ws 环境、机器人命名空间、profile、ROS_DOMAIN_ID、DDS 和 GPS 原点一致。下列 robot_1 是示例；现有环境若用 robot，应将所有命令相应改为 robot。只开实际存在的机器人命名空间，不要伪造额外 nav_run。

```bash
# 启动导航和 pipeline；仿真在 sim_ws 中单独启动
cd /mnt/d/shixi/south_lake_institution/outdoor_nav
bash launch_sim.sh robot_namespace:=robot_1

# 另一个同域终端，先预览再按实际需求提交
ros2 run task_pipeline send_area_search --namespace robot_1 --dry-run
ros2 run task_pipeline send_area_search --namespace robot_1
```

多机为每个 namespace 启动相应导航和 pipeline，再向每个 namespace 提交相同 YAML。`--namespace` 只改变任务接收者，不改变全局区域。导航已启动就会发布 nav_run，不需要启动搜索才发布。

真机使用 `bash launch_real.sh robot_namespace:=robot_1` 启动。clock 由启动入口统一设为系统时间；任务 YAML 不配置 `use_sim_time`。GPS 原点、footprint 和限速取导航配置；搜索边界不加载为 costmap 插件。

## 搜索边界与任务互斥

区域边界只约束探索地图的有效区域、分区、前沿目标与搜索路径，不写入导航 costmap，不把区域外设为障碍。`area_search/region` 仅发布原始任务矩形供观察；没有边界插件确认或 `waiting_boundary` 阶段。局部 costmap 保留真实障碍层和膨胀层，局部控制器可以根据真实可行空间绕行。

机器人中心已在总矩形内时直接进行前沿搜索，不设启动内缩门槛；在区域外时先沿最近边/角的方向导航入场，再开始前沿搜索。入场终点按机器人半径和地图分辨率适当内缩，使用导航 YAML 中 `area_entry_goal_checker` 的位置容差（默认 0.10 m）。搜索目标仍留在探索区域内；行驶路径可使用 `outside_margin_m` 指定的外围已观测绕行带，默认向外放宽 2 米。车体碰撞检查和真实障碍膨胀保持有效。入场或搜索无法通过真实障碍时报告失败。

旧的 straight_planner 收到 `area_search/active=true` 后取消自己的旧任务并拒收普通点目标，避免覆盖搜索路径。搜索每秒重发 active 心跳；若搜索进程丢失心跳超过约 4 秒，straight_planner 会取消本命名空间的 FollowPath。此保护要求保留现有 straight_planner 节点，即使搜索时它不再生成直线路径。

搜索执行期间按 `planning_period_s`（默认 0.5 秒）重新评估前沿和剩余路径。当前观察点仍有效时优先保留；新观察点评分至少达到当前的 `frontier_switch_score_ratio` 倍（默认 1.3），且相距至少 `frontier_switch_distance_m`（默认 1 米）才提前切换。当前前沿失效或路径被新观测阻断时也会重规划。机器人距局部路径段终点不足 `path_refresh_distance_m`（默认 1 米）且新路径能继续延伸时，提前更新路径段；入场过程中中心进入搜索矩形后即切换到前沿规划，不等入场终点。先取消当前路径，收到终止结果后从最新位置重新规划并下发，交接期间保持区域搜索所有权；无需等机器人走到旧终点。重规划取消不计入导航失败或前沿冷却。

探索矩形用于目标选择和覆盖率统计，不再强制截断行驶路径。外围带也接收实际点云更新，只有已确认空闲的格子可用于绕行；未知或障碍格不能通过。`outside_cost_weight` 默认 2.0，为外围路段增加软代价，优先使用区域内路径；`outside_margin_m=0` 可关闭外围绕行。局部路径段在区域外结束时从实际位置继续规划，不强制返回入场阶段。外围格子不计入分配面积或覆盖面积。

`min_goal_distance_m`（默认 0.8 米）表示优先选择的目标距离。若没有可用远目标，则允许选择距离不小于 `min_progress_distance_m`（默认 0.2 米）的已知可达观察点，逐步扩展观测；短步同样遵守障碍膨胀、区域边界和视线检查。短路径使用 `short_goal_checker_id`（默认复用导航的 `area_entry_goal_checker`，容差 0.1 米），避免普通 0.5 米容差将短步立即判为到达。调整短步下限时，应保证它大于对应到达容差。

搜索期间不要同时启动巡逻/航点任务。ExecuteTask 拒绝覆盖正在执行的区域任务；`stop_planner=true` 会暂停搜索，不会因遥控停止而自动恢复。取消路径时等待其终止结果，旧 action 回调不会推进下一段。

## 取消与状态

```bash
ros2 run task_pipeline send_area_search --namespace robot_1 --cancel
# 或使用独立取消服务
ros2 service call /robot_1/area_search/cancel std_srvs/srv/Trigger '{}'
ros2 topic echo /robot_1/area_search/status std_msgs/msg/String
```

服务的 success 只表示任务已转发。运行期间 `send_area_search` 会持续打印 pipeline feedback；反馈包含 `phase`、`searched_area_m2`（本机器人分配区内已被地图确认为空闲或占据的唯一栅格面积）、`assigned_area_m2`、`observed_ratio` 和前沿簇数。此面积计入直接观测和完全封闭障碍内的推断填充，两者按格子去重；`direct_observed_area_m2` 与 `inferred_obstacle_area_m2` 分别报告两部分，不是规划路径面积或理论可视范围。终态以 feedback 和 `area_search/status` 为准：`completed` 表示达到 YAML 指定的已观测比例；`partial` 表示无可用可达前沿但仍有未知区域，不能视为全部覆盖。MQTT 模式下同一 feedback JSON 转发到 `/mqtt/task_deploy/execution_feedback/<vehicle_id>`。地图为 -1 未知、0 空闲、100 占据（包含封闭区域推断）；可在 RViz 查看：

- `/<ns>/area_search/map`：RViz 的 Exploration map，白色为空闲已探索、黑色为障碍、灰色为未知。
- `/<ns>/area_search/region`、`assigned_region`（PolygonStamped）：RViz 默认显示橙色总区域、绿色本机分区。ENU 任务接收后立即显示总区域；GPS 任务在导航原点可用后显示。
- `/<ns>/area_search/path`
- `/<ns>/area_search/status`、`assignment`

## 点云观测的限制

点云通常先于对应时间戳的 TF 到达。`map_update_period_s` 默认 0.2 秒，控制点云融合间隔和地图发布间隔；实际频率还受精确时间 TF 等待和计算耗时影响。白色要求车体高度带获得足够空闲证据，不会仅因一条射线经过就变白。任务终态停止融合与地图刷新。搜索器缓存一帧待处理点云，以 0.1 秒周期重试精确时刻的 TF；等待期间不使用最新 TF 替代，也不刷新传感器有效时间。超过 `sensor_timeout_s` 的点云会被丢弃，持续没有有效处理结果仍会暂停任务。超时反馈包含最近的时间戳或 TF 错误，取消 Action 后也保留原始暂停原因。

未改动 sim_ws 和 MID360 插件。当前标准点云仅包含命中点，不包含量程内未命中的射线。搜索地图对实际命中射线做三维空闲更新、对障碍高度带做投影；实测地面落点可确认该 XY 列的地形观测，同帧低障碍优先，已有低障碍证据不被地面或桥顶直接清除；未返回数据不会被当作“整个 5 米圆盘空闲”。地面落点不作为车体高度障碍。每个 XY 栅格统计本帧全部有效回波的最低非地面点：最低点高于净空阈值时，该已观测列可通行；最低点落入车体高度带时保留为障碍。高空结构不会清除地图里已经观测到的低障碍，也不会把结构后方未观测的空间自动标为空闲。桥墩与桥顶分列判断，桥墩仍参与障碍膨胀。

整机高度按 0.30 m、余量按 0.05 m，搜索任务示例的净空阈值为 **0.35 m**、最低障碍高度为 0.05 m。导航的 `obstacle_layer.pointcloud.min_obstacle_height/max_obstacle_height` 独立读取导航 YAML，请按实际车体和噪声情况配置，与搜索任务的对应高度保持一致。

启用 `use_imu_ground_tilt` 时，两张地图共用导航发布的 `ground_height_reference` TF。`ground_tilt_filter` 以 IMU 融合姿态约束局部点云地面拟合，使用空间均匀采样和 RANSAC 排除障碍与高空结构；只有支撑点数、残差和倾角均满足条件时才接受地面。参考系原点的 Z 对齐实测地面，方向经低通滤波，定位 TF 保持原样。导航 YAML 中 `estimate_ground_plane` 开关控制点云拟合；`ground_max_offset_m`、`ground_max_tilt_deg`、`ground_fit_residual_m` 控制拟合范围；`time_constant_s` 默认 0.3 秒，增大更平滑但响应更慢，0 可旁路低通。首帧和长时间间断重新初始化倾角。关闭拟合时仅使用 IMU 车体平面，地面高度偏置需自行标定。`ground_z_m` 与 `height_reference_ground_z` 都相对这个参考系设置，使用拟合地面时为 0。缺少有效点云/TF 时搜索仍会暂停。

因此纯空场或观测不足时可能返回 partial；不能把这一结果解释为搜索算法已经验证覆盖。实物点云、安装 TF、地面高度和导航定位必须有效。没有使用 Gazebo 场景真值建图或真值定位。

## 验证范围

隔离目录编译通过全部相关包；算法/状态机测试覆盖分区、GPS/ENU、前沿、BFS、遮挡、取消和旧回调。隔离 ROS domain 验证真实 nav_run 消息、双机器人 pipeline 转发/分区、掉线暂停和取消；配置与状态机回归检查边界仅用于探索地图、入场无需等待 costmap 确认。另用假 FollowPath action 验证搜索心跳丢失后的取消，不接机器人、不发布速度或伪造定位。

2026-11-02 在现有 agriculture 仿真中实际下发了单机区域搜索任务，保留了真实导航动作、点云、地图与轨迹记录。没有新增场景、导入场景真值地图或伪造定位。验收数值和未观测格子的限制见实际运行记录，不能把纯算法测试当作运动覆盖验收。

搜索路径终点进入导航 YAML 的位置容差后，PositionGoalChecker 返回成功，搜索器据 FollowPath 结果更新下一目标；不等待终点朝向对齐。


## 高代价起点的后退恢复

所有基元任务共用 RPP 的恢复逻辑。机器人中心代价达到 `FollowPath.backup_recovery_trigger_cost`（默认原始 costmap 值 200）时，在生成局部路径或原地转向前检查后方。仅当前 footprint 和整段后退走廊已知且无碰撞、后方存在更低代价落点时，才以默认 0.1 m/s 后退，最大距离 0.5 m。脱离高代价区后重新规划；每条新路径至多完成一次后退恢复。

开关、触发代价、距离和速度在导航 YAML 的 `FollowPath` 下配置。普通低代价膨胀区不触发恢复；真实碰撞、未知区域、后方堵塞或没有距离范围内的退出点时，保持停止并报告失败。后退过程中重新检查走廊，保留原有碰撞检测和任务取消机制。


## 单机路径评分与转向

参考本地 co-search 的 `ViewNode::computeCost` 和 `PickBestUtilityViewpoint`，使用可见前沿收益除以行程与初始转向代价评分。`heading_cost_weight` 惩罚掉头，`revisit_cost_weight` 对 Dijkstra 路径中已经经过的栅格增加软代价；需要返回的通道仍然可走。没有常规可达视点时，残余盲区视点搜索从已知可达位置观察小块未知区域；视线预测只用于选目标，不计入观测面积。

`searched_area_m2`/`observed_ratio` 统计直接观测历史与当前封闭障碍填充的并集。`fill_enclosed_unknown=true` 时，将八邻域完全由黑色障碍包围的未知连通块填黑并计入覆盖率。默认同时启用 `fill_boundary_enclosed_unknown`，允许黑色障碍与探索边界共同围住未知块；这种填充必须接触已观测障碍，且区域内已有空闲观测，避免初始化时整片未知被填黑。接触区域内白色空闲格或存在对角空闲开口的未知块保持未知。填充只作用于搜索地图，不修改导航代价地图。障碍环打开后重新计算推断；直接观测历史保留，两部分重叠只计一次。默认完成门槛仍为 1.0，分母保持完整分配区域。

原地转向按 `rotation_accel_gain * heading_error - rotation_damping_gain * previous_command` 计算角加速度，限幅后积分，并受角速度与停止距离约束。步态引起的 IMU 瞬时角速度不再覆盖上一周期指令。保留 ±π 方向锁定与 45°/30°切换滞回。位置目标检查器不要求终点朝向，导航配置关闭 `use_rotate_to_goal_heading`；`use_rotate_to_heading` 保持开启，路径方向对齐仍可原地转向。跟踪阶段通过 `tracking_angular_vel_limit` 同时约束线速度以保持曲率，避免下游角速度截断而改变实际转弯半径。
