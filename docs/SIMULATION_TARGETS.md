# 143 目标观察巡检

143 只用于观察并发布到达事件，不控制武器、不发出打击命令。当前代码有两种输入/执行路径；完整 launch
优先级和板卡已知冲突见 [开发交接总览](DEVELOPER_HANDOVER.md)。

## A. 分配目标巡检

输入 `/<ns>/targets/assigned`，类型 `task_interfaces/msg/TargetArray`。每个目标包含正且唯一的
`target_id` 和 `target_position` (`geometry_msgs/Point`)；坐标为相同导航原点下的 map ENU，米。143 Action
启动时快照目标批次，按新鲜机器人位置选最近目标，接近到 10 m 内，转向目标并连续确认距离、朝向和停稳后，
发布 `targets/inspection_arrived`。默认参数为站距 9 m、距离上限 10 m、朝向误差 0.15 rad、线/角速度阈值
0.05、停稳 0.5 s、到达后停留 2.5 s。每个目标失败后继续其他目标，最终汇总失败计数。

输出类型 `task_interfaces/msg/InspectionArrival`：

```text
string task_id
uint64 target_id
geometry_msgs/Point target_position
geometry_msgs/Point robot_position
float64 distance_m
float64 heading_error_rad
```

配置在 `src/external_pkg/task_pipeline/config/inspection.yaml`。同一批次运行中新增分配不重置当前快照；
取消/子导航超时需要等待子 Action 终态后才释放任务通道。

## B. 区域搜索后观察 Marker

`search_inspection_executor` 提供 `search_inspection` Action。该流程调用现有 `area_search` 搜索 Action；收到
新鲜且在 map 坐标系中的 `targets/markers` MarkerArray 后暂停区域搜索，将目标交给目标巡检子 Action，完成
到达上报和停留后恢复同一张搜索地图。按距离合并去重，避免重复巡检同一位置。

`inspection.yaml` 的当前值：`marker_max_age_s=1.5`、`observation_merge_m=1.5`、
`search_service_timeout_s=15`。`SearchInspectionExecutor` 在 `inspection_search_mode` 开启时由 launch 启动；
`search_target_visitor` 使用隔离的 `targets/search_assigned`，不覆盖外部 `targets/assigned`。

**板卡仿真 launch 的已知选择问题：** `simulation.launch.py` 固定将 `simulation_targets` 与
`target_interaction_enabled` 传为 true。Dispatcher 的条件先匹配仿真目标模式，再考虑
`inspection_search_mode`；所以仿真入口下仅加 `inspection_search_mode:=true` 仍会走 A 类仿真目标交互，
不保证走本节的 `search_inspection` Action。修正条件并在板卡验证前，不应把新模式描述成仿真入口可用。

## YOLO 三维观测链

`yolo_detector` 先发布 `yolo/detections` 2D 检测。区域搜索任务 142/143 的 `targets` 写入共用类别 YAML；
仿真 `simulation_target_marker` 订阅二维检测、CameraInfo 和 Livox 点云，使用图像框内的雷达深度点估计 map
位置。有效结果发布 `targets/markers` (`visualization_msgs/MarkerArray`) 和 `targets/found`
(`task_interfaces/msg/TargetEvent`)。Marker 的 `pose.position` 是地图坐标米；`targets/found.position` 是
GPS/NavSatFix。二者接口和坐标含义不同。

YOLO 使用 `yolo.yaml` 的 YAML 相机内参和 TF 相机外参。当前投影窗 1 s、同步容差 0.15 s、传感器时效 1.5 s、
至少 3 个雷达点、最大深度离散 0.5 m。实际可用性依赖图像 header、CameraInfo/内参、TF、点云时间戳和导航
原点一致；具体 TF 错误诊断见 [区域搜索点云缓存与 TF 时间](AREA_SEARCH_CLOUD_WINDOW.md)。

## 示例：输入目标批次

以下直接操作 ROS topic 时需与机器人使用相同 domain 和 namespace：

```bash
ros2 topic pub --once -w 1 /robot_1/targets/assigned task_interfaces/msg/TargetArray \
  '{targets: [{target_id: 1, target_position: {x: 20.0, y: 0.0, z: 0.0}}]}'
ros2 topic echo /robot_1/targets/inspection_arrived task_interfaces/msg/InspectionArrival
ros2 topic pub --once -w 1 /robot_1/task task_interfaces/msg/Task '{task_id: 143}'
```

区域搜索巡检请求需要带 `center`、`length_m`、`width_m` 等区域字段；仅当 dispatcher 实际选择
`search_inspection` Action 时才会使用这些字段。当前仿真入口冲突下需先修复 launch/dispatcher。

## 可选 UDP 桥

`inspection_udp_bridge` 默认关闭，与传感器/速度 MavrosBridge 使用不同协议和端口。启用示例：

```bash
bash launch_sim.sh robot_namespace:=robot_1 inspection_udp:=true \
  inspection_input_port:=9101 inspection_ue_address:=192.168.1.186 inspection_output_port:=9001
```

UE 至少要按 bridge 源码约定发送目标数据报；ROS 到达事件会向配置地址/端口转发。UDP 不提供确认/重发，
需由实际 UE 端验证数据格式和网络收发。ROS topic 直接对接时不需要启用该桥。
