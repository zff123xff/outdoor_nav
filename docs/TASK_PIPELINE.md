# 任务接口与执行

本页记录公共任务协议；源码状态、启动模式、执行细节和参数完整表见
[开发交接总览](DEVELOPER_HANDOVER.md)。ROS CLI 示例见 [PRIMITIVE_ROS_EXAMPLES.md](PRIMITIVE_ROS_EXAMPLES.md)。

## 公共入口

发布到 `/<namespace>/task`，类型 `task_interfaces/msg/Task`。命名空间默认 `robot_1`。

```text
uint16 task_id
bool cancel
string[] targets
float32[] waypoints
float32[] center
float64 length_m
float64 width_m
float64 yaw_deg
int16 gait_speed
int16 air_route
```

`task_id` 是固定编号，不存在 `task_type` 字段。当前接受 140、141、142、143、145。GPS 数组按
`[latitude, longitude, altitude]` 重复排列；`center` 是一个 GPS 三元组。坐标经纬度是度、高度是米；
`yaw_deg` 是 ENU 东向逆时针角度。GPS 数组的公共消息类型是 float32，需考虑量化。
`gait_speed` 和 `air_route` 是预留字段，不影响当前执行行为。

| ID | 任务 | 必需字段 |
|---:|---|---|
| 140 | 定点导航 | `waypoints` 一个 GPS 点 |
| 141 | 两点巡线一轮 | `waypoints` 两个 GPS 点 |
| 142 | 区域搜索 | `center`、`length_m`、`width_m`、可选 `yaw_deg` |
| 143 | 目标观察巡检 | 默认目标批次模式无需坐标；区域搜索巡检模式还需 area 字段 |
| 145 | 多航点 | `waypoints` 至少一个 GPS 点，按数组顺序执行 |

调度器只保留一个当前任务。收到普通任务时，若当前任务尚未结束就直接回报 `rejected`；空闲时才校验参数并接收执行。
取消消息只需同编号 `task_id` 和 `cancel: true`；只有编号与当前任务一致才请求取消。只有确认子 Action
终态并完成相关清理后才接收新任务。

## 状态

- `/<namespace>/robot_task_status`：`task_interfaces/msg/TaskStatus`，约 1 Hz。活动任务进度 1–99，终态 100；
  `failures` 表示失败。空闲 ID 为字符串 `"0"`。
- `/<namespace>/task_feedback`：`std_msgs/msg/String` JSON 事件，包含 `accepted`、`executing`、`rejected`、终态及错误信息。
- MQTT 可选输入 `robots/<namespace>/task`，输出 `robots/<namespace>/taskstatus` 和 task_feedback；默认关闭。

只看 progress=100 不能判定成功；还要看 task_feedback 的 `completed`/`failed`/`rejected`。

## 检测类别文件

142 和 143 在对应 Action 真正下发前，把 `targets` 原子写入
`/home/nhy/outdoor_nav_1105/src/yolo_detector/config/detection_targets.yaml`；140、141 和 145 不写。
结束后保留最后一次设置；下一次 142/143 覆盖，空数组使 YOLO 暂停推理。文件写失败时任务不会下发。
YOLO 与 dispatcher 通过 launch 参数 `yolo_targets_file` 使用同一路径。

## 143 说明

143 有 `target_inspection` 目标批次巡检和 `search_inspection` 区域搜索巡检两种代码路径；
后者需提供与 142 一样的区域参数，以 `targets/markers` 触发暂停、接近、到达上报、停留和恢复。
板卡 `simulation.launch.py` 目前将 `simulation_targets` 和 `target_interaction_enabled` 固定传为 true，
dispatcher 优先选择旧的仿真目标交互路径。因此仅设置 `inspection_search_mode:=true` 不能保证进入
`search_inspection`。在 launch 优先级修正并联调前，应把这一模式视为未验证；详见
[仿真目标巡检](SIMULATION_TARGETS.md)。

143 当前仅用于观察和到达上报，不包含武器控制或打击接口。

## 构建

板卡所有权要求是源码及构建产物属于 `nhy`。消息定义变化要一起构建接口包和全部消费者；不可混用旧的
generated type support。接口迁移脚本和构建命令以板卡工作树内的 `scripts/dev/rebuild_task_protocol.sh`、
`build.sh` 为准。不要在 root 环境下直接 build。
