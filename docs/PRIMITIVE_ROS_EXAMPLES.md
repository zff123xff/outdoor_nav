# 基元任务 ROS 2 示例

所有任务发到 `/<namespace>/task`，类型 `task_interfaces/msg/Task`。先 source 与导航进程相同 ROS 环境；板卡默认
命名空间 `robot_1`。以下 GPS 是示例数据，必须换成当前导航原点附近的合法目标。

```bash
# 140 定点导航：一个 GPS 三元组 [纬度, 经度, 高度]
ros2 topic pub --once -w 1 /robot_1/task task_interfaces/msg/Task \
  '{task_id: 140, waypoints: [40.9446251, 113.1728190, 1298.73]}'

# 141 两点巡线：两个 GPS 三元组，一去一回
ros2 topic pub --once -w 1 /robot_1/task task_interfaces/msg/Task \
  '{task_id: 141, waypoints: [40.9445353, 113.1728190, 1298.73, 40.9446251, 113.1728190, 1298.73]}'

# 142 区域搜索；targets 写入 YOLO 类别文件，yaw 从 ENU 东向逆时针
ros2 topic pub --once -w 1 /robot_1/task task_interfaces/msg/Task \
  '{task_id: 142, targets: [person, car], center: [40.9445353, 113.1728190, 1298.73], length_m: 20.0, width_m: 10.0, yaw_deg: 90.0}'

# 143 默认分配目标巡检：先发布 map ENU 目标批次，再提交任务
ros2 topic pub --once -w 1 /robot_1/targets/assigned task_interfaces/msg/TargetArray \
  '{targets: [{target_id: 1, target_position: {x: 5.0, y: 3.0, z: 0.0}}]}'
ros2 topic pub --once -w 1 /robot_1/task task_interfaces/msg/Task '{task_id: 143}'

# 145 多航点：按输入顺序依次导航
ros2 topic pub --once -w 1 /robot_1/task task_interfaces/msg/Task \
  '{task_id: 145, waypoints: [40.9445353, 113.1728190, 1298.73, 40.9446251, 113.1728190, 1298.73]}'

# 仅取消当前 task_id 相同的任务
ros2 topic pub --once -w 1 /robot_1/task task_interfaces/msg/Task '{task_id: 143, cancel: true}'

# 状态和终态反馈
ros2 topic echo /robot_1/robot_task_status task_interfaces/msg/TaskStatus
ros2 topic echo /robot_1/task_feedback std_msgs/msg/String
```

直接 CLI 发布适用于 GPS 输入任务；ENU 示例应使用仓库 `run_primitive.sh`/发送器，它读取当前导航原点并转换。
`task_id` 是 uint16 整数，不能写成字符串。`targets` 空数组表示检测类别为空。142/143 修改类别文件，
其他任务不修改。143 区域搜索巡检需额外区域字段，但板卡仿真入口当前存在模式选择冲突；见
[SIMULATION_TARGETS.md](SIMULATION_TARGETS.md)。

MQTT 可选输入 topic 为 `robots/robot_1/task`，消息字段与 ROS Task 相同，不应发送 retained 任务。
