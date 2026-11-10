# UE GPS 原点接收

仿真导航默认启动 `robot_launch/simulation_origin_receiver`，不等待任何原点消息。
收到消息前，各模块使用导航 YAML 配置；收到有效消息后，在后台更新 GPS↔ENU 参考参数。
该节点持续运行，后续有效原点消息也会更新；不会修改地图、TF、里程计或重新发送已有 ENU 目标。
已经转换好的目标点、搜索区域保持原来的 ENU 坐标；之后发生的 GPS 转换使用新原点。

## ROS 接口

输入 `/<namespace>/simulation/gps_origin`，类型 `sensor_msgs/msg/NavSatFix`：

- latitude、longitude 为度，altitude 为米；必须是 UE ENU `(0,0,0)` 对应的经纬高。
- 三个字段均需有限值，经纬度范围分别为 ±90°、±180°；STATUS_NO_FIX 消息被忽略。
- 这不是当前机器人 GPS，不订阅 `mavros/global_position/global` 来猜测原点。

桥接的原点发布器为 RELIABLE、TRANSIENT_LOCAL、depth=1，接收节点晚启动也能获取最后一次原点。
接收节点也兼容普通 VOLATILE 发布器，但这种发布器应在接收节点启动后发送。

收到后通过 `set_parameters_atomically` 服务将三个 double 参数一起更新到：

```text
nav2_input_bridge
mavros_odom_relay
point_goal_executor
patrol_node
```

每个节点内部三坐标原子设置；跨节点异步同步，尚未就绪的节点会重试。
全部确认后发布 `/<namespace>/navigation/gps_origin`（同类型，保留最后一条），
区域搜索执行器据此更新后续 GPS 区域转换的原点，不改变正在执行的 ENU 区域。
ENU 多航点通过 PointGoal 进行转换，143 目标巡检本身使用 ENU，不需要改动。
默认每 0.2 秒处理同步，每 5 秒重新确认/写入当前原点以覆盖节点重启后的默认值。
参数只更新运行时，不改写源码 YAML；重启后可再次从桥接保留消息接收。

## 启动

```bash
bash launch_sim.sh
# 禁用后台接收：
bash launch_sim.sh use_ue_gps_origin:=false
# 自定义原点输入话题：
bash launch_sim.sh origin_topic:=simulation/gps_origin
```

实机启动和 `auto_set_gp_origin` 不变；本功能不向飞控发送 set_gp_origin。

ROS 调试发布示例（按实际原点填写，保持与导航相同的 ROS domain）：

```bash
ros2 topic pub --once /robot_1/simulation/gps_origin sensor_msgs/msg/NavSatFix \
  '{latitude: 40.9445353, longitude: 113.172819, altitude: 1298.7345711102703}'
ros2 topic echo /robot_1/navigation/gps_origin sensor_msgs/msg/NavSatFix \
  --qos-durability transient_local --once
ros2 param get /robot_1/nav2_input_bridge reference_latitude
```

## UE→ROS 桥接

现有 `MavrosBridge/ue_to_ros_bridge.py` 需要新增一个输入路由。应用配套工具：

```bash
python3 scripts/dev/patch_mavros_origin.py --bridge-dir ~/MavrosBridge --apply
```

工具先备份原脚本，仅新增 `/simulation/gps_origin` 的输入处理和保留 QoS；
不修改监听/监管脚本、ros_to_ue_bridge.py、原有点云、里程计和速度协议。
重新启动桥接进程后加载新代码。

UE 继续使用现有 UDP 端口和二进制 JSON 包头，仅 JSON 内容增加这一种消息：

```json
{
  "robdog": "robot_1",
  "topic": "/simulation/gps_origin",
  "data": {
    "latitude": 40.9445353,
    "longitude": 113.172819,
    "altitude": 1298.7345711102703
  }
}
```

topic 是不带命名空间的后缀，桥接根据 robdog 生成 ROS 话题。
不能把上面裸 JSON 直接发到 UDP：仍须沿用原桥接的二进制包头。
UE 初始化 ENU 参考时发送一次即可；若切换场景/原点，需要重新发送新值。

## 验证与构建

```bash
bash build.sh --packages-select nav2_input_bridge mavros_odom_relay co_search_nav robot_launch \
  --executor sequential
source install/local_setup.bash
PYTHONNOUSERSITE=1 python3 -m pytest -q tests/test_simulation_origin.py tests/test_ue_origin_bridge.py
ROS_DOMAIN_ID=184 PYTHONNOUSERSITE=1 python3 tests/simulation_origin_smoke.py
```

集成测试使用隔离 ROS domain，运行真实参数服务和 GPS 转换，未启动控制器或发送运动命令。
