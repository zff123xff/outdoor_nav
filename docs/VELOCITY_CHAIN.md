# 速度指令链路

仿真：导航 YAML -> RPP FollowPath/FollowTarget -> controller_server ->
`/<ns>/cmd_vel_nav` -> twist_mux -> `/<ns>/cmd_vel` -> ros_to_ue_bridge.py -> UE UDP。

- RPP 使用 `desired_linear_vel`（m/s）、`tracking_angular_vel_limit`（rad/s）、
  `rotate_to_heading_angular_vel`（rad/s）。FollowTarget 默认继承 YAML 锚点。
- `backup_recovery_speed` 独立控制自动后退速度（负的 linear.x），不使用前进参考速度。
- 当前 RPP 不读取 YAML 中的 `max_linear_vel`、`max_angular_vel`、`min_linear_vel`、
  `min_angular_vel`、`acc_lim_linear`、`acc_lim_angular`；这些名字不能当成有效限制。
- `/speed_limit` 为命名空间内相对话题。非零绝对值会替换 RPP 的当前参考速度，
  百分比按基础参考速度计算；当前实现没有额外保证其不超过 YAML 基础值。
- controller_server 将 TwistStamped.twist 直接发布为 Twist，没有再次缩放。
- mux 导航优先级 10、遥控优先级 20，超时均 0.5 s；只仲裁，不限幅。
  `cmd_vel_remote` 不受 RPP 参数约束，遥控零速度也会在超时前覆盖导航。
- 外部桥接脚本对机器狗 linear.xyz/angular.xyz 原值序列化，不转换单位或正负号。
  UDP JSON 为 `{"robdog":"robot_1","topic":"/cmd_vel","data":{
  "linear":{"x":...,"y":...,"z":...},"angular":{"x":...,"y":...,"z":...}},"ts":...}`。
  发送成功仅证明包已交给本机 socket，不证明 UE 收包或执行。

真机使用同一个 mux 输出，go2w_driver 限幅后调用 SportClient.Move。
驱动默认 vx_max=1.5 m/s、vy_max=0.6 m/s、vyaw_max=1.0 rad/s；未 ready 时拒收，
超时 0.5 s 调用 StopMove。参数在初始化时读入并由回调捕获，运行中修改不能改变限幅。
Move 接口将 vx/vy/vyaw 放入 Unitree request.parameter 的 x/y/z。
底层运动控制器及 UE 消费端的额外限制/单位转换不在本仓库内，无法靠发送端测试确认。

## 隔离回归

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source install/local_setup.bash
ROS_DOMAIN_ID=183 PYTHONNOUSERSITE=1 python3 tests/velocity_transport_smoke.py
```

默认读取 `/mnt/d/shixi/MavrosBridge/ros_to_ue_bridge.py`，可通过环境变量
`ROS_TO_UE_BRIDGE` 指向其他副本。测试只在内存中将其目标改为 127.0.0.1 临时端口，
不写桥接脚本、不向真实 UE 发包。真实 controller_server、twist_mux 与桥接节点之间
使用实际 DDS 消息，检查 YAML 前进速度、FollowTarget、原地转向、动态改速、
跟踪角速度限制、遥控负速度/负角速度及停车在本机 UDP 包中保持一致。
