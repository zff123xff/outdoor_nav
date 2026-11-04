# 区域搜索检测与仿真标记

## 生命周期与配置

`task_pipeline` 管理 `simulation_target_marker` 子进程：当前区域搜索的 Action 服务就绪并发送任务时启动；任务成功、失败、拒绝或取消完成时停止，调度器退出时也清理子进程。子进程异常退出会触发当前搜索取消，并将任务报告为失败。排队任务及其他三类基元不启动该节点。

`launch_sim.sh` 自动启用 dispatcher 的 `simulation_targets` 参数；实机入口默认关闭。YOLO 推理节点仍独立启动，以便使用目标机器的 GPU Python 环境：

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source install/local_setup.bash
ros2 launch yolo_detector yolo.launch.py namespace:=robot_1 config_file:=/absolute/path/to/yolo.yaml
```

算法唯一运行配置为 `src/external_pkg/co_search/co_search_nav/config/area_search.yaml`。任务示例在 `task_pipeline/config/examples/gps/` 和 `examples/enu/`；检测投影配置在 `yolo_detector/config/target_marker.yaml`。

## 检测类别

`task_interfaces/msg/Task` 新增 `string[] targets`，如 `targets: [bottle, chair]`。类别必须是模型支持的名称或编号字符串；空列表代表全部类别。取消消息忽略此字段。

dispatcher 在任务出队成为当前任务时原子改写 `/tmp/outdoor_nav/robot_1/detection_targets.yaml`（可通过 `targets_file` 参数自定义，所有消费者须一致）：

```yaml
task_id: area_demo
targets: [bottle, chair]
```

YOLO 每隔 1 秒读取一次文件，过滤检测结果与调试图；无效文件保留上一有效类别并报错。排队任务不提前改变类别。任务结束后保留最后一次选择，下个任务开始时覆盖。

## 三维定位与目标 ID

节点订阅 `yolo/detections`、`front_camera/camera_info`、`livox/lidar` 及 `area_search/active`。需要真实相机内参、畸变参数，以及相机光学坐标系、雷达、地图之间的 TF；图像与雷达时间戳必须可比。默认时间差不超过 0.15 秒，传感器超时 1.5 秒。

将实际雷达点按对应采样时刻的 TF 投影到图像，在检测框中筛选至少 3 个一致深度点，再转为地图 ENU，最后使用 `nav2_input_bridge` 的导航原点转换成经纬高。缺少有效标定、深度、TF 或原点时不发布伪造坐标。GPS 高度等于导航原点高度加 ENU up。

同类别目标按地图三维距离关联（默认 1 米），一帧内一对一匹配。新目标从 ID 1 递增，同一目标跨帧保持 ID；每次区域任务新建节点，因此 ID 在该任务内有效。每个目标最多每秒发布一次，首次立即发布。

## 统一事件消息

发现与请求均为 `task_interfaces/msg/TargetEvent`：

```text
std_msgs/Header header
sensor_msgs/NavSatFix position
string target_type
uint64 target_id
string command
```

`position.latitude/longitude` 单位度，`altitude` 单位米，`header` 为观测时间与地图坐标系；位置字段仍是 GPS，不能按 header 将其当成 ENU。`command` 为 `target found` 或 `request shoot`，消息提供 `TARGET_FOUND`、`REQUEST_SHOOT` 常量。

| 话题（robot_1） | 类型 | 发送方与用途 |
| --- | --- | --- |
| `/robot_1/task` | `task_interfaces/msg/Task` | 外部或示例节点下发含 targets 的任务 |
| `/robot_1/yolo/detections` | `yolo_interfaces/msg/DetectionArray` | YOLO 发布二维检测 |
| `/robot_1/targets/found` | `task_interfaces/msg/TargetEvent` | 管理节点发布已发现目标 |
| `/robot_1/targets/request` | `task_interfaces/msg/TargetEvent` | 外部发送仿真标记请求 |
| `/robot_1/targets/markers` | `visualization_msgs/msg/MarkerArray` | 管理节点发布 RViz 标记 |

发现、请求、标记话题均 RELIABLE、VOLATILE、depth 10。目标请求只接受当前任务中最近观测到的已知 ID 和仍选中的类别，坐标使用节点已存观测；未知或过期 ID 忽略。`request shoot` 仅将 RViz 球形标记由绿色改为红色，无实体打击输出。请求中的 GPS 不用于控制任何设备。

```bash
# 先观察真实发现消息，取其中 target_id，再请求仿真标记。
ros2 topic echo /robot_1/targets/found task_interfaces/msg/TargetEvent
ros2 topic pub --once -w 1 /robot_1/targets/request task_interfaces/msg/TargetEvent \
  '{target_id: 1, command: "request shoot"}'
```

四类任务下发命令见 [PRIMITIVE_ROS_EXAMPLES.md](PRIMITIVE_ROS_EXAMPLES.md)。在区域搜索示例消息中增加 `targets: [bottle]` 即仅检测该类别。外部发送端须重编译新版 `task_interfaces`。

## 验证

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source install/local_setup.bash
ROS_DOMAIN_ID=179 PYTHONNOUSERSITE=1 timeout 50s python3 tests/simulation_targets_smoke.py
```

该测试使用 CPU 假推理模型和真实 ROS 消息，覆盖任务类别文件、1 秒重读、图像/点云/TF 投影、GPS、稳定 ID、仿真请求、排队隔离及 dispatcher 子进程启停。实际 GPU engine、相机雷达标定和定位精度需在目标仿真部署中验证。
