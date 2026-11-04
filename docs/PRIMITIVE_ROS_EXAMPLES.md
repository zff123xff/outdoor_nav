# 四类基元任务 ROS 下发示例

## 环境与启动

以下命令在仓库根目录执行，使用 `robot_1`。外部发送端需要安装并 source `task_interfaces`，与导航端使用相同的 ROS domain，并确保 DDS 通信可达。

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source install/local_setup.bash
# 外部仿真器已独立启动时，另开终端启动导航；不需要 MQTT。
bash launch_sim.sh robot_namespace:=robot_1 use_mavros_bridge:=false
```

若使用外部 UE dog 桥，按部署配置启用 `use_mavros_bridge`。只有仿真器提供 `/clock` 且需要仿真时钟时，才加 `use_sim_time:=true`。

下面是实际提交运动任务的命令。先将坐标改成目标场景中的位置；示例 GPS 与仓库当前导航 YAML 原点附近的坐标一致，修改导航原点后要同步调整目标。多个任务会按到达顺序入队执行；同一个任务选择一种下发方式即可，重复执行命令会重复提交。

## 统一接口

入口：`/robot_1/task`，类型：`task_interfaces/msg/Task`。

```text
string task_id
string task_type
bool cancel
string[] targets
sensor_msgs/NavSatFix[] waypoints
task_interfaces/SearchArea area
```

`targets` 为检测类别字符串列表，如 `targets: [bottle, chair]`；省略或空列表表示全部类别。任务轮到执行时才更新检测类别文件；YOLO 每秒读取一次。完整检测与仿真标记接口见 [SIMULATION_TARGETS.md](SIMULATION_TARGETS.md)。

`cancel` 省略时默认为 false，正常提交任务；为 true 时只按 `task_id` 取消，任务类型和坐标不参与解析。

`waypoints` 的点使用 `latitude`、`longitude`、`altitude`：经纬度为度，高度为米。`area` 包含 `center`（同样为 GPS 点）、`length_m`、`width_m`、`yaw_deg`。长宽至少 1 米；方向角为矩形长轴从 ENU 东向逆时针旋转的角度，0° 向东，90° 向北。与默认值相同的未使用字段可省略。

命令显式使用 RELIABLE、VOLATILE、depth 10，`--once` 仅发布一次；`-w 1` 等待至少一个匹配订阅者。发布命令退出表示下发结束，任务结果应通过反馈话题确认。

## 1. 定点导航：point_goal

只执行 `waypoints[0]`，至少提供一个点。

```bash
ros2 topic pub --once -w 1 \
  --qos-reliability reliable --qos-durability volatile --qos-depth 10 \
  /robot_1/task task_interfaces/msg/Task \
  '{task_id: point_ros_001, task_type: point_goal, waypoints: [{latitude: 40.9446353, longitude: 113.172819, altitude: 1298.7345711102703}]}'
```

通过示例脚本发布相同类型的 GPS 任务：

```bash
bash run_primitive.sh point_goal --namespace robot_1 \
  --coordinate-frame gps
# 等价的 ROS 节点调用
ros2 run task_pipeline send_point_goal --namespace robot_1 \
  --coordinate-frame gps
```

默认配置 `examples/enu/point_goal.yaml` 当前选择 ENU；`bash run_primitive.sh point_goal` 使用它。

## 2. 多点导航：multi_point

按 `waypoints` 数组顺序执行；某一点失败会结束本项多点任务。

```bash
ros2 topic pub --once -w 1 \
  --qos-reliability reliable --qos-durability volatile --qos-depth 10 \
  /robot_1/task task_interfaces/msg/Task \
  '{task_id: multi_ros_001, task_type: multi_point, waypoints: [{latitude: 40.9446353, longitude: 113.172819, altitude: 1298.7345711102703}, {latitude: 40.9446400, longitude: 113.172825, altitude: 1298.7345711102703}]}'
```

示例脚本和等价节点命令：

```bash
bash run_primitive.sh multi_point --namespace robot_1 \
  --coordinate-frame gps
ros2 run task_pipeline send_multi_point --namespace robot_1 \
  --coordinate-frame gps
```

## 3. 两点巡线：patrol_line

要求恰好两个点：`waypoints[0]` 是 A，`waypoints[1]` 是 B。先到较近端点，再去另一端并返回初始端点，默认完成一次往返。脚本选择名称为 `patrol`，消息中的 `task_type` 为 `patrol_line`。

```bash
ros2 topic pub --once -w 1 \
  --qos-reliability reliable --qos-durability volatile --qos-depth 10 \
  /robot_1/task task_interfaces/msg/Task \
  '{task_id: patrol_ros_001, task_type: patrol_line, waypoints: [{latitude: 40.9445353, longitude: 113.172819, altitude: 1298.7345711102703}, {latitude: 40.9446353, longitude: 113.172819, altitude: 1298.7345711102703}]}'
```

示例脚本和等价节点命令：

```bash
bash run_primitive.sh patrol --namespace robot_1 \
  --coordinate-frame gps
ros2 run task_pipeline send_patrol --namespace robot_1 \
  --coordinate-frame gps
```

ENU 巡线示例位于 `config/examples/enu/patrol_line.yaml`，GPS 示例位于 `config/examples/gps/patrol_line.yaml`。统一 Task 接口不包含往返次数或端点停留参数，文件中的旧扩展参数不会经该接口下发。

## 4. 区域搜索：area_search

使用 `area`；`waypoints` 可为空。下面是 GPS 中心、20 米长、10 米宽、长轴逆时针旋转 30° 的矩形。

```bash
ros2 topic pub --once -w 1 \
  --qos-reliability reliable --qos-durability volatile --qos-depth 10 \
  /robot_1/task task_interfaces/msg/Task \
  '{task_id: area_ros_001, task_type: area_search, waypoints: [], area: {center: {latitude: 40.9446353, longitude: 113.172819, altitude: 1298.7345711102703}, length_m: 20.0, width_m: 10.0, yaw_deg: 30.0}}'
```

示例脚本和等价节点命令：

```bash
bash run_primitive.sh area_search --namespace robot_1 \
  --coordinate-frame gps
ros2 run task_pipeline send_area_search --namespace robot_1 \
  --coordinate-frame gps
```

默认配置 `examples/enu/area_search.yaml` 当前使用 ENU 中心；`bash run_primitive.sh area_search` 使用它。统一 Task 消息携带几何区域及检测类别；原生建图、搜索和优化参数由导航启动时区域搜索配置的 `co_search.parameters` 控制。

## 示例节点源码位置

示例入口集中在 `src/external_pkg/task_pipeline/task_pipeline/examples/sender.py`，四个 ROS 可执行命令映射到该文件的四个入口函数。`examples/coordinates.py` 负责 ENU 转 GPS，`examples/topic_client.py` 负责查询原点、发布任务、等待反馈和取消。目录调整后，`ros2 run task_pipeline send_*` 和 `run_primitive.sh` 的调用方式保持一致。

## 示例脚本的检查、预览与取消

四类脚本都支持 `--coordinate-frame enu|gps`，自动选择 `config/examples/<坐标类型>/<任务类型>.yaml`。显式 `--file`（或位置文件参数）优先，坐标解析以所选 YAML 内容为准。默认值保持：定点和区域为 `enu`，多点和巡线为 `gps`。ENU 仍在发送前转换成 GPS Task。

```bash
bash run_primitive.sh point_goal --coordinate-frame enu --namespace robot_1
bash run_primitive.sh multi_point --coordinate-frame gps --namespace robot_1
bash run_primitive.sh patrol --coordinate-frame enu --namespace robot_1
ros2 run task_pipeline send_area_search --coordinate-frame gps --namespace robot_1
```

四类脚本还支持以下选项：

```bash
# 只检查 task topic 的订阅者与反馈发布者，不提交任务
bash run_primitive.sh point_goal --namespace robot_1 --check
# GPS 示例可离线预览转换后的消息，不发布任务
bash run_primitive.sh point_goal --dry-run \
  --coordinate-frame gps
# 按所选文件中的 task_id 取消任务，须与原下发文件保持一致
bash run_primitive.sh point_goal --namespace robot_1 --cancel \
  --coordinate-frame gps
```

示例节点发布一次后等待自身任务的反馈终态；Ctrl+C 会请求取消。也可直接通过同一个任务 topic 指定 task_id 并置 cancel=true：

```bash
ros2 topic pub --once -w 1 --qos-durability volatile \
  /robot_1/task task_interfaces/msg/Task '{task_id: point_ros_001, cancel: true}'
```

## 现有 ENU 支持

现有示例节点支持 ENU 配置，不需要修改代码。但 `/robot_1/task` 的线上消息始终是 GPS，不能把 ENU 的 x/y 填到 latitude/longitude 中。

| 基元 | ENU 配置方式 | 默认示例 |
| --- | --- | --- |
| 定点 | `coordinate_frame: enu`，`task.params.waypoints[].enu: {x, y, z}` | `examples/enu/point_goal.yaml` |
| 多点 | 同一定点格式，在 waypoints 中放多个 ENU 点 | 默认 `examples/gps/multi_point.yaml`；可传 `examples/enu/multi_point.yaml` |
| 巡线 | `coordinate_frame: enu`，两个 waypoint 的 `enu: {x, y, z}` | 默认 `examples/gps/patrol_line.yaml`；可传 `examples/enu/patrol_line.yaml` |
| 区域 | `coordinate_mode: enu`，`center_enu: {east, north}`，`rectangle: {length_m, width_m, yaw_deg}` | `examples/enu/area_search.yaml` |

ENU 坐标相对于导航原点，单位米，x/east 向东、y/north 向北，z/up 是相对原点的高度；不是相对机器人当前位置或朝向。区域 ENU 配置的中心只使用 east、north，高度取导航原点高度。

例如，无需修改仓库配置即可建立多点 ENU 任务：

```bash
cat > /tmp/multi_point_enu.yaml <<'YAML'
robot_namespace: robot_1
coordinate_frame: enu
task:
  task_id: multi_enu_001
  params:
    waypoints:
      - enu: {x: 2.0, y: 0.0, z: 0.0}
      - enu: {x: 2.0, y: 3.0, z: 0.0}
YAML
bash run_primitive.sh multi_point --namespace robot_1 --file /tmp/multi_point_enu.yaml
```

这种 ENU 文件使用示例节点的坐标配置格式，不要在其中加 `task.task_type`；包含 `task_type` 的文件按统一 GPS Task 格式直接解析。

发送 ENU 示例前，节点从运行中的 `/robot_1/nav2_input_bridge/get_parameters` 读取 `reference_latitude/longitude/altitude`，然后进行转换。因此 ENU 的 `--dry-run` 也需要导航原点参数服务；`--check` 和 `--cancel` 不读取原点。

## 仿真中的 GPS 与 ENU 分工

```text
GPS 配置 → 示例节点发布 GPS Task ──────────────┐
ENU 配置 → 查询导航原点 → 转 GPS → 发布 Task ─┴→ task_pipeline → 基元 Action
  → GPS 目标/区域中心投影到 ENU → ENU 目标点或路径 → 导航控制
```

本仓库的仿真入口不启动 MAVROS，也不主动发布机器人的 GPS 定位。外部仿真器或 UE 桥是否发布 GPS 定位，取决于它们自己的实现。导航的运动执行依赖 ENU 目标/路径、里程计、TF 和点云。

不过，仿真任务入口仍包含 GPS 坐标发布，且调度器订阅 `/robot_1/mavros/global_position/global` 的有效定位来计算单点任务进度。因此不能概括为“仿真链路完全没有 GPS 发布或接收”。没有有效或新鲜 GPS 时，单点进度改用新鲜 odom/TF 提供的 ENU 位姿计算；二者均不可用时保持已有进度。Action 结束时仍发布 100，成功与否由 Action 结果判断。该定位输入不是 GPS/ENU 投影原点的来源，投影原点来自导航参数。

## 查看进度与结果

建议先订阅事件，再提交任务：

```bash
ros2 topic echo /robot_1/task_feedback std_msgs/msg/String
# 另一个终端查看当前任务进度
ros2 topic echo /robot_1/robot_task_status task_interfaces/msg/TaskStatus
```

`task_feedback` 的 `data` 为 JSON，按 `task_id` 关联事件：`queued` 表示入队，`executing` 表示 Action 接受，`completed` 表示成功，`failed` 表示失败或取消，`rejected` 表示输入非法。它按事件发送，不固定频率，后订阅不会补发历史事件。

`robot_task_status` 仅以 1 Hz 发布当前任务；Action 结束时立即补发 progress=100，失败和取消也一样。进度 100 表示结束，不等于成功。完整接口和启动链路见 [TASK_PIPELINE.md](TASK_PIPELINE.md)。
