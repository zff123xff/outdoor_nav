# YOLO TensorRT 检测

YOLO 检测节点支持独立启动；仿真导航入口默认同时启动 YOLO 与 RTMP 视频桥。节点默认位于
`/robot_1/yolo_detector`；所有配置 topic 都使用相对名称。

| 接口 | 默认相对 topic | 类型 | QoS |
| --- | --- | --- | --- |
| 输入 | `front_camera/image_raw` | `sensor_msgs/msg/Image` | BEST_EFFORT，depth=1 |
| 可选相机标定输入（topic 模式） | `front_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | BEST_EFFORT，depth=1 |
| 雷达输入 | `livox/lidar` | `sensor_msgs/msg/PointCloud2` | BEST_EFFORT，depth=1 |
| 三维坐标 | `targets/markers` | `visualization_msgs/msg/MarkerArray` | RELIABLE，depth=10 |
| 检测 | `yolo/detections` | `yolo_interfaces/msg/DetectionArray` | RELIABLE，depth=10 |
| 可选调试图 | `yolo/debug_image` | `sensor_msgs/msg/Image`，bgr8 | BEST_EFFORT，depth=1 |

有选中目标类别时，每个成功处理的相机帧发布一个数组，包括空检测数组，保留原始 header。空类别列表时保持模型加载，但不处理图像或推理。
无效输入会丢弃并限频记录日志；推理失败会报错退出，不发布伪造的空结果。
相机回调校验并转换 BGR，单槽缓冲覆盖未处理帧，推理线程通过条件变量等待。
模型加载、预热和推理都在同一线程执行；退出时先等待正在执行的推理结束。
检测节点发布二维结果到 `yolo/detections`，并复用 `projection.py` 将真实激光点云投影到检测框。
三维输出为 `targets/markers`（`visualization_msgs/msg/MarkerArray`），坐标位于每个
`markers[i].pose.position.{x,y,z}`，单位米，坐标系由 `header.frame_id` 指定，默认 `robot_1/map`。
每个 marker 含稳定目标 ID 和类别文本，3 秒后自动过期；位置是框内激光回波的中位数，代表可见表面位置。
默认使用 `config/yolo.yaml` 中的固定相机内参，无需发布 `front_camera/camera_info`。
`camera_info_source: yaml` 时读取 `camera_width/height`、`camera_fx/fy/cx/cy`、
`camera_distortion_model` 和 `camera_distortion_coefficients`，构造与原始图像一致的标定。
`camera_optical_frame` 为空时沿用输入图像的 `header.frame_id`，填写时必须完全匹配。
当前 YAML 保留原始标定尺寸 1920×1080 及内参 fx=fy=960、cx=960、cy=540。
视频桥默认输出 1920×1080，与标定一致。`camera_scale_to_image: true` 在将来整帧缩放时按实际宽高缩放 K，
不会改写原始标定，也不改变 TensorRT 的 640×480 输入。仅适用于整帧缩放，裁剪/填边/旋转需重新配置。
相机标定由 YOLO 独立管理；参数无效时告警并暂停三维发布，焦距和主点支持整数或浮点 YAML 数值。
`camera_info_source: topic` 可切回 CameraInfo 订阅，此时使用消息给出的内参并严格校验图像尺寸。
两种模式仍需 `livox/lidar` 及相机/雷达到地图的有效 TF；外参不由上述内参参数替代；
图像与最接近的雷达帧时间差默认不超过 0.15 秒，以该雷达帧为末帧向前累积 `cloud_accumulation_s`（默认 0.5 秒）的点云。
每帧雷达点先用自身时间戳的 TF 转到 map，再统一用检测图像时刻的 TF 转到相机系并投影，补偿机器人帧间运动。
此方式不补偿运动目标自身的位移，也不做雷达帧内逐点去畸变。累积时长范围 0～5 秒，0 恢复单帧模式。
输入超过 1.5 秒则丢弃，累积后框内至少需要 3 个有效点；不以过旧历史替代当前测量。
同一时间戳、同一坐标系的重复包只计一次。缺少采集时刻 TF 的帧跳过，其余帧仍可使用。
点云历史同时按时间和最多 512 帧限制内存；点云接收使用独立回调组，不在接收回调内执行投影。
缺失标定、TF、时间匹配或有效点时限频告警，不发布伪造坐标，不影响二维检测。
三维输出不依赖区域搜索激活或 GPS 原点；仍遵守检测类别文件的选择。
通过 `publish_target_markers: false` 可关闭三维订阅和输出。修改配置后需重启节点。
相机外参可在导航参数 `src/robot_launch/config/go2w_nav2.yaml` 的 `camera_mount` 中配置。
设置 `xyz`（米）、`rpy`（弧度）后将 `enabled` 改为 `true`，重启导航发布静态 TF：
`<namespace>/base_link -> <namespace>/front_camera_optical_frame`。
该旋转直接描述光学坐标系（X 右、Y 下、Z 前）相对机身（X 前、Y 左、Z 上）的姿态。
当前零平移是占位值，默认关闭；仿真端/URDF 已发布该 TF 时也应关闭。
真机和仿真导航入口共享此设置；单独运行 YOLO 不会启动导航侧 TF 发布器。
使用自定义导航 YAML 时，在其中添加相同的 `camera_mount.ros__parameters` 配置。

143 默认从 `targets/assigned` 接收分配位置；`inspection_search_mode:=true` 时在区域搜索中由 `targets/markers` 触发临时观察巡检。
两种 143 模式下发时均更新检测类别文件。
旧 simulation_target_marker 及其接近/确认交互已移除，巡检接口见 [SIMULATION_TARGETS.md](../../docs/SIMULATION_TARGETS.md)。

## 环境与构建

在实际部署机器使用已有 `~/yolo_env`，无需另建环境。以下假设其 Python
位于 `~/yolo_env/venv/bin/python`；若环境根目录直接包含 `bin/python`，相应改为
`~/yolo_env/bin/python`。需要与 ROS Humble 匹配的 Python 3.10，并能导入
`rclpy`、`cv_bridge`、`numpy`、`ultralytics`、`torch`、`tensorrt`。
Jetson 的 torch/TensorRT 必须与本机 JetPack/CUDA 匹配，不要直接用通用 pip 包覆盖。
若使用系统 Humble 的 cv_bridge，其 NumPy 1.x ABI 需要兼容的 NumPy 版本。

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source ~/yolo_env/venv/bin/activate
python -c 'import sys, numpy, rclpy, cv_bridge, ultralytics, torch, tensorrt; print(sys.executable, numpy.__version__, ultralytics.__version__, tensorrt.__version__); print("CUDA:", torch.cuda.is_available())'

cd /path/to/outdoor_nav
# 显式用虚拟环境 Python 构建，使 console script 的解释器指向该环境。
python -m colcon build --symlink-install --packages-select task_interfaces yolo_interfaces yolo_detector
source install/local_setup.bash
```

`ultralytics`、CUDA、TensorRT 由部署环境管理，不在 setup.py 中自动下载安装。
如果虚拟环境无法导入 colcon 或 ROS 系统依赖，先配置其系统 site-packages 可见性。

## Engine 与配置

在目标 NX 上导出静态 batch=1、FP16、H×W=480×640 engine。新版 Ultralytics：

```bash
yolo export model=best.pt format=engine imgsz=480,640 batch=1 device=0 dynamic=False quantize=16
```

旧版 Ultralytics 若不支持 `quantize`，使用其对应的 `half=True` 导出选项。
尺寸、精度及运行时兼容性以实际安装版本为准。参考
[官方 TensorRT 文档](https://docs.ultralytics.com/integrations/tensorrt/)。
Engine 应在目标 GPU/匹配的 TensorRT 与 CUDA 上构建和验证。

复制并编辑 `config/yolo.yaml`，设置 `model_path`（默认
`/home/nhy/models/best.engine`）。缺失文件会在启动时明确报错。
节点在预热后检查 engine 是静态 `(1, 3, model_input_height, model_input_width)`，
不接受动态 engine。FP16 由 engine 导出配置决定，日志中的 `fp16_io` 只表示输入输出精度，
不能证明每一层都使用 FP16。

`expected_width/height` 校验相机输入，`model_input_width/height` 指定 engine 输入，
两者独立；默认原图为 1920×1080，模型输入为 640×480。模型尺寸必须是正的 32 倍数。
预测显式使用 `imgsz=(height, width), rect=False`，防止默认方形预处理与 engine 不匹配；
参见 [官方预测参数](https://docs.ultralytics.com/modes/predict/#inference-arguments)。

关闭 `strict_image_size` 可接受其他相机尺寸，Ultralytics 会进行 letterbox 和坐标还原。
发布坐标始终属于实际输入原图，整数边界裁剪到 `[0, width-1]`、`[0, height-1]`，
中心和宽高从裁剪后的边界计算。退化、反向或非有限框不发布。
encoding 不符会警告并尝试转换；不支持的 encoding 或转换失败则丢弃。
ROS 参数启动后只读，修改 YAML 后重启节点。类别文件默认统一为
`/home/nhy/outdoor_nav_1105/src/yolo_detector/config/detection_targets.yaml`，每秒读取；空列表表示暂停检测。
142、143 真正下发时原子写入 `Task.targets`，任务结束、取消和退出后保留；140、141、145 不修改文件。
检测器启动不写文件，避免覆盖正在下发的任务；文件缺失时待机，恢复后继续，非法 YAML 保留最后有效选择。
仿真入口的 `yolo_targets_file` 同时传给调度器和检测器；独立启动自定义文件需让两端配置一致。

## 运行

随仿真导航自动启动：

```bash
bash launch_sim.sh robot_namespace:=robot_1 \
  yolo_python:=$HOME/yolo_env/venv/bin/python \
  yolo_config:=/absolute/path/to/yolo.yaml
```

不需要检测时使用 `yolo:=false`，不需要视频桥时使用 `video_stream:=false`。
`video_config` 指定视频 YAML，`video_url` 可覆盖其中的 RTMP 地址。独立启动时也须显式选取具有 GPU 依赖的解释器：

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source ~/yolo_env/venv/bin/activate
source install/local_setup.bash
ros2 launch yolo_detector yolo.launch.py namespace:=robot_1 \
  python_executable:=$HOME/yolo_env/venv/bin/python \
  config_file:=/absolute/path/to/yolo.yaml
# 第二台机器人使用 namespace:=robot_2，并使用相应相机 topic。
```

保持相机与节点的 ROS_DOMAIN_ID、DDS 配置一致。配置中 topic 不得以 `/` 或 `~` 开头。
也可直接指定虚拟环境解释器运行，避免已有构建产物的 shebang 指向系统 Python：

```bash
~/yolo_env/venv/bin/python -m yolo_detector.yolo_node --ros-args \
  -r __ns:=/robot_1 --params-file /absolute/path/to/yolo.yaml
```

接收结果（超时后退出）：

```bash
timeout 10s ros2 topic echo /robot_1/yolo/detections yolo_interfaces/msg/DetectionArray
```

调试图默认关闭，通过 `publish_debug_image: true` 启用，会增加绘图和传输开销。
低延迟策略只限制节点内部图像积压，不对 GPU 推理时长或可靠输出传输作硬实时保证。

## 测试

构建并 source 工作区后，在隔离 domain 运行 CPU 测试，使用假模型和真实 DDS 收发：

```bash
ROS_DOMAIN_ID=187 ROS_LOG_DIR=/tmp/yolo_detector_test_logs \
  python -m pytest -q src/yolo_detector/test/test_detector.py
```

覆盖最新帧替换、线程唤醒、图像校验、RGB→BGR、原始 header、坐标裁剪、空结果、
robot_2 命名空间、参数错误和推理异常。不替代 NX 上的真实 engine 精度、延迟及长时间运行验证。

## 从 UE RTMP 视频接入

保留现有检测节点，用独立的 `rtmp_image_bridge` 把视频发布到同一 `front_camera/image_raw`。
默认输出为 1920×1080 / rgb8，与检测器原图配置一致。地址配置和 pull/listen 模式见
[RTMP 图像节点说明](../external_pkg/rtmp_image_bridge/README.md)。

### 调试图发布诊断日志

节点启动时打印输入/输出话题、publish_debug_image 开关、QoS、targets 文件位置和当前类别。
新增稳态时钟诊断定时器，每秒检查；同类未发布原因最多每 5 秒重复一次：

- `debug_image not publishing: targets is empty`：类别为空，节点待命。
- `debug_image not publishing: no recent input image`：尚未收到或超过 5 秒没有新图像。
- `publish_debug_image=false`：启动配置关闭调试图，修改配置后需重启。
- `model initialization/warmup not finished`：模型初始化尚未返回。
- `input image rejected` / `image conversion failed`：尺寸无效、尺寸不匹配或编码转换失败。
- `inference has not returned`：推理尚未返回，附等待时长。
- `targets changed during inference`：类别变化导致旧帧丢弃。
- worker 异常会立即打印失败阶段，包括推理、绘图、发布，并保留原来的 FATAL 退出行为。

日志带累计输入/调试图发布数；首次成功或恢复发布打印 `debug_image publishing`，
包括话题、编码、类别和匹配订阅者数量。发布计数指成功调用本地 publish，
不代表远端 rqt 已收到图像。没有匹配订阅者时仍保留原发布行为。
类别变更也会打印日志；清空类别会明确提示 detections/debug_image 已暂停。
这些诊断不改变原有类别门控、推理和图像发布条件。


### 三维发布诊断

节点每 5 秒输出一条 `3D health`，包括图像回调、投影和推理耗时（毫秒），
检测、点云和 TF 延迟（秒），图像与点云时间差、三维发布累计帧数和当前阶段。
`accumulated_clouds` / `accumulated_points` 为该次处理成功转换的累积帧数和点数，
`tf_skipped_clouds` 为因缺少采集时刻 TF 而跳过的帧数。
`image_max_ms` / `projection_max_ms` 为最近一轮日志间隔的最大回调耗时。
当外部 TF 正常更新而节点内 `tf_age_s` 持续增大时，应检查执行器回调积压。
节点使用两个执行器线程，使 TransformListener 的独立回调组可以在图像转换和
点云投影期间继续接收 TF；坐标计算仍查询采集时刻的 TF，不使用最新 TF 替代。

水平朝前的相机，`camera_mount.rpy` 应为 `[-π/2, 0, -π/2]`，
对应光学轴 X 向右、Y 向下、Z 向前。`[0,0,0]` 不能直接表示机身系到光学系的转换。
目标框内实测雷达点不足、输入过期或时间不匹配时，该帧不会生成三维目标。
