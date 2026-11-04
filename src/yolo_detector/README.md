# YOLO TensorRT 检测

独立启动，不修改导航或模拟器启动流程。节点默认位于
`/robot_1/yolo_detector`；所有配置 topic 都使用相对名称。

| 接口 | 默认相对 topic | 类型 | QoS |
| --- | --- | --- | --- |
| 输入 | `front_camera/image_raw` | `sensor_msgs/msg/Image` | BEST_EFFORT，depth=1 |
| 检测 | `yolo/detections` | `yolo_interfaces/msg/DetectionArray` | RELIABLE，depth=10 |
| 可选调试图 | `yolo/debug_image` | `sensor_msgs/msg/Image`，bgr8 | BEST_EFFORT，depth=1 |

每个成功处理的相机帧发布一个数组，包括空检测数组，保留原始 header。
无效输入会丢弃并限频记录日志；推理失败会报错退出，不发布伪造的空结果。
相机回调校验并转换 BGR，单槽缓冲覆盖未处理帧，推理线程通过条件变量等待。
模型加载、预热和推理都在同一线程执行；退出时先等待正在执行的推理结束。
二维检测节点只负责推理；独立的 `simulation_target_marker` 结合点云、相机标定及 TF 提供三维定位与稳定 ID，由 task_pipeline 在仿真区域搜索期间启动和停止。

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
两者独立；默认均为 640×480。模型尺寸必须是正的 32 倍数。
预测显式使用 `imgsz=(height, width), rect=False`，防止默认方形预处理与 engine 不匹配；
参见 [官方预测参数](https://docs.ultralytics.com/modes/predict/#inference-arguments)。

关闭 `strict_image_size` 可接受其他相机尺寸，Ultralytics 会进行 letterbox 和坐标还原。
发布坐标始终属于实际输入原图，整数边界裁剪到 `[0, width-1]`、`[0, height-1]`，
中心和宽高从裁剪后的边界计算。退化、反向或非有限框不发布。
encoding 不符会警告并尝试转换；不支持的 encoding 或转换失败则丢弃。
ROS 参数启动后只读，修改 YAML 后重启节点。检测类别通过 `targets_file` 动态更新：默认 `/tmp/outdoor_nav/<namespace>/detection_targets.yaml`，每 1 秒读取，空列表检测全部类别；无效文件保留上一次有效配置。dispatcher 在任务开始时原子更新此文件。调试图也只绘制选中类别。

## 运行

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source ~/yolo_env/venv/bin/activate
source install/local_setup.bash
ros2 launch yolo_detector yolo.launch.py namespace:=robot_1 \
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

## 区域搜索仿真标记

由 task_pipeline 管理节点生命周期，仿真区域搜索执行时启动，任务终态及调度器退出时停止。使用 `config/target_marker.yaml` 配置相机、点云、同步和关联阈值。发现目标与请求共用 `task_interfaces/msg/TargetEvent`；`request shoot` 只把已发现目标的 RViz 标记改为红色，不连接实体设备。完整话题、标定要求、示例见 [SIMULATION_TARGETS.md](../../docs/SIMULATION_TARGETS.md)。
