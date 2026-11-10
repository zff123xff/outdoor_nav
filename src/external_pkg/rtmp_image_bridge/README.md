# UE RTMP → ROS Image

独立节点 `rtmp_image_bridge` 接收 RTMP/RTMPS 视频，用系统 FFmpeg 解码并发布
`sensor_msgs/msg/Image`。默认发布 `/robot_1/front_camera/image_raw`，与保留的 YOLO 检测节点直接对接。
不自动启动 UE、RTMP 服务或 YOLO；不改变现有导航启动文件。

## 地址配置

配置文件：[config/rtmp.yaml](config/rtmp.yaml)。`rtmp_url` 默认为空，必须填写或通过 launch 参数提供。
修改源码 YAML 后，symlink install 的工作区会读取同一配置。也可复制配置到自己的文件并传 `config_file`。

```yaml
/**:
  ros__parameters:
    rtmp_url: "rtmp://192.168.1.186:1935/live/robot_1"
    mode: pull
    image_topic: front_camera/image_raw
    width: 640
    height: 480
    encoding: rgb8
```

`pull` 模式地址是 **RTMP 服务所在机器的地址**，不一定是 UE 的地址。
典型链路：UE 向 RTMP 服务推流，节点从相同流地址拉取。

## 构建与启动

依赖系统 `ffmpeg`，节点本身不依赖 OpenCV、NumPy 或 GPU Python 环境。
缺少 FFmpeg 时先安装发行版的 ffmpeg 软件包。

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
bash build.sh --packages-select rtmp_image_bridge --executor sequential
source install/local_setup.bash

# 方式一：在 launch 参数里指定地址，覆盖 YAML
ros2 launch rtmp_image_bridge rtmp.launch.py namespace:=robot_1 \
  rtmp_url:=rtmp://192.168.1.186:1935/live/robot_1

# 方式二：已在 config/rtmp.yaml 填好地址
ros2 launch rtmp_image_bridge rtmp.launch.py namespace:=robot_1

# 方式三：使用自己的 YAML
ros2 launch rtmp_image_bridge rtmp.launch.py namespace:=robot_1 \
  config_file:=/absolute/path/my_rtmp.yaml
```

配置优先级为：节点默认值 < YAML < 非空 launch 参数。参数启动后只读，修改地址后重启节点。

### UE 直接推流到 ROS 机器

没有单独 RTMP 服务时，可启用 `listen`，FFmpeg 作为单路接收端：

```bash
ros2 launch rtmp_image_bridge rtmp.launch.py namespace:=robot_1 \
  mode:=listen rtmp_url:=rtmp://0.0.0.0:1935/live/robot_1
```

UE 推流地址填写 `rtmp://<ROS机器可达IP>:1935/live/robot_1`，不能填 `0.0.0.0`。
listen 模式每个节点只接收一个发布者，不是通用的多客户端 RTMP 服务；多机器人需不同监听端口。
RTMPS 仅支持 pull，需由外部服务处理 TLS。

先启动接收节点，再让 UE 推流。监听超时后会重新监听，UE 推流端应支持失败重试。
长 GOP 会增加首次解码等待，建议 UE 使用 H.264、固定帧率、约 1 秒关键帧间隔。

## 对接 YOLO

默认输出是 `640×480 / rgb8`，sensor-data QoS：BEST_EFFORT、VOLATILE、KEEP_LAST depth=1。
现有 `yolo_detector/config/yolo.yaml` 的默认 image_topic、尺寸和编码已经匹配。

```bash
# 另一个终端，在已有模型/GPU 环境中单独启动 YOLO
ros2 launch yolo_detector yolo.launch.py namespace:=robot_1 \
  python_executable:=/home/nhy/yolo_env/venv/bin/python
```

YOLO 原有模型路径、依赖和类别选择文件仍需有效。空类别列表时，YOLO 保持待机，不进行检测；
需要推理时在其 targets_file 中配置类别，例如 `targets: [person, car]`，不需要启动巡检任务。

```bash
ros2 topic info /robot_1/front_camera/image_raw -v
# 有实际图像才证明视频链路通了：只观察 header，不打印整帧像素
ros2 topic echo /robot_1/front_camera/image_raw sensor_msgs/msg/Image \
  --qos-reliability best_effort --field header --once
ros2 topic hz /robot_1/front_camera/image_raw
ros2 topic echo /robot_1/yolo/detections yolo_interfaces/msg/DetectionArray
```

如果其他相机节点已发布同名图像，请停用该发布源，或修改本节点 image_topic，并同步修改 YOLO 配置。

## 参数与时间

| 参数 | 默认值 | 含义 |
|---|---|---|
| rtmp_url | 空，必填 | 服务端拉流地址或本机监听地址 |
| mode | pull | pull / listen |
| image_topic | front_camera/image_raw | 相对话题，自动使用节点 namespace |
| frame_id | 空 | 空时使用 `<namespace>/front_camera_optical_frame` |
| width / height | 640 / 480 | 解码输出尺寸；按该尺寸缩放，需与 YOLO 期望尺寸一致 |
| encoding | rgb8 | rgb8 或 bgr8 |
| publish_fps | 30.0 | ROS 发布频率上限（令牌桶，允许 1 帧突发），不补帧 |
| decoder | auto | auto / ffmpeg / ffmpeg-hw / gstreamer；auto 只使用实测通过的后端 |
| gst_launch_path | gst-launch-1.0 | GStreamer 后端可执行路径 |
| stats_period_s | 5.0 | 性能统计周期 |
| read_timeout_s | 5.0 | 指定时间内没有完整帧就重连，包含连接和首次解码等待 |
| reconnect_delay_s | 2.0 | 每次断线后重试间隔 |
| max_frame_age_s | 0.5 | 解码完成到发布的最大本机等待时间 |
| ffmpeg_path | ffmpeg | 解码器可执行路径 |
| use_sim_time | false | 图像时间戳是否使用 ROS 仿真时钟 |

图像 header.stamp 为**本机完成解码时的 ROS 时间**，不代表 UE 原始曝光时间。
与仿真使用同一时钟体系；use_sim_time=true 但尚无有效 /clock 时丢弃帧，避免发布零时间戳。
未生成 CameraInfo 或相机 TF；如需三维投影，应另行提供与输出尺寸匹配的标定，并处理原始采集时间同步。

单槽缓存只保留最新解码帧，慢订阅者不导致节点内排队；断流清空缓存，不反复发布停住的旧图像。
网络/RTMP 服务/编码端仍可能缓存，以上不构成端到端延迟保证。节点断线重启 FFmpeg，退出时回收解码进程。
日志不输出流地址或 FFmpeg 的原始 stderr，以免暴露 URL 中的认证信息。

## 解码后端与实测性能（Jetson Orin NX，L4T R36.5.2）

`decoder: auto` 启动时按顺序实测，并在日志里报告结果，不凭 `ffmpeg -hwaccels` 下结论：

1. `ffmpeg-hw`：用合成 H.264 实际解码一次，验证 `h264_cuvid` 可用。本机因缺少
   `libnvcuvid.so.1`（只有 `libnvcuvidv4l2.so`）而**探测失败**，不会启用。
2. `gstreamer`：要求 `rtmpsrc`、`h264parse`（属 `gstreamer1.0-plugins-bad`）、
   `nvv4l2decoder`、`nvvidconv` 全部存在。本机缺 `rtmpsrc`/`h264parse`，**不可用**；
   `nvv4l2decoder` 与 `nvvidconv` 本身是装好的。
3. `ffmpeg`：软件解码，默认走这条。

本机基准（1920×1080@30 H.264）：

| 环节 | 实测 |
|---|---|
| 纯软件解码（`-f null`） | 312 fps |
| 解码 + scale 640×480 | 129 fps |
| 解码 + scale 640×480 + rgb24 走管道 | **74 fps** |
| 解码 + 1080p bgr24 走管道 | **71 fps** |
| `h264_cuvid` | 不可用（缺 libnvcuvid.so.1） |
| `h264_v4l2m2m` | 不可用（no valid device） |

结论：解码不是瓶颈，管道+格式转换后仍有 2.4 倍余量。真实链路瓶颈在**发布节拍与上游帧率**。

端到端实测（本机 USB 摄像头 1280×720 MJPG → x264 → RTMP → NX 解码发布）：

| 输出尺寸 | input_fps | publish_fps | dropped | decode_latency | publish_latency |
|---|---|---|---|---|---|
| 1280×720 | 30.0~30.6 | 27.8~30.0 | 0~2/5s | 3.3~5.2 ms | 1~3 ms |
| 640×480 | 30.0~30.6 | **30.0** | 0 | 1.2~1.6 ms | 0.4 ms |
| 1920×1080（文件回放） | 30.0~30.2 | **29.6~29.8** | 0~2/5s | 6.8~12.3 ms | 2.0 ms |

资源占用（640×480@30，tegrastats）：CPU 46~58%（4 核 @1267 MHz）、GPU 0%、
NVDEC off、RAM 3.9/15.6 GB、整板约 6.9 W。软件解码在 640×480 几乎没有成本。

已知限制：

- **Python 订阅端**比发布端慢：`ros2 topic hz` 在 640×480 约 25 Hz、720p 约 20 Hz，
  因为 rclpy 反序列化 2.7 MB/帧的开销在订阅进程里。发布端计数仍是 30 Hz，
  用 `[RTMP统计]` 的 `publish_fps` 判断发布能力，不要用 Python 订阅端反推。
- 本机上游最高 1280×720@30（MJPG）；1080p 只能用文件回放或真实 UE 流验证。
- `-fflags nobuffer -flags low_delay` 实测会让该 RTMP 流的 H.264 序列头丢失
  （`decode_slice_header error`，0 帧），已从命令中移除，不要再加回。

## 性能统计

每 `stats_period_s` 输出一行：

```text
[RTMP统计] input_fps= decode_fps= publish_fps= dropped_frames= overwritten_frames=
           width= height= decode_latency_ms= publish_latency_ms= backend=
```

- `input_fps`/`decode_fps`：解码线程实际产出完整帧的速率（等于入流帧率）。
- `publish_fps`：本周期实际发布次数，重复帧不会计入。
- `dropped_frames`：本周期解出但未发布的帧数（上游快于发布上限）。
- `overwritten_frames`：单槽缓存被新帧覆盖的次数。
- `decode_latency_ms`：单帧从首个字节到拼帧完成的耗时（传输+解码）。
- `publish_latency_ms`：构造并发布一条 ROS Image 的耗时。

## 验证

```bash
source install/local_setup.bash
PYTHONNOUSERSITE=1 python3 -m pytest -q src/external_pkg/rtmp_image_bridge/test
ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 \
  timeout 55s python3 tests/rtmp_image_smoke.py
```

端到端测试使用真实 H.264/RTMP/FFmpeg 与 DDS，YOLO 使用 CPU 假模型，仅验证图像接口和断流重连，不依赖 UE 或 GPU。
