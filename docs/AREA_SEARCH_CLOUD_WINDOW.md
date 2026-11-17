# 区域搜索点云缓存与 TF 时间

源码：`src/external_pkg/co_search/co_search_nav/src/search_engine.cpp`；配置：
`src/external_pkg/co_search/co_search_nav/config/area_search.yaml`。

每个点云回调先拒绝超过 `options.sensor_timeout_s` 的旧帧，或领先 ROS 时间超过
`cloud_filter.max_future_offset_s` 的帧；然后去除非有限点，按需在原始点云 x 轴裁剪、VoxelGrid 降采样，
按消息采集时间查 TF 转换到任务/map 与 `base_link`，执行高度带过滤后才放入待处理队列。
TF 查询最长等 `tf_lookup_timeout_s`；超时后丢弃该帧并保留错误文本，不以旧 TF 代替。

已转换点云按各自采集时间戳保存在滚动窗口，窗口合并后再次体素去重。每个有效观测的传感器原点保留，
每帧只集成一次，不会因为重复读缓存而反复增加占据概率。待处理队列最多 512 帧；地图窗口最多保留
256 帧。`map_period_s` 是地图整合节流周期，不是窗口长度。

板卡当前参数：

```yaml
options:
  sensor_timeout_s: 3.0
co_search:
  cloud_filter:
    enable_x_filter: false
    min_x_m: 3.0
    max_x_m: 40.0
    voxel_size_m: 0.2
    accumulation_window_s: 1.0
    min_accumulated_points: 1
    max_future_offset_s: 0.5
    tf_lookup_timeout_s: 2.0
  map_period_s: 0.2
```

因为 `enable_x_filter` 为 false，目前 `min_x_m/max_x_m` 不會用于裁剪。`min_accumulated_points` 统计窗口里
体素去重后的点数，不是帧数。`cloud_window_frames` 和 `cloud_window_points` 见
`/<ns>/co_search/engine_state`；`insufficient accumulated cloud` 会报告实际窗口、帧数、点数和最近帧错误。

## 点云掉线和不同方向的 TF 错误

只有完成 TF 转换并进入待处理点云队列的帧才刷新有效接收时间。没有有效帧超过 3 秒时，原生引擎进入
`waiting_sensor`；如果搜索已经使能，Python 执行器会将 142 暂停。因此连续 3 秒无有效点云可能暂停任务。
仅有 TF 异常的帧即使传感器话题仍在发布，也不会刷新有效云时间。

- `Lookup would require extrapolation into the future`：请求时间晚于 TF 最新时间。`tf_lookup_timeout_s`
  可以等待对应 TF 到达，但不能修正时钟偏差或永远没有的 TF。
- `earlier than all the data in the transform cache`（常见于 Nav2 message filter）：请求时间早于 TF 缓存
  最早时间。增加等待时间没有帮助，应检查传感器消息积压、ROS 时间是否跳变，以及 TF 缓存与消息延迟。

Nav2 costmap 的 message filter 和 co-search 引擎分别持有自己的 TF buffer；co-search 的 2 秒查询等待不自动
改变 Nav2 costmap 的缓存历史。也不要把 `cloud_accumulation_s`（YOLO 关联窗口）和
`accumulation_window_s`（区域搜索建图窗口）混为一项。

修改 YAML 后重启 `co_search_executor`/原生引擎使配置生效；如果变更 C++ 实现，则编译 `co_search_nav`。
