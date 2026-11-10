# 任务链路排障手册（`/task` → cmd_vel）

> 本文是**按现象排障的 runbook**，用于接手开发者在任务“发不进、不动、报错、崩溃”时快速定位。
> 原理性背景见 [开发交接总览](DEVELOPER_HANDOVER.md)、[任务接口与执行](TASK_PIPELINE.md)、
> [整体数据链路](DATA_FLOW.md)、[速度指令链路](VELOCITY_CHAIN.md)；本文只引用结论，不重复展开。
> 内容以 2026-11-10 核对的**最新源码 `outdoor_nav_1110`** 为准；板卡当前运行的还是旧 build，
> 二者行为有差异（尤其 dispatcher），引用前先确认对的是哪份。

---

## 0. 三分钟定位流程

先在**与导航栈相同用户、相同 ROS domain** 的终端里开两个窗口，再复现一次任务下发：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 topic echo /robot_1/task_feedback        # 窗口 A：事件流
ros2 topic echo /robot_1/robot_task_status    # 窗口 B：状态流（可选）
ros2 topic echo /robot_1/cmd_vel              # 窗口 C：速度（可选）
```

按窗口 A 的输出去分支（`<ns>` 换成 `robot_1` 等）：

| 观察 | 结论 | 详见 |
|---|---|---|
| 没有任何 `accepted` | 任务没进 dispatcher：丢在入口（发布侧 / 桥 / 命名空间 / domain） | §2.1 |
| 出现 `rejected` + `message` | 被拒收：校验失败 / 忙碌 / 取消无匹配，`message` 直接给原因 | §2.3 |
| `accepted` 后迟迟不 `executing` | dispatcher 状态机卡住：无法确认子任务所有权 / 等目标交互收尾 / Action 不在线 | §2.2 |
| `executing`，但 cmd_vel 恒 0 / 狗不动 | 收口在控制侧：twist_mux 被遥控抢占 / 前置门槛未满足 | §2.4 |
| 进程直接崩溃 / `bad_alloc` / 字符串错乱 | ABI 不一致（新旧库混装） | §4 |

在深入任何一条之前，**先排除 §3 的环境一致性陷阱**——它会让上面所有观测结果失真。

---

## 1. 完整链路与逐跳观测点

```text
UE/后端 → FastDDS(task_command) → /mission_node(aiunit, MutilTask-dog)
   → /<ns>/task (task_interfaces/msg/Task)
   → pipeline_dispatcher.receive()：cancel 走取消；忙碌直接 rejected；否则 validate 后设 current 并发 accepted
   → pump() 按 task_id 派发 Action：
        140 → point_goal (patrol_node/PointGoal)
        141 → patrol      (patrol_node/Patrol)
        142 → area_search (task_interfaces/AreaSearch)
        145 → multi_point (task_interfaces/MultiPoint)
   → 执行器最终都走 Nav2：
        140/141/145 → navigate_to_pose → controller_server
        142        → native search_engine 出 co_search/path → follow_path → controller_server
   → controller_server 发布 cmd_vel（launch 里 remap 成 cmd_vel_nav）
   → twist_mux(nav=10 / remote=20) → /<ns>/cmd_vel
   → ros_to_ue_bridge.py → UDP → UE
```

逐跳怎么确认（括号内是正常现象）：

| 跳 | 观测命令 / 对象 | 正常 |
|---|---|---|
| 入口话题 | `ros2 topic info /<ns>/task -v` | Publisher = `mission_node`，Subscription = `pipeline_dispatcher` |
| 校验/接收 | `ros2 topic echo /<ns>/task_feedback` | 合法任务立即 `accepted` |
| 分发 | 同上 | `accepted` 后很快 `executing` |
| Action | `ros2 action list -t` | `point_goal/patrol/area_search/multi_point` 都在 |
| 控制 | `ros2 topic echo /<ns>/cmd_vel` | 执行中非零；`twist_mux` 在发 |
| 终态 | `task_feedback` 的 `completed`/`failed` + `robot_task_status` 的 `progress=100` | 两者都要看，不能只看 progress |

dispatcher 事件词表（`std_msgs/String` JSON 的 `event` 字段）：`accepted`、`executing`、`completed`、
`failed`、`rejected`、`ownership_error`、`cancel_pending`。**没有 `queued`**（旧版才有）。

---

## 2. 断点清单

### 2.1 现象：`task_feedback` 里连 `accepted` 都没有

任务根本没到 dispatcher。按可能位置从外到内排查：

1. **发布侧只发一次、无 ack、无持久化。** `mission_node`（`MutilTask-dog` 的
   `aiunit`，**独立仓库，不在 outdoor_nav 内**）在 `on_fastdds_MultiTask` 里对 `/<ns>/task` 只
   `publish` 一次，QoS 是 `RELIABLE / VOLATILE`。如果发布瞬间 dispatcher 还没完成 DDS 匹配
   （导航刚重启、发现窗口内），消息直接丢，没有任何重试或落盘。判断方法：窗口 A 完全无输出，
   且窗口 B 一直是 `task_id:"0"`。修法方向：发布前等 `get_subscription_count() > 0`，或改
   `TRANSIENT_LOCAL` 只保最后一条。
2. **`uav_idx` 静默过滤。** 同函数开头 `if (mt.uav_idx != uav_id_) return;`，配置 `uav_id: 1`
   （`mission_config.yaml`）。UE 下发 `uav_idx` 不是 1 时**一行日志都没有**直接丢。判断：对比
   UE 侧 `uav_idx` 与板卡 `uav_id`。修法方向：改成 `RCLCPP_WARN` 并带上两个值。
3. **5 秒阻塞 sleep 导致迟到。** 同函数第一条就是
   `std::this_thread::sleep_for(std::chrono::milliseconds(5000));`，而且这段 sleep 跑在 FastDDS
   reader 的 `on_data_available` 同步回调线程里。结果：任务最早 5 秒后才出现在 ROS；连续下发被串行化
   （每条约 5 秒），reader 历史 `KEEP_LAST depth 30` 以内的消息排队、超出的丢。判断：窗口 A 的
   `accepted` 比点击晚 5 秒以上。修法方向：把延时移到工作队列/定时器，别占 reader 线程。
4. **命名空间 / domain 不一致。** dispatcher 订的是它自己 namespace 下的相对话题 `task`
   （即 `/<ns>/task`）。发布方打到了别的 namespace 或别的 `ROS_DOMAIN_ID`，就看不到。判断：
   `ros2 topic info /<ns>/task -v` 看两端端点；`echo $ROS_DOMAIN_ID` 两边一致。

### 2.2 现象：`accepted` 后迟迟不 `executing`（或后续任务被秒拒）

dispatcher 是**单任务模型**：`receive()` 里只要 `current` / `_pending_finish` / `_ownership_error`
任一非空，普通任务立刻 `rejected('another task is still active')`，**没有队列**。所以“上一个任务没结束”
的对外表现是**新任务被秒拒**，而不是排队。`pump()` 有四个提前返回点
（`task_pipeline/task_pipeline/dispatcher.py`）：

```python
def pump(self):
    if self._pending_finish is not None: ... ; return   # (a) 等 142/143 目标交互收尾
    if self._ownership_error: return                    # (b) 无法确认子任务所有权，永久闭锁
    if self.current is None: return                     # 空闲，无任务
    if self.sending or self.goal_handle is not None: return  # (c) 正在派发/执行
    ...
    if not client.server_is_ready():
        if now > self.deadline: self.finish(False, 'primitive Action server unavailable')
        return                                          # (d) Action server 30s 内没上线
```

- **(b) `_ownership_error` 一旦置位就永不清除。** 来源是 `send_goal_async` / `future.result()`
  抛异常（Action server 重启、图抖动）。现象：日志里有 `Task channel retained: cannot confirm
  primitive ownership`，之后**所有**普通任务都被 `rejected('another task is still active')`。
  恢复只能重启 dispatcher。
- **(a) `_pending_finish` 依赖 `target_interaction.drained()`。** 142/143 目标交互没收尾、且
  `follow_path/_action/cancel_goal` 服务不可用时，永远不 drain，任务通道被永久占用。
- **(c) 无执行超时。** 一旦派发（`goal_handle` 非空）而 primitive 不返回终态（长时间 142、primitive
  卡住、接近 Action 不返回），`pump()` 就一直 return，`current` 不清空。`action_wait_timeout_s`
  （默认 30 s）只用于等 Action server 上线，**不是执行超时**。
- **(d)** Action server 没起来会等 30 s 后 `failed('primitive Action server unavailable')`，属正常失败。

判据：窗口 A 出现 `rejected('another task is still active')`，或 `accepted` 之后无 `executing`；
窗口 B 停在某个旧 `task_id` 不动、progress 不涨。

### 2.3 现象：`rejected`（带 `message`）

`rejected` 的 `message` 分两类：

**A. 调度层拒收**（`dispatcher.py` 直接发）：

| message | 含义 |
|---|---|
| `another task is still active` | 当前任务未结束（含 `_pending_finish`/`_ownership_error` 占用），见 §2.2 |
| `no active task with matching task_id` | 取消消息的 `task_id` 与当前任务不一致 |

注意：`receive()` **先判忙碌、再校验参数**，所以忙碌时哪怕参数不合法，也先回报 `another task is still active`。

**B. 协议校验拒收**（`task_schema.py::validate()` 抛 `ValueError`）：

| task_id | 校验要点 |
|---|---|
| 140 | 恰好 1 个 GPS waypoint |
| 141 | 恰好 2 个 waypoint |
| 142 | `center` 一个三元组 + `length_m`/`width_m` ≥ 1 m |
| 143 | `center` + 尺寸 ≥ 1 m + `targets` 至少一个类别 |
| 145 | ≥ 1 个 waypoint |
| 通用 | `task_id` 必须是 140/141/142/143/145；waypoints 是 `[lat,lon,alt]` 三元组；经纬度在合法范围；未知字段直接拒 |

**特别注意 `mission_node`（UE 路径）的字段映射**（`MutilTask-dog` 的 `mission_publisher.cpp`，
独立仓库，不在 outdoor_nav 内）：

- 141/143 分支：`waypoints` 来自航线点（`get_air_route_points()`）。若下发任务时航线还没收齐，
  点数不是 2，141 会被拒（`141 needs exactly two GPS waypoints`）。该函数会打
  `任务 %u 下发时航线尚未接收完整` 警告。
- 143 分支：`targets` 始终被 `clear()` 且从不填，`center` 那三行是注释掉的 → **143 从 UE 路径必然被拒**
  （`143 needs the search-area center and dimensions >= 1 metre`）。
- 145 落到 `else` 分支：只填 `center`，不填 `waypoints` → **145 从 UE 路径必然被拒**
  （`145 needs at least one GPS waypoint`）。

> 快照：2026-11-10 核对时 141/143 的经纬度顺序已修正（lat 在前，符合协议）；但该分支内
> “航点数组格式：[经度, 纬度, …]”那条注释已过时。143 空 targets / 145 无 waypoints 的问题仍在。
> 直接打 ROS 话题（`send_*` 脚本 / `ros2 topic pub`）时没有这些映射问题，规则只由 `validate()` 决定。

### 2.4 现象：`executing` 但 cmd_vel 恒 0 / 狗不动

链路收口在控制侧。两个常见原因：

1. **twist_mux 被遥控抢占。** `twist_mux` 里 `remote`（UE 遥控）优先级 20 > `nav` 优先级 10，
   超时 0.5 s。只要 UE 持续发 `/<ns>/cmd_vel_remote`（哪怕摇杆回中的全零），就会在超时前覆盖导航。
   判断：`ros2 topic echo /<ns>/cmd_vel_remote` 是否持续非静默；原理见 [速度指令链路](VELOCITY_CHAIN.md)。
2. **执行器的前置门槛没满足。** 例如：
   - 140/145 的 `point_goal_executor` 要求 `reference_latitude/longitude` 已配、`navigate_to_pose`
     在线、`odom` 有数据、`map→base_link` TF 新鲜（>2 s 判失锁），不满足则 `point_goal` 直接 REJECT。
   - 142 的 `co_search_executor.primitive_goal` 要求 GPS 原点就绪、`controller_server` active、
     `lifecycle_manager_navigation` is_active、`follow_path` 在线；执行时还要能拉起原生
     `search_engine` 并收到 `co_search/engine_state`，否则停在 planning。

### 2.5 现象：进程崩溃 / `bad_alloc` / 字符串错乱

几乎可以确定是 **C++ 动态库与头文件版本不一致（ABI）**，而不是数据量问题。典型现场：控制器进程里
`std::bad_alloc`、字符串内容错乱——同进程内不同库对某个类/结构体的大小或布局认知不同。处理见 §4。

---

## 3. 环境一致性陷阱（先排除）

### 3.1 root 与 nhy 的 FastDDS 共享内存隔离（板端实测）

导航栈以 root 启动时，**以 `nhy` 身份运行的 `ros2` CLI 只能完成发现、收不到数据面**：

```text
nhy : ros2 topic echo /robot_1/robot_task_status --once   → 12s 超时无输出
nhy : ros2 param get /robot_1/pipeline_dispatcher <any>   → 超时
root: ros2 topic echo /robot_1/robot_task_status --once   → 立即收到 task_id: '0'
```

原因：root 创建的 FastDDS 共享内存段 `nhy` 打不开，而 UDP 发现照常，所以“看得见节点、收不到数据”。
**推论**：所有排障命令、以及发任务的脚本，都要用与导航栈**相同用户**（root）执行；否则会把
“收不到反馈”误判成“任务没执行”。`scripts/dev/check_task_ingress.sh` 在 nhy 下同样会误报。

### 3.2 其它

- `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` 必须与已运行栈一致；不一致时连节点都看不见。
- 板端 `/opt/ros/humble` 还装有一份 apt 版 `nav2_costmap_2d`（lib + include）。若
  `LD_LIBRARY_PATH` 或 source 顺序把它排在 workspace 之前，会加载 stock 库 → 症状和 ABI 错配一样。
  正常 controller_server 的 `RUNPATH` 里 `/opt/ros/humble/lib` 排在 workspace 之后（用
  `readelf -d .../controller_server | grep RUNPATH` 核对）。

---

## 4. ABI / 构建一致性检查

### 4.1 为什么会发生

- 本工作区 `build.sh` 用 `--symlink-install`：`install/*/lib/*.so` 是**软链指向 build/** 的实体。
  所以“看 install 版本”要看 build 里真实文件；只删 install 不删 build 无效。
- 源码树从 Windows 挂载拷贝而来时，部分源文件 mtime 是 1970-01-01。`make` 会认为它们比已有
  `.o` 旧而**不重新编译**——结果 `.so` mtime 很新、里面却是旧代码。这是“重编了但特性没进去”的经典根因。
- 只重编了 A 包、没重编依赖 A 头文件的 B 包（如 `nav2_controller` 用了旧 `nav2_costmap_2d` 头文件），
  就会得到“新控制器 + 旧 costmap 库”的组合。

### 4.2 核对命令（root）

```bash
# 1) 实际加载了哪份库（跨 build/install、跨 workspace/system 一眼看出）
sudo grep -oE "/[^ ]*(nav2_costmap_2d_core|liblayers|regulated_pure_pursuit|controller_server_core)\.so" \
  /proc/$(pgrep -f controller_server)/maps | sort -u

# 2) 特性标记（新旧库一眼区分：新特性字符串是否存在）
strings install/nav2_costmap_2d/lib/libnav2_costmap_2d_core.so | grep -c visualization_ground

# 3) 运行期参数存在性（旧库会 Parameter not set）
ros2 param get /robot_1/local_costmap/local_costmap visualization_ground_reference_frame

# 4) 二进制 RUNPATH 顺序
readelf -d install/nav2_controller/lib/nav2_controller/controller_server | grep -E "RPATH|RUNPATH"
```

### 4.3 修复与验证

1. 先 `find` 相关包的 `.cpp/.hpp` 并 `touch`（消除 1970 mtime），或直接删掉对应 `build/`+`install/`
   目录，再一次性重编依赖链上所有包（典型三件套：`nav2_costmap_2d` + `nav2_controller` +
   `nav2_regulated_pure_pursuit_controller`，以及 `nav2_core`/`nav2_util` 这类被多个包 include 的接口包）。
2. 用 4.2 的命令验证：特性字符串出现、参数能读到、`/proc/<pid>/maps` 里三份库同源同次构建。
3. 已知案例（板端旧 build 观察到）：`install/nav2_costmap_2d` 一度是旧构建，
   `visualization_ground_reference_frame` 参数在运行节点上 `Parameter not set`——直接后果见 §5。

---

## 5. 代价地图在 RViz 的可视化排障

代价地图不是 3D 网格，而是 2D `nav_msgs/OccupancyGrid`，RViz 用 `Map` display 把它当成贴在
`origin.position.z` 高度上的一块平面纹理渲染。

- 发布话题：`/<ns>/local_costmap/costmap`（`RELIABLE / TRANSIENT_LOCAL / depth 1`，5 Hz）。
  栅格值经 `cost_translation_table` 映射成 0–100（100 红 = lethal，0 透明，-1 灰 = unknown）。
  `costmap_raw`（`nav2_msgs/Costmap`，0–255）需 `nav2_rviz_plugins` 的 Costmap display。
- **z 平面**：`costmap_2d_publisher.cpp::visualizationGroundZ()` 把
  `visualization_ground_offset_z`（地面在 base_link 的高度，本机 -0.3165）经 TF 变换到 map 帧，
  得到地面 z 作为 `origin.z`。为什么需要它：map 帧原点可能离地面很远（板端仿真曾观察到
  `odom→base_link` 的 z ≈ -63 m），不做 z 修正栅格平面会飘在机器人上方几十米。
- **`visualization_ground_reference_frame: base_link` 不用手写 namespace**：`robot_launch/bringup/nav2.py`
  的 `FRAME_PARAM_NAMES` 会把裸 `base_link` 重写为 `<ns>/base_link`。所以若看到 z=0，**先怀疑是旧库**
  （见 §4.2 的 `Parameter not set`），而不是“frame 名没加前缀”。
- RViz 侧：`Map` display 的 Topic 配 `/<ns>/local_costmap/costmap`、Durability `Transient Local`、
  Reliability `Reliable`、Color Scheme `costmap`；Fixed Frame 用 `<ns>/map`（消息 frame_id 是
  `<ns>/map`）。`Subscription count = 0` 说明当前没有任何 RViz 在订阅，先确认 display 已启用。
- 注意 `always_send_full_costmap: true` 时 5 Hz 全量约 244 KB（约 1.2 MB/s），远程 RViz 走 WiFi 会吃带宽。

---

## 6. 修复清单（与 §2 对应）

| 位置 | 建议 | 风险 |
|---|---|---|
| `MutilTask-dog` mission_publisher.cpp | 删 5s sleep（改异步）；`uav_idx` 不匹配打 WARN；发布前等订阅者 + 重发或 TRANSIENT_LOCAL | 低 |
| `MutilTask-dog` mission_publisher.cpp | 143 补 `targets` + `center`；145 走多航点分支填 `waypoints` | 需明确 UE 侧字段语义 |
| `dispatcher.py` | `current` 加执行看门狗（超时 cancel + finish）；`_ownership_error` 受控恢复；`_pending_finish` 超时兜底 | 中（改调度语义，需回归 test） |
| `mission_node` | 订阅 `/<ns>/task_feedback` 并把 `rejected/executing/failed` 转给 UE；过滤 `task_id=0` 空闲心跳 | 低 |

> 忙碌拒收（无队列）已是 `outdoor_nav_1110` 的现有行为，无需再改；旧版“队列去重/上限”建议不再适用。
> 改 `dispatcher.py` 属调度语义变更，改后跑 `task_pipeline` 的 pytest
> （尤其 `test_dispatcher_ownership.py`、`test_primitive_entrypoints.py`）。

---

## 7. 交接注意

- 最新源码以 `outdoor_nav_1110` 为准；板卡 `~/outdoor_nav_1105` 仍在跑旧 build，二者 dispatcher 语义不同
  （旧版有 `queued` 队列，新版忙碌直接 `rejected`）。引用行为前先确认对的是哪份。
- 板卡 `~/outdoor_nav_1105` 有大量未提交改动，`git status --short` 为准，**不要 reset/clean/整仓覆盖**。
- 构建以 `nhy` 用户执行（`sudo -u nhy bash -lc '... build.sh ...'`），避免 install/build 出现 root 所有文件。
- 只重编受影响包即可，但消息定义改动要连 `task_interfaces` 一起建；Nav2 相关按 §4 三件套一起建。
- 排障命令统一在 root（与栈同用户）下跑，否则受 §3.1 共享内存隔离影响。
- 本文标注“快照”的结论（mission_node 映射、旧 .so 缺失特性等）在别人同步代码后可能已变，引用前先核对对应文件。
