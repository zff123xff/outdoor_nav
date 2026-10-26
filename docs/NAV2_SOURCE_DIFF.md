# 项目 Nav2 与官方 1.1.18 差异

对比日期：2026-10-21。只读检查源码，没有修改导航实现或配置，没有启动机器人。

基准：[官方 navigation2 1.1.18 标签](https://github.com/ros-navigation/navigation2/tree/1.1.18)。本地官方包的 package.xml 均标注 1.1.18，因此比较同版本，而非随时间变化的 humble 分支。该版本标签作为可复现比较基准，不代表已证明每个文件的原始导入提交。nav_2d_msgs、nav_2d_utils 的上游路径位于 nav2_dwb_controller 下。

## 整体结构

本地共有 13 个包：11 个来自官方包布局，另有 2 个自定义包 nav2_input_bridge、nav2_straight_planner；不是完整的官方导航栈。当前链路为 GPS 输入桥 → 直线规划器 → FollowPath → 修改版 RPP 控制器 → cmd_vel_nav。没有使用官方 bt_navigator/行为树驱动这条 HIL 链路。

11 个官方对应包中，267 个同路径文件字节完全一致，18 个同路径文件不同，额外加入 13 个文件；没有删除这些包内的上游文件。18 处文件差异包含构建脚本、README、测试和纯格式差异，不能理解为 18 个算法改动。两个自定义包不计入这些文件统计。

## 实际功能差异

| 包 | 相对官方版本的变化 | 当前配置是否使用 |
| --- | --- | --- |
| nav2_regulated_pure_pursuit_controller | 嵌入 Theta* 局部规划；每个速度计算周期先从全局路径挑选局部地图内可用目标，生成局部路径，再由 RPP 跟踪；新增转向接近目标角时的减速上限 | 是；FollowPath 指向该修改版 RPP |
| nav2_costmap_2d / ObstacleLayer | 新增 clear_before_update，开启后每轮更新前重置本层栅格，再进行清除与障碍标记 | 是；go2w_nav2.yaml、zny_hil.yaml 和仿真 nav.yaml 都配置 true |
| nav2_costmap_2d / InflationLayer | onFootprintChanged 将计算 inscribed_radius_ 的来源由 footprint 内切半径换成外接半径，并增加打印 | 当前配置加载 InflationLayer，因此生效；相对官方会改变障碍附近高代价区域 |
| nav2_controller | 新增 PositionGoalChecker，只检查 XY 位置而忽略最终朝向；新增 publish_zero_velocity，控制成功返回时是否发零速度 | 当前使用 SimpleGoalChecker，新增位置检查器未选用；publish_zero_velocity 未显式配置，默认 true，与官方成功时发零速度一致 |
| nav2_msgs | 增加 Route/RouteNode/RouteEdge/EdgeCost，以及 ComputeRoute、ComputeAndTrackRoute、DynamicEdges、SetRouteGraph 等接口 | 当前导航链路未发现对应路由服务实现或调用；新增接口本身不代表具备完整路由功能 |
| nav2_core | 额外加入 route_exceptions.hpp | 当前链路未发现调用 |
| nav2_util | robot_utils.hpp 补充 memory 头文件 | 编译层面的差异 |
| nav2_lifecycle_manager | README 字节不同，源码与官方一致 | 生命周期行为未因包内源码改变；项目 launch 外另有服务 relay/watchdog，属于启动适配层 |

ObstacleLayer 还在两个 LaserScan 回调里加入了重复日志和硬编码 map/livox_frame_flat 的 TF 调试查询；当前配置输入为 PointCloud2，这两个 LaserScan 分支不是主链路。其他 costmap 差异是缩进和空白调整。

完全相同的包：nav2_common、nav2_voxel_grid、nav_2d_msgs、nav_2d_utils。

## 两个项目自定义包

- nav2_input_bridge：GPS/UAV NavSatFix 目标转换为地图 goal_pose，target_detected 触发停止规划与取消 FollowPath，订阅 odom 发布 odom→base_link。与项目 Git HEAD 相比，当前只保留 TF 的 z 使用 odom.z 一处修正；外部原点接入等实验性修改已撤回。
- nav2_straight_planner：生命周期节点，按当前位置到目标生成直线参考路径，定时发送 FollowPath，接收 stop_planner，并处理到达和取消。该节点本身没有基于障碍地图搜索绕行路径；绕障局部规划位于修改版 RPP 内的 Theta*。

## 文件清单

### nav2_controller

同路径有差异：

- `CMakeLists.txt`
- `include/nav2_controller/controller_server.hpp`
- `plugins.xml`
- `src/controller_server.cpp`

本地新增：

- `include/nav2_controller/plugins/position_goal_checker.hpp`
- `plugins/position_goal_checker.cpp`

### nav2_core

本地新增：

- `include/nav2_core/route_exceptions.hpp`

### nav2_costmap_2d

同路径有差异：

- `include/nav2_costmap_2d/denoise/image_processing.hpp`
- `include/nav2_costmap_2d/obstacle_layer.hpp`
- `plugins/inflation_layer.cpp`
- `plugins/obstacle_layer.cpp`
- `src/costmap_2d_ros.cpp`
- `test/unit/denoise_layer_test.cpp`
- `test/unit/image_processing_test.cpp`

### nav2_lifecycle_manager

同路径有差异：

- `README.md`

### nav2_msgs

同路径有差异：

- `CMakeLists.txt`

本地新增：

- `action/ComputeAndTrackRoute.action`
- `action/ComputeRoute.action`
- `msg/EdgeCost.msg`
- `msg/Route.msg`
- `msg/RouteEdge.msg`
- `msg/RouteNode.msg`
- `srv/DynamicEdges.srv`
- `srv/SetRouteGraph.srv`

### nav2_regulated_pure_pursuit_controller

同路径有差异：

- `CMakeLists.txt`
- `include/nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp`
- `src/regulated_pure_pursuit_controller.cpp`
- `test/test_regulated_pp.cpp`

本地新增：

- `include/nav2_regulated_pure_pursuit_controller/theta_star.hpp`
- `src/theta_star.cpp`

### nav2_util

同路径有差异：

- `include/nav2_util/robot_utils.hpp`

## 构建与部署影响

不能把项目内 Nav2 当成与系统安装包完全相同的副本。直接改用 /opt/ros/humble 的官方 RPP，会失去此处嵌入的 Theta* 局部绕障路径；clear_before_update 自定义行为也不会存在。扩展消息和 C++ 类成员变化还需要保证消费者使用匹配的头文件与库。

colcon 的“覆盖 underlay 同名包”警告有实际背景：本地包名、版本可能与系统相同，但实现不同。是否精简本地包需要单独梳理依赖和实际运行库来源；本次仅做差异审查，没有进行替换或删除。

机器可读清单：[nav2_1_1_18_diff.json](validation/nav2_1_1_18_diff.json)。该清单记录比较基准和上游压缩包 SHA256，可用于后续复核。
