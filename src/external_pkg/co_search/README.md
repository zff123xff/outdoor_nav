# 完整移植的原生 co-search 区域搜索

`co_search_nav` 是唯一的区域搜索后端，提供 `AreaSearch` Action。输入与任务队列保持不变：GPS 矩形中心、长宽和方向角；输出为交给 Nav2 `FollowPath` 的地图坐标路径。

## 算法链路

实时点云/里程计经 TF 变换进入矩形局部坐标系，调用原生 `FastExplorationManager::updateFrontierStruct` 和 `planExploreMotion`：原生占据/ESDF 建图、边界聚类与视点采样、网格/巡回选择、局部多视点搜索、SNN 波前/A* 路径、原生多项式初始化和 NLopt B 样条优化。适配器仅采样原生优化曲线并转回地图坐标，Nav2 保留速度控制和碰撞检查。

只有完整移植版本；Python `GroundGrid` 简化搜索和旧区域搜索实现已删除，不提供切换或回退后端。区域覆盖按原生地图在任务矩形内的实际观测单元计算；新观测连续确认可达边界耗尽后结束，不用覆盖阈值替代原算法结束判定。

## 源码来源与单狗部署

`upstream/` 来自现有完整移植快照，原始项目路径为 `/mnt/d/shixi/co-search-main/src/swarm_exploration_ros2`。`UPSTREAM_SHA256.json` 记录原始源文件哈希，保留原始许可。恢复自相邻工作区已完成适配的 co-search 版本，原生搜索、路径与优化源文件保持其完整实现；地图输入、坐标、障碍高度和构建适配见代码及来源校验测试。

单狗运行固定原生 ID=1、数量=1，关闭多机区域分配。只启动本仓库的任务 Action 适配器和原生 `search_engine`；原 `exploration_node`、`ground_node`、飞行 FSM、轨迹命令入口及其启动文件已删除，构建只保留单机算法所需库与接口。原算法所需数据结构及计算库保留，不恢复旧集群 MQTT backend、任务桥或 ExecuteTask 服务。

## 配置与构建

唯一原生算法运行配置为 `co_search_nav/config/area_search.yaml`；任务示例位于 `task_pipeline/config/examples/{gps,enu}/area_search.yaml`。用 `co_search.parameters` 的原始斜杠参数名覆盖算法参数；`plan_period_s`、`map_period_s`、`path_spacing_m` 控制适配周期和采样密度。原生障碍高度范围在该文件中配置，Nav2 costmap 的高度范围由导航 YAML 单独控制。

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
colcon build --packages-up-to co_search_nav task_pipeline --symlink-install --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
bash run_primitive.sh area_search
```

原生构建需要 PCL、Eigen、OpenCV、NLopt 等依赖。首次构建必须使用 `--packages-up-to` 构建原生依赖；已经构建过依赖后可 `--packages-select co_search_nav`。

Action 取消会先取消所有已接受/在途 FollowPath，再停止原生进程和释放导航占用；旧路径结果不能推进新任务。完整任务接口见 [任务链路](../../../docs/TASK_PIPELINE.md)，直接下发见 [ROS 示例](../../../docs/PRIMITIVE_ROS_EXAMPLES.md)。
