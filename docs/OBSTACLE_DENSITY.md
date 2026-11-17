# Nav2 点云栅格障碍点数阈值

参数位于 `src/robot_launch/config/go2w_nav2.yaml` 的
`local_costmap.local_costmap.ros__parameters.obstacle_layer`，与 co-search native 地图使用不同缓存和参数。

板卡当前配置：

```yaml
resolution: 0.1
min_points_per_cell: 5
point_count_window_s: 3.0
pointcloud:
  observation_persistence: 3.0
  height_reference_frame: base_link
  height_reference_ground_z: -0.31
  min_obstacle_height: 0.2
  max_obstacle_height: 1.5
```

通过有限坐标、距离、高度和地图范围过滤后，最近 3 秒内同一平面栅格至少累计 5 个有效点才会标记为障碍。
格子边长是 0.1 m；不同高度的点仍累计在同一 XY 格子内，这不是三维体素密度。
源缓存 `observation_persistence` 至少要覆盖 `point_count_window_s`；板卡当前两者均为 3 秒。
`clear_before_update` 已启用，窗口过期后旧标记会随地图更新清除。射线清除流程由原插件继续处理。

当前点云高度带相对 `base_link` 的地面参考 `z=-0.31 m` 计算，障碍范围为离地 0.2–1.5 m。
这是 Nav2 避障配置；co-search 的 `sdf_map/ground_height` 和高度带独立，不能只改一边就认为两边同步。

运行时可在 local costmap 节点上调整并查看：

```bash
ros2 param get /robot_1/local_costmap/local_costmap obstacle_layer.min_points_per_cell
ros2 param get /robot_1/local_costmap/local_costmap obstacle_layer.point_count_window_s
```

提高点数阈值能过滤稀疏噪点，也可能漏掉远处、细小或低反射障碍。当前数值是部署配置，不应在未复核实际
雷达回波和地面误标的情况下视为经过安全标定。
