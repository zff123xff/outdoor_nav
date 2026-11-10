# 仿真导航工作流

板卡源码交接总览与当前限制见 [DEVELOPER_HANDOVER.md](DEVELOPER_HANDOVER.md)。

`launch_sim.sh` 启动导航并接入已运行的 UE/仿真桥，不启动 UE/Gazebo。板卡默认 namespace 为 `robot_1`，
`use_sim_time` 默认 false；仿真/传感器时间源须与此设置一致。视频和 YOLO 默认随导航入口启动，可分别关闭。

```bash
cd /home/nhy/outdoor_nav_1105
source /opt/ros/humble/setup.bash
source install/local_setup.bash
bash launch_sim.sh robot_namespace:=robot_1
```

常用参数示例：

```bash
bash launch_sim.sh robot_namespace:=robot_17 rviz:=false
bash launch_sim.sh video_stream:=false yolo:=false
bash launch_sim.sh inspection_udp:=true inspection_input_port:=9101 \
  inspection_ue_address:=192.168.1.186 inspection_output_port:=9001
```

启动后先确认 `/clock` 与 `use_sim_time` 一致、`/<ns>/odom`、`/<ns>/livox/lidar`、map/base/livox TF 更新，
再提交运动任务。任务 topic、编号、取消和状态定义见 [TASK_PIPELINE.md](TASK_PIPELINE.md)。

区域搜索：`bash run_primitive.sh area_search --coordinate-frame enu --check` 只做检查；实际执行移除 `--check`。
当前板卡 area_search YAML 选择 `parallel_lines`，覆盖完成条件和点云掉线行为见
[区域搜索入场与完成条件](AREA_SEARCH_ENTRY.md)。

仿真 143 的 `inspection_search_mode:=true` 当前存在与固定 `simulation_targets/target_interaction_enabled`
标志的优先级冲突；启用前需对照交接总览完成修复和验证。
