# 工作空间职责边界

| 目录 | 负责 | 对外接口 |
|---|---|---|
| sim_ws | 世界、模型、传感器、PX4 仿真定位、CHAMP/轮腿控制 | 发布 clock、传感器、mavros/odom 与模型 TF；接收 cmd_vel |
| outdoor_nav | Nav2、导航配置、导航 TF、RViz、twist_mux、遥控取消、任务链 | 消费传感器/定位；输出 cmd_vel |

两个目录分别构建、分别启动，生产入口不得 source 或启动另一个工作空间。导航侧的 launch_sim.sh 表示连接仿真输入，不负责启动 Gazebo。真机入口仍是 launch_real.sh。

仿真定位使用其自身出生点/定位 YAML；导航使用 robot_launch/config/go2w_nav2.yaml。两端的 reference_latitude、reference_longitude、reference_altitude、ROS 域与命名空间必须一致。默认原点已按当前导航配置同步。

跨目录集成测试保存在 outdoor_nav/scripts：check_sim_navigation.py、check_sim_hybrid.py、check_sim_contract.py；simulation_fixture.launch.py 仅用于测试。模型、传感器和融合测试保留在 sim_ws。检查导航参数运行 python3 scripts/check_sim_navigation.py，需先构建并 source outdoor_nav。

启动命令与参数见 [仿真导航流程](SIM_NAVIGATION_WORKFLOW.md)。迁移前 sim_ws 文件备份位于 /tmp/workspace_split/backup（临时目录，不作为项目依赖）。

导航独占公共 `map → odom → base_link → livox_frame`。雷达安装外参位于 go2w_nav2.yaml 的 lidar_mount 段。仿真只维护私有模型 TF；可视化可以合并显示，不能回写公共 TF。

sim_ws/commands 仅验证单点、巡线、区域搜索基元通过 MQTT 发布。它不启动 broker、ROS 接收网关、调度器或执行器；这些接收与执行职责由 outdoor_nav 的导航入口管理。
