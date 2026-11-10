# outdoor_nav 文档入口

维护说明以板卡 `/home/nhy/outdoor_nav_1105` 的源码为准。当前交接基线及源码核实结果见
[开发交接总览](DEVELOPER_HANDOVER.md)。该页记录连接时间、分支/提交、实际 launch、任务接口、
数据链路、关键参数、运行诊断和仍待处理的问题。

## 当前接口和运行

- [开发交接总览](DEVELOPER_HANDOVER.md)：新开发者首先阅读；包括已知实现差异和验证边界。
- [任务接口与执行](TASK_PIPELINE.md)：固定任务编号、字段、状态反馈、忙碌拒收和取消语义。
- [ROS 基元发布示例](PRIMITIVE_ROS_EXAMPLES.md)：ROS 2 CLI 请求、取消和反馈查看。
- [仿真目标巡检](SIMULATION_TARGETS.md)：目标分配、Marker 输入、到达事件和 UDP 对接。
- [整体数据链路](DATA_FLOW.md)：系统模块图及接口边界。
- [仿真导航工作流](SIM_NAVIGATION_WORKFLOW.md)：启动入口与常用命令。

## 算法与配置专题

- [区域搜索入场、覆盖与完成条件](AREA_SEARCH_ENTRY.md)
- [区域搜索点云缓存和 TF 时间处理](AREA_SEARCH_CLOUD_WINDOW.md)
- [区域搜索视点锁](AREA_SEARCH_GOAL_LOCK.md)
- [障碍物点云栅格密度](OBSTACLE_DENSITY.md)
- [绕障路径滞回](PATH_HYSTERESIS.md)
- [里程计恢复与区域搜索传感器诊断](ODOMETRY_RECOVERY.md)
- [速度命令链路](VELOCITY_CHAIN.md)
- [仿真 GPS 原点](SIMULATION_GPS_ORIGIN.md)

## 维护边界和硬件资料

- [工作空间边界](WORKSPACE_BOUNDARIES.md)
- [硬件驱动检查](HARDWARE_DRIVER_CHECK.md) 与 [第三方驱动说明](HARDWARE_DRIVERS_3RD.md)
- [Go2-W SDK 开发指南](Go2-W%20SDK开发指南.md)
- [模块流程图](diagrams/MODULES.md)

`STRIKE_REQUEST.md` 是旧接口移除说明；当前任务接口不含武器控制或打击请求。
`docs/validation/` 保存历史对比数据，不代表板卡当前运行结果。专题页面中的参数以
交接总览标注的源码配置为准；发现数值不一致时应先修正文档，再修改代码。
