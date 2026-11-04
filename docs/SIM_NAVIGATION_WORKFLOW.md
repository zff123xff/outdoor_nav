# 仿真导航工作流

启动、接口和状态定义统一见 [任务接口与数据链路](TASK_PIPELINE.md)。

使用 `bash run_primitive.sh --help` 查看四类发送示例。任务经 `/<namespace>/task` topic 入队，由 Action 执行器顺序执行。
