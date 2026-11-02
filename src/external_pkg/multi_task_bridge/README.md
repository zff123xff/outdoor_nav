# MultiTask bridge

`multi_task_bridge` converts backend `MultiTask` messages to the shared
`/agent/<robot>/execute_task` service. It does not command navigation topics or
call executor Actions directly. The service is owned by `task_pipeline`.

Task mapping:

| `SingleTask.task_type` | Pipeline request |
| --- | --- |
| `7` (linear/point task) | `goto_waypoints`; one waypoint is dispatched to the PointGoal executor, multiple waypoints remain a waypoint-list task |
| `11` (cruise/patrol task) | `patrol_line`; `arrive_path` must contain exactly two GPS endpoints; one A→B→A round trip is requested |
| `2` (coverage) | `goto_waypoints` with the generated coverage waypoint list |
| `4` (return) | `return` with the assigned GPS waypoint |
| `12` (takeoff) | `takeoff` |

`ExecuteTask` returns the scheduling/acceptance response. Runtime Action
feedback and terminal results are published by the pipeline on
`/<robot>/task_feedback`. `swarms_backend` forwards these JSON messages to
`/mqtt/task_deploy/execution_feedback/<vehicle_id>`; this is separate from the
legacy UInt8 task-index `stream_feedback` topic.
