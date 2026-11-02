# ExecuteTask interface

This package defines the `execute_task_server/srv/ExecuteTask` service type. In
the standard navigation bringup, the service server
is `task_pipeline/pipeline_dispatcher`. This package contains only the service
interface; task routing and service handling live in `task_pipeline`.

## Task flow

```text
MQTT → MultiTaskBridge ─┐
manual point sender ────┼→ /agent/<robot>/execute_task
manual patrol sender ──┘  → task_pipeline dispatcher
                             ├─ one GPS waypoint → PointGoal Action executor
                             ├─ patrol_line → Patrol Action executor
                             └─ generic waypoint list → /<robot>/waypoint_goals
                                  → multi_goal_to_gp_goal → /<robot>/navigate_to_pose
```

Action feedback and final results are published as JSON on
`/<robot>/task_feedback`. With MQTT enabled, `swarms_backend` forwards the same
payload to `/mqtt/task_deploy/execution_feedback/<vehicle_id>`. The ExecuteTask
service response reports scheduling acceptance; it is not an arrival result.

## Request example

```bash
ros2 service call /agent/robot_1/execute_task execute_task_server/srv/ExecuteTask \
  '{request_json: "{\"mission_id\":\"point_demo\",\"agent_id\":\"robot_1\",\"capability_id\":\"goto_waypoints\",\"primitive_id\":\"point_001\",\"params\":{\"waypoint_frame\":\"gps\",\"waypoints\":[{\"lat\":40.9446353,\"lon\":113.172819,\"alt\":1298.734571}]}}"}'
```

Use `capability_id: patrol_line` with exactly two GPS waypoints to dispatch the
Patrol Action. `max_laps` defaults to one; zero means indefinite patrol.

## Build and launch

```bash
colcon build --packages-up-to task_pipeline --symlink-install
source install/setup.bash
ros2 launch robot_launch simulation.launch.py robot_namespace:=robot_1
```

For ready-to-edit YAML tasks, use `ros2 run task_pipeline send_point_goal` or
`ros2 run task_pipeline send_patrol`; both go through this service and print
pipeline feedback.
