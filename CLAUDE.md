# Repository notes

## Supported full-system startup

- Local Gazebo: `bash launch_sim.sh`. This launch starts Nav2 and task pipeline only. Start Gazebo separately from sim_ws/launch_sim.sh. Neither workspace sources or starts the other.
- Physical Go2-W: `bash launch_real.sh`. This single launch starts the Go2-W driver, MAVROS/PX4, Nav2, and task pipeline.
- NX/ZNY HIL deployment helper: `bash deploy/nx/run.sh hil`.

Do not run a second Nav2 stack or a separate task-pipeline launch alongside either full-system entry. Only real.launch.py and simulation.launch.py are public launches. Shared composition lives in src/robot_launch/robot_launch/bringup.

## Task flow

Manual YAML clients and optional MQTT use the same chain:

```text
manual/MQTT → ExecuteTask → task_pipeline → PointGoal/Patrol executor
            → navigate_to_pose Action → straight_planner → FollowPath → controller_server
```

Task mission files live in `src/external_pkg/task_pipeline/config`. Their ENU origin is resolved from the running `nav2_input_bridge`; do not copy reference coordinates into task YAML. The active Nav2 YAML is the upstream source for reference coordinates; the controller goal_checker owns arrival tolerances.

## Build and checks

```bash
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
bash build.sh
```

Run package tests from the sourced workspace. Keep `ROS_DOMAIN_ID`, RMW implementation, and DDS settings inherited unless the selected simulator or robot setup explicitly needs a task-local override.

See `README.md`, `docs/`, and `deploy/` for operating instructions.
