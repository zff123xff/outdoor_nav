# outdoor_nav

ROS 2 Humble navigation for the Go2-W robot. The supported navigation entry points are `launch_sim.sh` for an independently running simulator and `launch_real.sh` for the physical robot. They start navigation and its primitive Action executors; task ingress and task dispatch run in the adjacent `sim_ws/commands` workspace.

```text
sim_ws MQTT YAML → ExecuteTask → PointGoal or Patrol executor
                    → navigate_to_pose Action → straight_planner → FollowPath → controller_server
```

## Build

Build the workspace after installing its dependencies:

```bash
cd /mnt/d/shixi/south_lake_institution/outdoor_nav
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
bash build.sh
```

The physical robot also needs the Go2-W SDK, MAVROS, and MID360 drivers under `/3rd`; see [hardware driver installation](docs/HARDWARE_DRIVERS_3RD.md) and [NX deployment](deploy/nx/README.md).

## Simulation

The workspaces have separate responsibilities. `sim_ws` owns Gazebo, models, sensors, PX4 simulated localization and low-level controllers. `outdoor_nav` owns Nav2, RViz, velocity arbitration and the task pipeline. Each workspace builds independently; they communicate over ROS. See [workspace boundaries](docs/WORKSPACE_BOUNDARIES.md).

```bash
# Terminal 1, in sim_ws/simulation: start the simulated robot
bash start.sh robot_namespace:=robot_1

# Terminal 2, in outdoor_nav: start navigation and tasks
bash launch_sim.sh robot_namespace:=robot_1
```

Use the same ROS domain and namespace in both terminals. With no domain override both use the ROS default domain 0. Navigation uses the shared navigation configuration `robot_launch/config/go2w_nav2.yaml`; the simulator GPS origin lives in its own spawn/localization configuration, whose reference coordinates must match the navigation origin. Changes to the origin require restarting both processes. See [simulation workflow](docs/SIM_NAVIGATION_WORKFLOW.md).

## Primitive validation through the navigation pipeline

After starting navigation, submit primitives directly through ExecuteTask:

```bash
bash run_primitive.sh point_goal --dry-run
bash run_primitive.sh point_goal --check
bash run_primitive.sh point_goal
bash run_primitive.sh patrol
bash run_primitive.sh area_search
```

Edit the corresponding YAML in `src/external_pkg/task_pipeline/config/`, or pass `--file /path/task.yaml`. Use `--namespace robot_2` to select another agent. This entry calls the navigation pipeline and waits for task feedback; MQTT is optional. `sim_ws/commands` separately validates publishing primitives through MQTT. See [the workflow](docs/SIM_NAVIGATION_WORKFLOW.md).

## Physical Go2-W

Configure `GO2W_NET_IFACE` and the DDS network settings, then run:

```bash
bash launch_real.sh robot_namespace:=robot_1
```

This starts the Go2-W driver, MAVROS/PX4, odometry relay, Nav2, and the task pipeline. To enable MQTT task input, pass `mqtt:=true` and set the broker arguments such as `address:=tcp://<broker>:1883`. `setup_go2w.sh` expects the physical drivers under `/3rd`; it does not start nodes by itself.

The separate Beijing HIL connector is maintained under `deploy/hil/launch.sh` and invoked on NX with `deploy/nx/run.sh hil`.

## Task entry points

The command pipeline and per-primitive YAML files are maintained in the adjacent `sim_ws/commands` workspace. Submit tasks there over MQTT; this workspace supplies the Nav2 Action executors and navigation interfaces:

Edit `../sim_ws/commands/src/task_pipeline/config/{point_goal,patrol_goal,area_search}.yaml`, then send with `bash ../sim_ws/commands/send.sh point_goal`, `patrol`, or `area_search`.

Area search is submitted with `send_area_search`; its feedback reports searched area and coverage. Detailed usage:

- [Point goal](docs/POINT_GOAL_USAGE.md)
- [Patrol](docs/PATROL_NODE_USAGE.md)
- [Area search](docs/AREA_SEARCH_USAGE.md)
- [Navigation chain and simulator checks](docs/SIM_NAVIGATION_WORKFLOW.md)

The ENU origin for manual tasks is read from the running `nav2_input_bridge`; task YAMLs do not copy the navigation origin. Navigation profiles and their single-source parameters live in `src/robot_launch/config/`.

## Repository layout

- `src/robot_launch/launch/`: exactly two public entries: `real.launch.py` and `simulation.launch.py`.
- `src/robot_launch/robot_launch/bringup/`: internal hardware, navigation and task composition.
- `src/robot_launch/config/`: simulation and hardware Nav2 profiles.
- `src/external_pkg/patrol_node/`: navigation-side point and patrol Action executors.
- `src/external_pkg/task_pipeline/`: navigation-side area-search executor and implementation shared with the sim command workspace; task ingress is owned by `sim_ws/commands`.
- `src/external_pkg/patrol_node/`: PointGoal and Patrol Actions.
- `src/external_pkg/multi_goal_to_gp_goal/`: GPS waypoint Action dispatcher.
- `deploy/`: NX and HIL launch/install helpers.
- `scripts/dev/`: isolated module launchers for development and diagnosis; these do not replace the two full-system entry points.

All simulators use the same navigation launch and go2w_nav2.yaml. Navigation consumes odometry and pointclouds and publishes velocity commands. Set use_sim_time:=true only when the source uses /clock. Neither navigation entry starts a simulator.
