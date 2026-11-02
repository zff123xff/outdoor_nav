# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 Humble navigation workspace for the Go2-W. ROS packages live under `src/`: `robot_launch` owns the two public system launch files and navigation configuration; `navigation2` contains Nav2 and navigation-specific packages; `external_pkg` contains the task pipeline, area search, patrol actions, and supporting packages; `driver` contains hardware drivers. `deploy/` holds NX and HIL helpers, `scripts/dev/` has diagnostic launchers, and `tests/` contains workspace-level checks. The simulator itself belongs to the separate `sim_ws` workspace.

## Build, Test, and Development Commands

- `source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"` selects the ROS environment for this shell.
- `bash build.sh` builds the workspace with symlink install, skipping `fast_lio` and `go2w_driver` by default. Pass colcon options after the script as needed.
- `source install/local_setup.bash` makes built packages available in the current shell.
- `bash launch_sim.sh robot_namespace:=robot_1` starts navigation against an already-running simulator. `bash launch_real.sh robot_namespace:=robot_1` starts the physical robot stack and requires its configured hardware drivers.
- `bash run_primitive.sh point_goal|patrol|area_search` validates primitives directly through the navigation pipeline; use `--dry-run` or `--check` before execution. MQTT publication validation belongs to sim_ws/commands.
- Run focused Python checks with `python3 -m pytest -q tests/test_navigation_completion.py src/external_pkg/task_pipeline/test`; `python3 scripts/check_sim_navigation.py` checks navigation/simulator interface assumptions.

## Coding Style & Naming Conventions

Use four spaces for Python indentation and follow the surrounding ROS 2 package conventions. Keep Python modules and ROS package names lowercase with underscores; use descriptive `test_*.py` names. Keep navigation parameters in `src/robot_launch/config/`, task examples in the owning package's `config/`, and hardware or simulator-specific settings in their respective workspace. Preserve the boundary that navigation launch files do not start or configure the simulator.

## Testing Guidelines

Add focused tests alongside the owning package under `test/`, or workspace-wide regression checks under `tests/`. Pure logic tests should avoid requiring a live robot. ROS integration checks should use a bounded, isolated ROS domain and clean up any processes they start. Run the relevant pytest files and interface checks after changing task behavior, launch wiring, or configuration parsing.

## Commit & Pull Request Guidelines

Recent history uses concise Chinese summaries and Conventional Commit prefixes such as `feat(nav):`, `fix(nav):`, and `chore(livox):`; keep commits focused and describe the affected subsystem. Pull requests should explain the behavior change, list affected packages and configuration, include build/test commands and outcomes, and link related issues. Include screenshots or ROS logs only when they clarify visualization or runtime behavior.

## Configuration & ROS Runtime

Use the selected navigation YAML as the source of navigation parameters. Keep robot namespaces consistent across launch arguments and ROS topics/TF frames. Source ROS and workspace setup files in the same shell before using `ros2` commands, and preserve the caller's ROS domain and DDS configuration.
