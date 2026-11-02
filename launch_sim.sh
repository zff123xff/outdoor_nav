#!/usr/bin/env bash
# Start navigation and tasks against an independently running simulator.
set -eo pipefail
nav_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == '-h' || "${1:-}" == '--help' ]]; then
    echo 'Usage: bash launch_sim.sh [robot_namespace:=robot_1] [use_sim_time:=false] [rviz:=false] [mqtt:=true] [params_file:=/absolute/nav.yaml]'
    echo 'Start sim_ws/simulation/start.sh separately, using the same ROS domain, namespace and GPS origin.'
    exit 0
fi
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
[[ -r "$nav_ws/install/local_setup.bash" ]] || { echo 'Build outdoor_nav first.' >&2; exit 1; }
source "$nav_ws/install/local_setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/outdoor_nav_ros}"
exec ros2 launch robot_launch simulation.launch.py "$@"
