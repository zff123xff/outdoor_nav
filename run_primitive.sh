#!/usr/bin/env bash
# Validate navigation primitives directly through the navigation task pipeline.
set -eo pipefail
nav_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
primitive="${1:---help}"
[[ $# == 0 ]] || shift
case "$primitive" in
    -h|--help)
        echo 'Usage: bash run_primitive.sh [point_goal|patrol|area_search] [--file YAML] [--namespace ID] [--dry-run|--check]'
        echo 'Submit directly to the navigation ExecuteTask pipeline; no MQTT broker is required.'
        exit 0;;
    point_goal) executable=send_point_goal;;
    patrol) executable=send_patrol;;
    area_search) executable=send_area_search;;
    *) echo "Unknown primitive: $primitive" >&2; exit 2;;
esac
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
[[ -r "$nav_ws/install/local_setup.bash" ]] || { echo 'Build outdoor_nav first.' >&2; exit 1; }
source "$nav_ws/install/local_setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/outdoor_nav_ros}"
exec ros2 run task_pipeline "$executable" "$@"
