#!/usr/bin/env bash
# Launch MAVROS only (for standalone PX4 testing)

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

PLUGINLISTS_YAML="${SCRIPT_DIR}/install/robot_launch/share/robot_launch/config/mavros_pluginlists.yaml"

python3 - <<'PYTHON'
from launch import LaunchService
from robot_launch.bringup.mavros import generate_launch_description
service = LaunchService()
service.include_launch_description(generate_launch_description())
raise SystemExit(service.run())
PYTHON
