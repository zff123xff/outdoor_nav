#!/usr/bin/env bash
# One entry point for the physical Go2-W, navigation, and task pipeline.
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
namespace="${NAMESPACE:-robot_1}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    cat <<'HELP'
Usage: bash launch_real.sh [robot_namespace:=robot_1] [mqtt:=false]
                          [fcu_url:=/dev/ttyACM0:230400]

Starts the Go2-W driver, MAVROS/PX4, Nav2, and task pipeline. Requires
setup_go2w.sh configuration (including GO2W_NET_IFACE) and installed /3rd
drivers. MQTT is optional; enable it with mqtt:=true.
HELP
    exit 0
fi

source "$workspace/setup_go2w.sh"
[[ -r "$workspace/install/local_setup.bash" ]] || {
    echo "Missing workspace install; build outdoor_nav first." >&2
    exit 1
}
source "$workspace/install/local_setup.bash"

has_namespace=false
for launch_arg in "$@"; do
    if [[ "$launch_arg" == robot_namespace:=* ]]; then
        has_namespace=true
        break
    fi
done
if [[ "$has_namespace" == false ]]; then
    set -- "robot_namespace:=$namespace" "$@"
fi

echo "[real robot] outdoor_nav=$workspace namespace=${namespace} domain=${ROS_DOMAIN_ID:-0}"
exec ros2 launch robot_launch real.launch.py "$@"
