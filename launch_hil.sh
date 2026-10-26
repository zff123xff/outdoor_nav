#!/usr/bin/env bash
# Beijing HIL navigation entry point. Does not source real-dog network settings.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    cat <<'HELP'
Usage: bash launch_hil.sh [robot_namespace:=robot_1] [use_sim_time:=false]
                          [params_file:=/absolute/path/nav.yaml] [use_foxglove:=false]
Namespace: params_file launch_config.ros__parameters.robot_namespace (fallback robot_1).
Command-line robot_namespace overrides the YAML. Default clock: system time.
Beijing ZNY mode: navigation publishes TF from received odometry (including z) and fixed lidar extrinsics.
Only odom/pointcloud inputs are required; simulator TF and /clock are not required.
Set use_sim_time:=true only when the simulator publishes /clock.
Set HIL_INSTALL_DIR to select install or install_nx explicitly.
Inherited ROS_DOMAIN_ID, RMW_IMPLEMENTATION and DDS settings are preserved.
No Go2-W, MAVROS, MID360, auto-origin or remote-control nodes are launched.
HELP
    exit 0
fi

# Use the shared ROS helper when available; otherwise use the NX system ROS.
if [[ -r "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash" ]]; then
    source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
else
    source /opt/ros/humble/setup.bash
fi

if [[ -n "${HIL_INSTALL_DIR:-}" ]]; then
    hil_install="$HIL_INSTALL_DIR"
elif [[ -r "$SCRIPT_DIR/install_nx/setup.bash" ]]; then
    hil_install="$SCRIPT_DIR/install_nx"
else
    hil_install="$SCRIPT_DIR/install"
fi
[[ -r "$hil_install/local_setup.bash" ]] || {
    echo "[HIL] Missing workspace: $hil_install/local_setup.bash. Build outdoor_nav first." >&2
    exit 1
}
source "$hil_install/local_setup.bash"
# ros2 pkg also loads RMW: do not misreport middleware load failures as
# missing twist_mux after suppressing the package lookup's stderr.
if ! python3 -c 'from rclpy.utilities import get_rmw_implementation_identifier; print("[HIL] RMW: " + get_rmw_implementation_identifier())'; then
    echo "[HIL] Cannot load ROS middleware ${RMW_IMPLEMENTATION:-default}. Check its installation and library paths before checking navigation dependencies." >&2
    exit 1
fi
# Native local dependencies belong to this navigation workspace.
if ! ros2 pkg prefix twist_mux >/dev/null 2>&1; then
    hil_deps_root="$SCRIPT_DIR/hil_deps/ros"
    hil_deps_prefix="$hil_deps_root/opt/ros/${ROS_DISTRO:-humble}"
    if [[ -r "$hil_deps_root/.architecture" && "$(cat "$hil_deps_root/.architecture")" != "$(dpkg --print-architecture)" ]]; then
        echo "[HIL] Local dependencies use a different architecture. Run: bash $SCRIPT_DIR/deploy/hil/setup_deps.sh" >&2
        exit 1
    fi
    if [[ -r "$hil_deps_prefix/share/ament_index/resource_index/packages/twist_mux" ]]; then
        export AMENT_PREFIX_PATH="$hil_deps_prefix:${AMENT_PREFIX_PATH:-}"
        export LD_LIBRARY_PATH="$hil_deps_prefix/lib:${LD_LIBRARY_PATH:-}"
    fi
fi
if ! ros2 pkg prefix twist_mux >/dev/null 2>&1; then
    echo "[HIL] Missing twist_mux. Run: bash $SCRIPT_DIR/deploy/hil/setup_deps.sh (or install the native ROS twist-mux system package)." >&2
    exit 1
fi
hil_share="$(ros2 pkg prefix --share robot_launch)"
[[ -r "$hil_share/launch/outdoor_zny.launch.py" ]] || {
    echo '[HIL] Rebuild robot_launch to install outdoor_zny.launch.py.' >&2
    exit 1
}
echo "[HIL] Workspace: $hil_install; ROS domain: ${ROS_DOMAIN_ID:-0}"
exec ros2 launch robot_launch outdoor_zny.launch.py "$@"
