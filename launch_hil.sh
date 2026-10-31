#!/usr/bin/env bash
# Beijing HIL navigation entry point. Does not source real-dog network settings.
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
    cat <<'HELP'
Usage: bash launch_hil.sh [--namespace <robot>] [robot_namespace:=robot_1]
                          [use_sim_time:=false]
                          [params_file:=/absolute/path/nav.yaml] [use_foxglove:=false]

Robot selection (per-robot namespace, e.g. robot_1 / robot_a / robot_b):
  -n, --namespace <robot>   Run the navigation stack under /<robot>/...
                              (topics, TF frames and node names all use it).
      --namespace=<robot>   Same, with the '=' form.
  robot_namespace:=<robot>  The raw ROS launch argument; still accepted and
                             equivalent to --namespace <robot>.
  HIL_ROBOT_NAMESPACE=<robot>  Environment variable, used by deploy/nx/run.sh hil.

Precedence: --namespace > robot_namespace:= > params_file
  (params_file launch_config.ros__parameters.robot_namespace) > robot_1.

The namespace must match [A-Za-z][A-Za-z0-9_]* (one non-empty ROS name).

Default clock: system time.
Beijing ZNY mode: navigation publishes TF from received odometry (including z) and fixed lidar extrinsics.
Only odom/pointcloud inputs are required; simulator TF and /clock are not required.
Set use_sim_time:=true only when the simulator publishes /clock.
Set HIL_INSTALL_DIR to select install or install_nx explicitly.
Inherited ROS_DOMAIN_ID, RMW_IMPLEMENTATION and DDS settings are preserved.
No Go2-W, MAVROS, MID360, auto-origin or remote-control nodes are launched.
HELP
}

# --- argument parsing -------------------------------------------------------
# The robot namespace may be given as --namespace/-n, as --namespace=<robot>,
# or as the plain ROS launch argument robot_namespace:=<robot>. The explicit
# --namespace wins over the raw launch argument.
namespace=""
ns_from_arg=""
launch_args=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
        -n|--namespace)
            if [[ $# -lt 2 ]]; then
                echo "[HIL] $1 needs a robot namespace, e.g. $1 robot_2" >&2
                exit 2
            fi
            namespace="$2"
            shift 2
            ;;
        --namespace=*)
            namespace="${1#*=}"
            shift
            ;;
        robot_namespace:=*)
            ns_from_arg="${1#robot_namespace:=}"
            launch_args+=("$1")
            shift
            ;;
        *)
            launch_args+=("$1")
            shift
            ;;
    esac
done

if [[ -n "$namespace" ]]; then
    if [[ ! "$namespace" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]]; then
        echo "[HIL] Invalid robot namespace '$namespace': expected [A-Za-z][A-Za-z0-9_]*" >&2
        exit 2
    fi
    # Drop a conflicting robot_namespace:= argument so the launch file does not
    # see the same argument twice (ROS launch rejects duplicates).
    kept=()
    for arg in "${launch_args[@]}"; do
        case "$arg" in
            robot_namespace:=*)
                echo "[HIL] Ignoring '$arg' (--namespace $namespace takes precedence)" >&2
                ;;
            *)
                kept+=("$arg")
                ;;
        esac
    done
    launch_args=("${kept[@]}")
    launch_args+=("robot_namespace:=$namespace")
elif [[ -n "$ns_from_arg" ]]; then
    # Already in launch_args; only validate and use it for the banner.
    namespace="$ns_from_arg"
elif [[ -n "${HIL_ROBOT_NAMESPACE:-}" ]]; then
    namespace="$HIL_ROBOT_NAMESPACE"
    launch_args+=("robot_namespace:=$namespace")
fi

if [[ -n "$namespace" && ! "$namespace" =~ ^[A-Za-z][A-Za-z0-9_]*$ ]]; then
    echo "[HIL] Invalid robot namespace '$namespace': expected [A-Za-z][A-Za-z0-9_]*" >&2
    exit 2
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
echo "[HIL] Robot namespace: ${namespace:-<params_file or robot_1>}"
exec ros2 launch robot_launch outdoor_zny.launch.py "${launch_args[@]}"
