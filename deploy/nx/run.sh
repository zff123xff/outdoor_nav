#!/usr/bin/env bash
set -eo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-check}"
[[ $# == 0 ]] || shift
if [[ "$mode" == hil ]]; then
 # Simulation has separate settings: no dog interface, FCU or Unitree DDS check.
 ws="$(cd "$here/../.." && pwd)"
 hil_domain="${ROS_DOMAIN_ID-}"
 hil_rmw="${RMW_IMPLEMENTATION-}"
 hil_cyclone="${CYCLONEDDS_URI-}"
 hil_fast="${FASTRTPS_DEFAULT_PROFILES_FILE-}"
 hil_fastdds="${FASTDDS_DEFAULT_PROFILES_FILE-}"
 if [[ -r "$here/hil.env" ]]; then
  set -a
  source "$here/hil.env"
  set +a
 fi
 [[ -z "$hil_domain" ]] || export ROS_DOMAIN_ID="$hil_domain"
 [[ -z "$hil_rmw" ]] || export RMW_IMPLEMENTATION="$hil_rmw"
 [[ -z "$hil_cyclone" ]] || export CYCLONEDDS_URI="$hil_cyclone"
 [[ -z "$hil_fast" ]] || export FASTRTPS_DEFAULT_PROFILES_FILE="$hil_fast"
 [[ -z "$hil_fastdds" ]] || export FASTDDS_DEFAULT_PROFILES_FILE="$hil_fastdds"
 export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
 export OUTDOOR_NAV_WS="$ws"
 if [[ -z "${HIL_INSTALL_DIR:-}" ]]; then
  if [[ -r "$ws/install_nx/setup.bash" ]]; then
   export HIL_INSTALL_DIR="$ws/install_nx"
  else
   export HIL_INSTALL_DIR="$ws/install"
  fi
 fi
 export HIL_INSTALL_DIR
 # Omit an unset namespace so the launch file can use params_file's default.
 # ROS launch rejects a command-line argument with an empty value.
 hil_args=()
 if [[ -n "${HIL_ROBOT_NAMESPACE:-}" ]]; then
  hil_args+=("robot_namespace:=$HIL_ROBOT_NAMESPACE")
 fi
 exec bash "$ws/launch_hil.sh" "${hil_args[@]}" "$@"
fi
case "$mode" in check|nav|tasks) source "$here/env.sh";;
 *) echo 'Usage: bash deploy/nx/run.sh [check|nav|tasks|hil] [key:=value ...]'; exit 2;;
esac
case "$mode" in
 check)
  ros2 topic list --no-daemon --spin-time 5
  timeout 12 ros2 topic echo /lf/sportmodestate unitree_go/msg/SportModeState --once --qos-reliability best_effort
  ;;
 nav)
  exec ros2 launch robot_launch outdoor_all_go2w.launch.py \
    robot_namespace:="$ROBOT_NAMESPACE" fcu_url:="$FCU_URL" use_foxglove:=false "$@"
  ;;
 tasks)
  # Direct original launches avoid setup_go2w.sh's old ~/ww hardcoded path.
  exec ros2 launch robot_launch task_pipeline.launch.py \
    robot_namespace:="$ROBOT_NAMESPACE" usv_sn:="$ROBOT_NAMESPACE" prefix:='' \
    drone_name:="$ROBOT_NAMESPACE" mqtt:=false "$@"
  ;;
esac
