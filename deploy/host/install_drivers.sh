#!/usr/bin/env bash
# MAVROS + MID360 installer for an existing Ubuntu 22.04 / ROS 2 Humble host.
# Never starts robot nodes, changes network/DDS settings, or uses sudo.
set -eo pipefail

third="${DRIVER_THIRD_PARTY:-$HOME/ww/3rd}"
jobs="${DRIVER_BUILD_JOBS:-1}"
mode=all
download_source=0
for argument in "$@"; do
case "$argument" in
  all|prepare|build|check) mode="$argument" ;;
  --download-source) download_source=1 ;;
  --no-download-source) download_source=0 ;;
  -h|--help)
    echo 'Usage: bash install_drivers.sh [all|prepare|build|check] [--download-source|--no-download-source]'
    echo 'Default: use existing local sources; --download-source permits cloning missing sources.'
    echo 'DRIVER_THIRD_PARTY: installation root (default: $HOME/ww/3rd)'
    echo 'DRIVER_BUILD_JOBS: compiler jobs (default: 1)'
    echo 'Requires Ubuntu 22.04, ROS 2 Humble and the base ROS/PCL development dependencies.'
    exit 0 ;;
  *) echo "Unknown argument: $argument" >&2; exit 2 ;;
esac
done
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'DRIVER_BUILD_JOBS must be positive' >&2; exit 2; }
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || {
  echo 'Requires Ubuntu 22.04 (x86_64 or ARM64 NX) with ROS 2 Humble.' >&2; exit 1;
}
case "$(uname -m)" in x86_64|aarch64) ;; *) echo 'Unsupported host architecture' >&2; exit 1;; esac
[[ -r /opt/ros/humble/setup.bash ]] || { echo 'Install ROS 2 Humble first' >&2; exit 1; }
if [[ -r "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash" ]]; then
  source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
else
  source /opt/ros/humble/setup.bash
fi
[[ "$ROS_DISTRO" == humble ]] || { echo 'Use a ROS 2 Humble shell' >&2; exit 1; }
export CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
export MAKEFLAGS="-j$jobs"

clone_pinned() {
  local url="$1" destination="$2" revision="$3" required="${4:-CMakeLists.txt}"
  if [[ -f "$destination/$required" ]]; then
    if [[ -e "$destination/.git" ]]; then
      [[ "$(git -C "$destination" rev-parse HEAD)" == "$revision" ]] || {
        echo "Unexpected source revision in $destination; no files were reset." >&2
        echo "Expected $revision. Review the checkout before continuing." >&2
        return 1
      }
    else
      echo "Using local source snapshot without Git metadata: $destination"
    fi
  else
    if [[ "$download_source" != 1 ]]; then
      echo "Missing local source: $destination" >&2
      echo 'Upload the source first or pass --download-source.' >&2
      return 1
    fi
    [[ ! -e "$destination" ]] || {
      echo "Refusing to replace existing non-Git path: $destination" >&2; return 1;
    }
    git clone --recursive "$url" "$destination"
    git -C "$destination" checkout --detach "$revision"
    git -C "$destination" submodule update --init --recursive
  fi
}

prepare() {
  mkdir -p "$third/mavros_ws/src" "$third/livox_ws/src"
  clone_pinned https://github.com/mavlink/mavros.git "$third/mavros_ws/src/mavros" e4f39208aa872fb887de33520f158b1612bcf8f0 mavros/package.xml
  clone_pinned https://github.com/mavlink/mavlink.git "$third/mavros_ws/src/mavlink" 45ec1a19169e3be7d3859ef5fb69e2d68feebbdc
  clone_pinned https://github.com/Livox-SDK/Livox-SDK2.git "$third/Livox-SDK2" 08f523c930b2f0ba1e98a6afaa8d7476bf479908
  clone_pinned https://github.com/Livox-SDK/livox_ros_driver2.git "$third/livox_ws/src/livox_ros_driver2" 21445540f0d100dc86a7e6df312dd70bbdb4afdf
  for required in "$third/mavros_ws/src/mavlink/pymavlink/generator/mavgen.py" \
    "$third/mavros_ws/src/mavlink/message_definitions/v1.0/common.xml"; do
    [[ -f "$required" ]] || { echo "Incomplete MAVLink source: $required" >&2; return 1; }
  done
  cp "$third/livox_ws/src/livox_ros_driver2/package_ROS2.xml" "$third/livox_ws/src/livox_ros_driver2/package.xml"

  # Keep missing APR build/runtime dependencies inside the requested root.
  if ! pkg-config --exists apr-1 apr-util-1; then
    mkdir -p "$third/driver_deps/debs" "$third/driver_deps/root"
    (
      cd "$third/driver_deps/debs"
      apt-get download libapr1 libaprutil1 libsctp1 libapr1-dev libaprutil1-dev libsctp-dev libldap2-dev
      for archive in ./*.deb; do dpkg-deb -x "$archive" "$third/driver_deps/root"; done
    )
    python3 - "$third/driver_deps/root" <<'PY'
from pathlib import Path
import sys
root = Path(sys.argv[1]).resolve()
for pc in root.glob('usr/lib/*/pkgconfig/*.pc'):
    data = pc.read_text()
    for key in ('prefix', 'includedir', 'libdir'):
        data = data.replace(f'{key}=/usr', f'{key}={root}/usr')
    pc.write_text(data)
PY
  fi
  mkdir -p "$third/GeographicLib"
  if [[ ! -s "$third/GeographicLib/geoids/egm96-5.pgm" ]]; then
    geographiclib-get-geoids -p "$third/GeographicLib" egm96-5
  fi
}

build() {
  local architecture
  architecture="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
  export PKG_CONFIG_PATH="$third/driver_deps/root/usr/lib/$architecture/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
  export LIBRARY_PATH="$third/driver_deps/root/usr/lib/$architecture${LIBRARY_PATH:+:$LIBRARY_PATH}"
  export LD_LIBRARY_PATH="$third/Livox-SDK2/install/lib:$third/driver_deps/root/usr/lib/$architecture${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export GEOGRAPHICLIB_DATA="$third/GeographicLib"

  cmake -S "$third/Livox-SDK2" -B "$third/Livox-SDK2/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$third/Livox-SDK2/install"
  cmake --build "$third/Livox-SDK2/build" --target livox_lidar_sdk_shared livox_lidar_sdk_static --parallel "$jobs"
  cmake --install "$third/Livox-SDK2/build/sdk_core"
  (
    cd "$third/mavros_ws"
    colcon build --base-paths src --packages-up-to mavros mavros_extras \
      --executor sequential --symlink-install --cmake-args \
      -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
      -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      -DGEOGRAPHICLIB_GEOID_PATH="$third/GeographicLib"
  )
  (
    cd "$third/livox_ws"
    colcon build --base-paths src --executor sequential --symlink-install --cmake-args \
      -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$third/Livox-SDK2/install/include" \
      -DLIVOX_LIDAR_SDK_LIBRARY="$third/Livox-SDK2/install/lib/liblivox_lidar_sdk_shared.so"
  )
}

write_environment() {
  cat > "$third/setup.bash" <<'BASH'
#!/usr/bin/env bash
# Source only. Does not start nodes or change ROS domain/DDS configuration.
_driver_third="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for _driver_setup in "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" \
  "$_driver_third/mavros_ws/install/local_setup.bash" \
  "$_driver_third/livox_ws/install/local_setup.bash"; do
  if [[ ! -r "$_driver_setup" ]]; then
    echo "Missing driver environment: $_driver_setup" >&2
    unset _driver_setup _driver_third
    return 1
  fi
done
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" || return 1
if [[ -r "$_driver_third/unitree_ros2/cyclonedds_ws/install/local_setup.bash" ]]; then
  source "$_driver_third/unitree_ros2/cyclonedds_ws/install/local_setup.bash" || return 1
fi
source "$_driver_third/mavros_ws/install/local_setup.bash" || return 1
source "$_driver_third/livox_ws/install/local_setup.bash" || return 1
export GEOGRAPHICLIB_DATA="$_driver_third/GeographicLib"
_driver_arch="$(dpkg-architecture -qDEB_HOST_MULTIARCH)"
export LD_LIBRARY_PATH="$_driver_third/Livox-SDK2/install/lib:$_driver_third/driver_deps/root/usr/lib/$_driver_arch${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
unset _driver_setup _driver_third _driver_arch
BASH
  for repository in "$third/mavros_ws/src/mavros" "$third/mavros_ws/src/mavlink" \
    "$third/Livox-SDK2" "$third/livox_ws/src/livox_ros_driver2"; do
    printf '%s ' "$repository"
    if [[ -e "$repository/.git" ]]; then
      git -C "$repository" rev-parse HEAD
    else
      echo 'local source snapshot (no .git)'
    fi
  done > "$third/installation_logs/driver_commits.txt"
}

check() {
  source "$third/setup.bash"
  for package in mavros mavros_extras livox_ros_driver2; do
    ros2 pkg prefix "$package"
    ros2 pkg executables "$package"
  done
  python3 - "$third" <<'PY'
import ctypes
from pathlib import Path
import subprocess
import sys
from rclpy.type_support import check_for_type_support
from mavros_msgs.msg import State
from livox_ros_driver2.msg import CustomMsg
root = Path(sys.argv[1])
for msg in (State, CustomMsg):
    check_for_type_support(msg)
for relative in (
    'mavros_ws/install/mavros/lib/libmavros.so',
    'mavros_ws/install/mavros/lib/libmavros_plugins.so',
    'mavros_ws/install/mavros_extras/lib/libmavros_extras.so',
    'mavros_ws/install/mavros_extras/lib/libmavros_extras_plugins.so',
    'livox_ws/install/livox_ros_driver2/lib/liblivox_ros_driver2.so',
):
    library = root / relative
    result = subprocess.run(['ldd', str(library)], check=True, capture_output=True, text=True)
    if 'not found' in result.stdout:
        raise RuntimeError(result.stdout)
    ctypes.CDLL(str(library))
    print('Library OK:', library)
subprocess.run(['GeoidEval', '--input-string', '40.9445353 113.172819'], check=True)
print('Driver installation check passed; no hardware nodes started.')
PY
}

if [[ "$mode" == check ]]; then
  check
  exit 0
fi
mkdir -p "$third/installation_logs"
exec > >(tee -a "$third/installation_logs/install_$(date +%Y%m%d_%H%M%S).log") 2>&1
echo "Driver root: $third; mode: $mode; compiler jobs: $jobs; download source: $download_source"
if [[ "$mode" == all || "$mode" == prepare ]]; then prepare; fi
if [[ "$mode" == all || "$mode" == build ]]; then
  build
  write_environment
  check
fi
