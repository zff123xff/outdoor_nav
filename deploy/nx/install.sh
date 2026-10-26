#!/usr/bin/env bash
# Run on the NX itself, as its normal login user. No robot nodes are started.
set -eo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws="$(cd "$here/../.." && pwd)"
third="$ws/nx_deps"
mode="${1:-check}"
case "$mode" in check|install|build|all) ;; *) echo 'Usage: bash deploy/nx/install.sh [check|install|build|all]'; exit 2;; esac
source /etc/os-release
printf 'OS: %s\nArchitecture: %s\nWorkspace: %s\n' "$PRETTY_NAME" "$(uname -m)" "$ws"
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || { echo 'Requires native Ubuntu 22.04 (ROS Humble).'; exit 1; }
[[ "$(uname -m)" == aarch64 ]] || { echo 'Run this installer on ARM64 NX, not the x86 development PC.'; exit 1; }
[[ "$EUID" != 0 ]] || { echo 'Run as the normal user; individual installation steps use sudo.'; exit 1; }
if [[ "$mode" == check ]]; then
    df -h "$ws"
    free -h
    ip -brief address
    ls -l /dev/serial/by-id/ 2>/dev/null || true
    [[ ! -f /etc/nv_tegra_release ]] || cat /etc/nv_tegra_release
    echo 'Check complete; nothing installed. Use all for installation and native build.'
    exit 0
fi
export CMAKE_BUILD_PARALLEL_LEVEL="${NX_BUILD_JOBS:-2}"
export MAKEFLAGS="-j${NX_BUILD_JOBS:-2}"
# The existing backend CMakeLists reads the environment variable explicitly.
export CMAKE_BUILD_TYPE=Release
if [[ "$mode" == install || "$mode" == all ]]; then
    sudo apt-get update
    sudo apt-get install -y ca-certificates curl gnupg git build-essential cmake pkg-config locales software-properties-common
    sudo locale-gen en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo add-apt-repository -y universe
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/nx-ros-archive-keyring.gpg
    sudo install -m 644 /tmp/nx-ros-archive-keyring.gpg /usr/share/keyrings/ros-archive-keyring.gpg
    printf 'deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu jammy main\n' | sudo tee /etc/apt/sources.list.d/nx-ros2.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y ros-humble-ros-base ros-dev-tools python3-colcon-common-extensions python3-rosdep \
      ros-humble-navigation2 ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl \
      ros-humble-mavros ros-humble-mavros-extras ros-humble-twist-mux ros-humble-foxglove-bridge \
      ros-humble-rviz2 ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-cv-bridge \
      ros-humble-ament-cmake-auto libpcl-dev libapr1-dev libeigen3-dev libboost-all-dev \
      libyaml-cpp-dev nlohmann-json3-dev libjsoncpp-dev libssl-dev libpaho-mqtt-dev libpaho-mqttpp-dev \
      python3-pip python3-yaml python3-numpy python3-shapely python3-paho-mqtt geographiclib-tools
    python3 -m pip install --user 'pymap3d==3.1.0'
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
    rosdep update --rosdistro humble
    sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
    sudo usermod -aG dialout "$USER"
    [[ -f "$here/config.env" ]] || cp "$here/config.env.example" "$here/config.env"
    echo 'Dependencies installed. Log out/in to apply dialout group membership.'
fi
if [[ "$mode" == build || "$mode" == all ]]; then
    source /opt/ros/humble/setup.bash
    mkdir -p "$third/livox_ws/src"
    clone() {
        if [[ ! -d "$2/.git" ]]; then git clone --recursive "$1" "$2"; fi
    }
    clone https://github.com/unitreerobotics/unitree_ros2.git "$third/unitree_ros2"
    clone https://github.com/Livox-SDK/Livox-SDK2.git "$third/Livox-SDK2"
    clone https://github.com/Livox-SDK/livox_ros_driver2.git "$third/livox_ws/src/livox_ros_driver2"
    cmake -S "$third/Livox-SDK2" -B "$third/Livox-SDK2/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$third/Livox-SDK2/build" --parallel "${NX_BUILD_JOBS:-2}"
    sudo cmake --install "$third/Livox-SDK2/build"
    sudo ldconfig
    cd "$third/unitree_ros2/cyclonedds_ws"
    colcon build --packages-select unitree_go unitree_api --executor sequential --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    source install/local_setup.bash
    cd "$third/livox_ws"
    # Prepare ROS2 manifest without upstream build.sh deleting its workspace build/install folders.
    cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
    rosdep install --from-paths src --ignore-src --rosdistro humble -y --dependency-types build buildtool exec
    colcon build --executor sequential --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    source install/local_setup.bash
    cd "$ws"
    rosdep install --from-paths src --ignore-src --rosdistro humble -y --dependency-types build buildtool exec
    colcon build --base-paths src --build-base build_nx --install-base install_nx \
      --executor sequential --symlink-install \
      --packages-up-to robot_launch go2w_driver remote_override mavros_odom_relay \
        execute_task_server multi_goal_to_gp_goal multi_task_bridge backend_nodes \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    mkdir -p "$here/reports"
    for repository in "$third/unitree_ros2" "$third/Livox-SDK2" "$third/livox_ws/src/livox_ros_driver2"; do
        printf '%s ' "$repository"
        git -C "$repository" rev-parse HEAD
    done > "$here/reports/dependency_commits.txt"
    source install_nx/local_setup.bash
    python3 -c 'import rclpy, pymap3d, multi_task_bridge.node; from unitree_go.msg import SportModeState; from unitree_api.msg import Request; from execute_task_server.srv import ExecuteTask; print("NX imports OK")'
    [[ -f "$here/config.env" ]] || cp "$here/config.env.example" "$here/config.env"
    echo "Build complete. Edit $here/config.env, hardware nav YAML and MID360 JSON before startup."
fi
