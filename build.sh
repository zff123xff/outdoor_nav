#!/usr/bin/env bash
set -euo pipefail

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source Unitree workspace for unitree_go package (required by go2w_driver)
# colcon 生成的 setup.bash 会引用可能未设置的变量 (如 COLCON_TRACE)，
# 与 set -u (nounset) 冲突，所以这里临时禁用 nounset
# set +u
# source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/nx_deps/unitree_ros2/cyclonedds_ws/install/setup.bash"
# set -u

colcon build --symlink-install --packages-skip fast_lio go2w_driver --cmake-args \
  -DROS_EDITION=ROS2 -DDISTRO_ROS="${ROS_DISTRO:-humble}" \
  -DBUILD_TESTING=OFF "$@"
