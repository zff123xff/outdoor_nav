workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace/install/local_setup.bash"

ros2 launch fast_lio mapping.launch.py
