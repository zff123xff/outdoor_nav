#!/usr/bin/env bash
# Launch go2w_driver only (no Nav2/PX4). For standalone driver testing.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source "${SCRIPT_DIR}/setup_go2w.sh"
source install/setup.bash

ros2 launch go2w_driver go2w_driver.launch.py "$@"
