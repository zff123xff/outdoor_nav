#!/usr/bin/env bash
# Diagnose the invoking terminal and active navigation processes, without motion.
set -eo pipefail
nav_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
source "$nav_ws/install/local_setup.bash"
exec python3 "$nav_ws/scripts/dev/check_task_ingress.py" "$@"
