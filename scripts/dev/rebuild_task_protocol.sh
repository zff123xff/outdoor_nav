#!/usr/bin/env bash
# One-time clean rebuild after replacing/removing generated ROS interfaces.
# Preserve prior artifacts for recovery instead of deleting them.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
backup_dir="$(mktemp -d "${TMPDIR:-/tmp}/outdoor_nav_protocol_backup.XXXXXX")"
for relative in build/task_interfaces install/task_interfaces build/yolo_detector install/yolo_detector; do
    if [[ -e "$relative" ]]; then
        mv "$relative" "$backup_dir/${relative//\//_}"
    fi
done
echo "Previous generated artifacts saved at $backup_dir"
source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
bash build.sh --packages-select task_interfaces task_pipeline robot_launch yolo_detector --executor sequential "$@"
