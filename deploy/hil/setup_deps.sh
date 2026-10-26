#!/usr/bin/env bash
# Prepare native ROS runtime packages locally; no sudo or robot nodes.
set -eo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws="$(cd "$here/../.." && pwd)"
if [[ -r "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash" ]]; then
    source "${CODEX_HOME:-$HOME/.codex}/ros/setup.bash"
else
    source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
fi
cache="$ws/hil_deps/debs"
root="$ws/hil_deps/ros"
mkdir -p "$cache" "$root"
cd "$cache"
package="ros-${ROS_DISTRO:-humble}-twist-mux"
arch="$(dpkg --print-architecture)"
apt-get download "$package:$arch"
for archive in "$cache"/"${package}"_*_"${arch}".deb; do
    [[ -f "$archive" ]] || continue
    dpkg-deb -x "$archive" "$root"
done
printf '%s\n' "$arch" > "$root/.architecture"
[[ -r "$root/opt/ros/${ROS_DISTRO:-humble}/share/ament_index/resource_index/packages/twist_mux" ]] || {
    echo '[HIL] twist_mux extraction failed.' >&2; exit 1;
}
echo "[HIL] Native $arch twist_mux prepared in $root."
