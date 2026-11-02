#!/usr/bin/env bash
# Go2W DDS environment setup for NX
# Sourced by launch_real.sh for the physical Go2-W startup.

set -eo pipefail

if [ -z "${ROS_DISTRO}" ]; then
    source /opt/ros/humble/setup.bash
fi

# Unitree messages, MAVROS, MID360 driver, SDK and geoid data under ~/ww/3rd.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/setup_hardware.sh"

# CycloneDDS configuration
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Check GO2W_NET_IFACE (for dog communication)
if [ -z "${GO2W_NET_IFACE}" ]; then
    echo "[setup_go2w] ERROR: GO2W_NET_IFACE is not set"
    echo "  Please set GO2W_NET_IFACE to your USB network interface (e.g., enx0826ae32db83)"
    echo "  Example: export GO2W_NET_IFACE=enx0826ae32db83"
    return 1 2>/dev/null || exit 1
fi

# Build multi-interface CYCLONEDDS_URI
# Include: GO2W_NET_IFACE (dog) and LAN interface (for remote RViz)
CYCLONEDDS_INTERFACES="<NetworkInterface name=\"${GO2W_NET_IFACE}\" priority=\"default\" multicast=\"default\" />"

# Optional: Add LAN interface for remote RViz access
if [ -n "${GO2W_LAN_IFACE}" ]; then
    CYCLONEDDS_INTERFACES="${CYCLONEDDS_INTERFACES}<NetworkInterface name=\"${GO2W_LAN_IFACE}\" priority=\"default\" multicast=\"default\" />"
    echo "[setup_go2w] Adding LAN interface: ${GO2W_LAN_IFACE}"
fi

export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>${CYCLONEDDS_INTERFACES}</Interfaces></General></Domain></CycloneDDS>"

echo "[setup_go2w] Environment ready"
echo "  GO2W_NET_IFACE=${GO2W_NET_IFACE}"
if [ -n "${GO2W_LAN_IFACE}" ]; then
    echo "  GO2W_LAN_IFACE=${GO2W_LAN_IFACE}"
fi
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
