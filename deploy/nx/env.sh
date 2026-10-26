#!/usr/bin/env bash
_nx_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export OUTDOOR_NAV_NX_WS="$(cd "$_nx_here/../.." && pwd)"
[[ -f "$_nx_here/config.env" ]] || { echo 'Copy config.env.example to config.env and fill hardware settings.'; return 1; }
# Preserve explicitly inherited DDS/domain/network values.
_nx_domain="${ROS_DOMAIN_ID-}"; _nx_rmw="${RMW_IMPLEMENTATION-}"; _nx_uri="${CYCLONEDDS_URI-}"
source "$_nx_here/config.env" || return 1
[[ -z "$_nx_domain" ]] || ROS_DOMAIN_ID="$_nx_domain"
[[ -n "${GO2W_NET_IFACE:-}" && -d "/sys/class/net/$GO2W_NET_IFACE" ]] || { echo 'Set GO2W_NET_IFACE to the real dog Ethernet interface in config.env.'; return 1; }
for _nx_setup in /opt/ros/humble/setup.bash \
 "$OUTDOOR_NAV_NX_WS/nx_deps/unitree_ros2/cyclonedds_ws/install/local_setup.bash" \
 "$OUTDOOR_NAV_NX_WS/nx_deps/livox_ws/install/local_setup.bash" \
 "$OUTDOOR_NAV_NX_WS/install_nx/local_setup.bash"; do
 [[ -r "$_nx_setup" ]] || { echo "Missing setup: $_nx_setup"; return 1; }
 source "$_nx_setup" || return 1
done
export RMW_IMPLEMENTATION="${_nx_rmw:-rmw_cyclonedds_cpp}"
[[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ]] || { echo 'Real Unitree driver requires Cyclone DDS; use a separate shell with the intended DDS configuration.'; return 1; }
if [[ -n "$_nx_uri" ]]; then
 export CYCLONEDDS_URI="$_nx_uri"
else
 _nx_interfaces="<NetworkInterface name=\"$GO2W_NET_IFACE\" priority=\"default\" multicast=\"default\"/>"
 if [[ -n "${GO2W_LAN_IFACE:-}" && "$GO2W_LAN_IFACE" != "$GO2W_NET_IFACE" ]]; then
  [[ -d "/sys/class/net/$GO2W_LAN_IFACE" ]] || { echo 'GO2W_LAN_IFACE does not exist'; return 1; }
  _nx_interfaces+="<NetworkInterface name=\"$GO2W_LAN_IFACE\" priority=\"default\" multicast=\"default\"/>"
 fi
 export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>$_nx_interfaces</Interfaces></General></Domain></CycloneDDS>"
fi
export ROS_DOMAIN_ID ROBOT_NAMESPACE GO2W_NET_IFACE GO2W_LAN_IFACE FCU_URL
unset _nx_here _nx_domain _nx_rmw _nx_uri _nx_setup _nx_interfaces
