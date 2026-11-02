#!/usr/bin/env bash
# Load installed hardware dependencies; no nodes are started, DDS is preserved.
_hardware_setup="${OUTDOOR_NAV_THIRD_PARTY:-$HOME/ww/3rd}/setup.bash"
if [[ ! -r "$_hardware_setup" ]]; then
    echo "Hardware driver setup missing: $_hardware_setup" >&2
    unset _hardware_setup
    return 1
fi
# shellcheck disable=SC1090
source "$_hardware_setup"
_hardware_status=$?
unset _hardware_setup
if [[ "$_hardware_status" != 0 ]]; then
    unset _hardware_status
    return 1
fi
unset _hardware_status
