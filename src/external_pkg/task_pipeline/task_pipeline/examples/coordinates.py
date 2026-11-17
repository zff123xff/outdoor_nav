"""Coordinate validation and ENU-to-WGS84 conversion for task files."""

import math
from multi_goal_to_gp_goal.coordinates import enu_to_lla


def finite_number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{field} 必须是数值')
    if not math.isfinite(value):
        raise ValueError(f'{field} 必须是有限数值')
    return float(value)


def enu_to_gps(east, north, up, origin):
    """Invert the local azimuthal-equidistant projection used by nav2 bridge."""
    east = finite_number(east, 'x/east')
    north = finite_number(north, 'y/north')
    up = finite_number(up, 'z/up')
    return enu_to_lla(east, north, up, *origin)

