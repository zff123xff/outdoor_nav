"""Coordinate validation and ENU-to-WGS84 conversion for task files."""

import math


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
    lat0, lon0, alt0 = origin
    radius = math.hypot(east, north)
    angular = radius / 6378137.0
    if angular > math.pi:
        raise ValueError('ENU 平面距离必须小于半个地球周长')
    if radius < 1e-9:
        return lat0, lon0, alt0 + up
    lat0_r = math.radians(lat0)
    lon0_r = math.radians(lon0)
    sin_c = math.sin(angular)
    cos_c = math.cos(angular)
    lat_r = math.asin(cos_c * math.sin(lat0_r) + north * sin_c * math.cos(lat0_r) / radius)
    lon_r = lon0_r + math.atan2(
        east * sin_c,
        radius * math.cos(lat0_r) * cos_c - north * math.sin(lat0_r) * sin_c,
    )
    return math.degrees(lat_r), math.degrees(lon_r), alt0 + up


def task_coordinate_frame(data):
    frame = data.get('coordinate_frame')
    if frame not in ('gps', 'enu'):
        raise ValueError('coordinate_frame 必须明确设置为 gps 或 enu')
    return frame
