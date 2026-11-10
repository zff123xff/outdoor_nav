import math

WGS84_R = 6378137.0


def enu_to_lla(east, north, up, lat0_deg, lon0_deg, alt0_m):
    """Invert the spherical projection used by nav2_input_bridge."""
    if not all(math.isfinite(v) for v in (east, north, up, lat0_deg, lon0_deg, alt0_m)):
        raise ValueError('ENU position and geographic reference must be finite')
    if abs(lat0_deg) > 90 or abs(lon0_deg) > 180:
        raise ValueError('Geographic reference is outside latitude/longitude bounds')
    radius = math.hypot(east, north)
    angular = radius / WGS84_R
    if angular >= math.pi:
        raise ValueError('ENU distance must be less than half the Earth circumference')
    if radius < 1e-9:
        return lat0_deg, lon0_deg, alt0_m + up
    lat0_r = math.radians(lat0_deg)
    lon0_r = math.radians(lon0_deg)
    sin_c, cos_c = math.sin(angular), math.cos(angular)
    sin_lat = cos_c * math.sin(lat0_r) + north * sin_c * math.cos(lat0_r) / radius
    lat_r = math.asin(max(-1.0, min(1.0, sin_lat)))
    lon_r = lon0_r + math.atan2(
        east * sin_c, radius * math.cos(lat0_r) * cos_c - north * math.sin(lat0_r) * sin_c)
    return math.degrees(lat_r), (math.degrees(lon_r) + 180.) % 360. - 180., alt0_m + up


def lla_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    lat0_deg: float,
    lon0_deg: float,
    alt0_m: float,
) -> tuple[float, float, float]:
    """Spherical tangent-plane projection (matches nav2_input_bridge)."""
    lat_r = math.radians(lat_deg)
    lon_r = math.radians(lon_deg)
    lat0_r = math.radians(lat0_deg)
    lon0_deg_r = math.radians(lon0_deg)
    sin_lat = math.sin(lat_r)
    cos_lat = math.cos(lat_r)
    sin_lat0 = math.sin(lat0_r)
    cos_lat0 = math.cos(lat0_r)
    dlon = lon_r - lon0_deg_r
    arg = sin_lat0 * sin_lat + cos_lat0 * cos_lat * math.cos(dlon)
    arg = max(-1.0, min(1.0, arg))
    c = math.acos(arg)
    k = c / math.sin(c) if abs(c) > 1e-6 else 1.0
    north = k * (cos_lat0 * sin_lat - sin_lat0 * cos_lat * math.cos(dlon)) * WGS84_R
    east = k * cos_lat * math.sin(dlon) * WGS84_R
    up = alt_m - alt0_m
    return east, north, up

