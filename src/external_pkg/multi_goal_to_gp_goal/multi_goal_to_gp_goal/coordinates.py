import math

WGS84_R = 6378137.0

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

