"""Shared validation for the UE GPS coordinate corresponding to ENU (0,0,0)."""
import math

ORIGIN_PARAMETERS = ('reference_latitude', 'reference_longitude', 'reference_altitude')


def validate_origin(latitude, longitude, altitude):
    values = (latitude, longitude, altitude)
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           for v in values):
        raise ValueError('GPS origin latitude/longitude/altitude must be finite numbers')
    if abs(latitude) > 90. or abs(longitude) > 180.:
        raise ValueError('GPS origin latitude/longitude out of range')
    return tuple(float(v) for v in values)
