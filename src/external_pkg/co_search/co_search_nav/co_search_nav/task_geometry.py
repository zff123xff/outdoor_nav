"""Area-search executor configuration validation and rectangle geometry."""
from dataclasses import dataclass
import json
import math
import re


def number(value, name, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    value = float(value)
    if minimum is not None and value < minimum:
        raise ValueError(f'{name} must be >= {minimum}')
    return value


def validate_task(data):
    if not isinstance(data, dict):
        raise ValueError('area search YAML must be a mapping')
    result = json.loads(json.dumps(data, allow_nan=False))
    ns = str(result.get('robot_namespace', '')).strip('/')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', ns):
        raise ValueError('robot_namespace must be a single non-empty ROS namespace')
    result['robot_namespace'] = ns
    if result.get('search_controller', 'co_search') != 'co_search':
        raise ValueError('area_search only supports search_controller: co_search')
    result['search_controller'] = 'co_search'
    mode = result.get('coordinate_mode', 'enu')
    if mode not in ('enu', 'gps'):
        raise ValueError('coordinate_mode must be gps or enu')
    result['coordinate_mode'] = mode
    topics = result.get('topics', {})
    if not isinstance(topics, dict) or any(not isinstance(v, str) or not v for v in topics.values()):
        raise ValueError('topics must be non-empty ROS topic names')
    rect = result.setdefault('rectangle', {})
    for name in ('length_m', 'width_m'):
        rect[name] = number(rect.get(name), 'rectangle.'+name, 1.)
    rect['yaw_deg'] = number(rect.get('yaw_deg', 0.), 'rectangle.yaw_deg')
    point = result.get('center_enu' if mode == 'enu' else 'center_gps', {})
    for name in (('east', 'north') if mode == 'enu' else ('latitude', 'longitude')):
        number(point.get(name), 'center.'+name)
    if mode == 'gps':
        number(point.get('altitude', 0.), 'center_gps.altitude')
        if abs(point['latitude']) > 90 or abs(point['longitude']) > 180:
            raise ValueError('GPS latitude/longitude out of range')
    options = result.setdefault('options', {})
    timeout = number(options.get('sensor_timeout_s', 3.), 'options.sensor_timeout_s', 0.)
    if timeout <= 0:
        raise ValueError('options.sensor_timeout_s must be positive')
    options['sensor_timeout_s'] = timeout
    result['mission_id'] = str(result.get('mission_id', 'rectangle_search'))
    if not result['mission_id']:
        raise ValueError('mission_id must not be empty')
    return result


def gps_to_enu(lat, lon, alt, origin):
    """Same spherical azimuthal-equidistant conversion as nav2_input_bridge."""
    lat, lon, lat0, lon0 = map(math.radians, (lat, lon, origin[0], origin[1]))
    d = lon - lon0
    arg = max(-1.0, min(1.0, math.sin(lat0) * math.sin(lat) + math.cos(lat0) * math.cos(lat) * math.cos(d)))
    c = math.acos(arg)
    k = c / math.sin(c) if abs(c) > 1e-6 else 1.0
    return (6378137.0 * k * math.cos(lat) * math.sin(d),
            6378137.0 * k * (math.cos(lat0) * math.sin(lat) - math.sin(lat0) * math.cos(lat) * math.cos(d)),
            alt - origin[2])


@dataclass(frozen=True)
class Rectangle:
    east: float
    north: float
    length: float
    width: float
    yaw: float = 0.0

    def world(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.east + c*x - s*y, self.north + s*x + c*y

    def local(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x-self.east, y-self.north
        return c*dx+s*dy, -s*dx+c*dy

    def contains(self, x, y, margin=0):
        u, v = self.local(x, y)
        return abs(u) <= self.length/2-margin and abs(v) <= self.width/2-margin

    def closest_point(self, x, y, margin=0.0):
        """Orthogonal projection, including rotated edges and corners."""
        u, v = self.local(x, y)
        half_x = max(0.0, self.length/2-margin)
        half_y = max(0.0, self.width/2-margin)
        return self.world(max(-half_x, min(half_x, u)),
                          max(-half_y, min(half_y, v)))

    def corners(self):
        return [self.world(x, y) for x, y in [(-self.length/2, -self.width/2),
                (self.length/2, -self.width/2), (self.length/2, self.width/2), (-self.length/2, self.width/2)]]


def rectangle_for_task(task, origin):
    if task['coordinate_mode'] == 'gps':
        p = task['center_gps']
        x, y, _ = gps_to_enu(p['latitude'], p['longitude'], p.get('altitude', origin[2]), origin)
    else:
        x, y = task['center_enu']['east'], task['center_enu']['north']
    r = task['rectangle']
    return Rectangle(x, y, r['length_m'], r['width_m'], math.radians(r['yaw_deg']))


