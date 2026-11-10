"""The flat public Task protocol; internal navigation actions retain NavSatFix."""
import math
import struct

TASK_IDS = {'point_goal': 140, 'patrol_line': 141, 'area_search': 142,
            'target_inspection': 143, 'multi_point': 145}
TASK_KINDS = {value: key for key, value in TASK_IDS.items()}
TASK_TYPES = tuple(TASK_IDS)  # Internal names only, never a wire field.


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def gps(point):
    if not isinstance(point, dict):
        raise ValueError('GPS point must be an object')
    out = {key: finite(point.get(key, 0.) if key == 'altitude' else point.get(key), key)
           for key in ('latitude', 'longitude', 'altitude')}
    if abs(out['latitude']) > 90 or abs(out['longitude']) > 180:
        raise ValueError('GPS coordinates out of range')
    return out


def coordinates(values, name):
    if not isinstance(values, (list, tuple)) or len(values) % 3:
        raise ValueError(f'{name} must contain latitude/longitude/altitude triplets')
    result = []
    for value in values:
        try:
            value = struct.unpack('f', struct.pack('f', finite(value, name)))[0]
        except (OverflowError, struct.error) as exc:
            raise ValueError(f'{name} exceeds float32 range') from exc
        result.append(finite(value, name))
    for i in range(0, len(result), 3):
        gps(dict(zip(('latitude', 'longitude', 'altitude'), result[i:i+3])))
    return result


def validate(data):
    if not isinstance(data, dict):
        raise ValueError('task must be an object')
    task_id, cancel = data.get('task_id'), data.get('cancel', False)
    if isinstance(task_id, bool) or not isinstance(task_id, int) or not 0 <= task_id <= 65535:
        raise ValueError('task_id must be a uint16 integer (0..65535)')
    if task_id not in TASK_KINDS:
        raise ValueError('task_id must be one of 140, 141, 142, 143, 145 (integer)')
    if not isinstance(cancel, bool):
        raise ValueError('cancel must be a boolean')
    out = dict(task_id=task_id, cancel=cancel, targets=[], waypoints=[], center=[],
               length_m=0., width_m=0., yaw_deg=0., gait_speed=0, air_route=0)
    if cancel:
        return out
    unknown = set(data) - set(out)
    if unknown:
        raise ValueError('unsupported task fields: ' + ', '.join(sorted(unknown)))
    for key in ('gait_speed', 'air_route'):
        value = data.get(key, 0)
        if isinstance(value, bool) or not isinstance(value, int) or not -32768 <= value <= 32767:
            raise ValueError(f'{key} must be an int16 (-32768..32767)')
        out[key] = value
    targets = data.get('targets', [])
    if not isinstance(targets, list) or any(not isinstance(t, str) or not t.strip() for t in targets):
        raise ValueError('targets must be an array of non-empty strings')
    out['targets'] = list(dict.fromkeys(targets))
    out['waypoints'] = coordinates(data.get('waypoints', []), 'waypoints')
    out['center'] = coordinates(data.get('center', []), 'center')
    if len(out['center']) not in (0, 3):
        raise ValueError('center must be empty or one GPS triplet')
    for key in ('length_m', 'width_m', 'yaw_deg'):
        out[key] = finite(data.get(key, 0.), key)
    kind = TASK_KINDS[task_id]
    count = len(out['waypoints']) // 3
    if kind == 'point_goal' and count != 1:
        raise ValueError('140 needs exactly one GPS waypoint')
    if kind == 'patrol_line' and count != 2:
        raise ValueError('141 needs exactly two GPS waypoints')
    if kind == 'multi_point' and count < 1:
        raise ValueError('145 needs at least one GPS waypoint')
    if kind == 'area_search':
        if len(out['center']) != 3 or min(out['length_m'], out['width_m']) < 1.:
            raise ValueError('142 needs a center triplet and dimensions >= 1 metre')
    return out


def to_message(data):
    from task_interfaces.msg import Task
    return Task(**validate(data))


def from_message(task):
    return validate({key: list(getattr(task, key)) if key in ('targets', 'waypoints', 'center')
                     else getattr(task, key) for key in
                     ('task_id', 'cancel', 'targets', 'waypoints', 'center',
                      'length_m', 'width_m', 'yaw_deg', 'gait_speed', 'air_route')})


def nav_points(values):
    from sensor_msgs.msg import NavSatFix
    return [NavSatFix(latitude=float(values[i]), longitude=float(values[i+1]),
                      altitude=float(values[i+2])) for i in range(0, len(values), 3)]


def distance(a, b):
    """Great-circle horizontal distance in metres."""
    lat1, lat2 = math.radians(a.latitude), math.radians(b.latitude)
    dlat, dlon = lat2-lat1, math.radians(b.longitude-a.longitude)
    h = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 6378137.0 * 2 * math.asin(math.sqrt(min(1., max(0., h))))
