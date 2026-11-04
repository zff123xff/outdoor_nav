"""Validation and JSON representation of the public GPS-only task topic."""
import math
from yolo_detector.targets import validate_targets

TASK_TYPES = ('point_goal', 'multi_point', 'patrol_line', 'area_search')


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def gps(point):
    if not isinstance(point, dict):
        raise ValueError('GPS point must be an object')
    output = {k: finite(point.get(k, 0.0) if k == 'altitude' else point.get(k), k)
              for k in ('latitude', 'longitude', 'altitude')}
    if abs(output['latitude']) > 90 or abs(output['longitude']) > 180:
        raise ValueError('GPS coordinates out of range')
    return output


def validate(data):
    if not isinstance(data, dict):
        raise ValueError('task must be an object')
    cancel = data.get('cancel', False)
    if not isinstance(cancel, bool):
        raise ValueError('cancel must be a boolean')
    if not isinstance(data.get('task_id'), str) or not data['task_id']:
        raise ValueError('task_id must be a non-empty string')
    if cancel:
        return dict(task_id=data['task_id'], cancel=True, task_type='', targets=[], waypoints=[], area={})
    for key in ('task_type',):
        if not isinstance(data.get(key), str) or not data[key]:
            raise ValueError(f'{key} must be a non-empty string')
    kind = data['task_type']
    if kind not in TASK_TYPES:
        raise ValueError(f'unsupported task_type: {kind}')
    points = data.get('waypoints', [])
    if not isinstance(points, list):
        raise ValueError('waypoints must be an array')
    if kind in ('point_goal', 'multi_point') and not points:
        raise ValueError(f'{kind} needs at least one waypoint')
    if kind == 'patrol_line' and len(points) != 2:
        raise ValueError('patrol_line needs exactly two waypoints')
    output = dict(task_id=data['task_id'], task_type=kind, cancel=False, targets=validate_targets(data.get('targets', [])),
                  waypoints=[gps(p) for p in points], area={})
    if kind == 'area_search':
        area = data.get('area')
        if not isinstance(area, dict):
            raise ValueError('area_search needs area')
        output['area'] = dict(center=gps(area.get('center')))
        for key in ('length_m', 'width_m', 'yaw_deg'):
            value = finite(area.get(key, 0.0) if key == 'yaw_deg' else area.get(key), key)
            if key != 'yaw_deg' and value < 1.0:
                raise ValueError(f'{key} must be >= 1 meter')
            output['area'][key] = value
    return output


def to_message(data):
    from sensor_msgs.msg import NavSatFix
    from task_interfaces.msg import Task
    data = validate(data)
    task = Task(task_id=data['task_id'], task_type=data['task_type'], cancel=data['cancel'], targets=data['targets'])
    task.waypoints = [NavSatFix(**p) for p in data['waypoints']]
    if data['area']:
        task.area.center = NavSatFix(**data['area']['center'])
        for key in ('length_m', 'width_m', 'yaw_deg'):
            setattr(task.area, key, data['area'][key])
    return task


def from_message(task):
    def point(p):
        return dict(latitude=p.latitude, longitude=p.longitude, altitude=p.altitude)
    return validate(dict(task_id=task.task_id, task_type=task.task_type, cancel=task.cancel, targets=list(task.targets),
        waypoints=[point(p) for p in task.waypoints],
        area=dict(center=point(task.area.center), length_m=task.area.length_m,
                  width_m=task.area.width_m, yaw_deg=task.area.yaw_deg)))


def distance(a, b):
    """Great-circle horizontal distance, in meters."""
    lat1, lat2 = math.radians(a.latitude), math.radians(b.latitude)
    dlat = lat2-lat1
    dlon = math.radians(b.longitude-a.longitude)
    h = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 6378137.0 * 2 * math.asin(math.sqrt(min(1.0, max(0.0, h))))
