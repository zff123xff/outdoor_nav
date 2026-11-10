"""Publish YAML examples through the public Task topic (GPS or converted ENU)."""
import argparse
import json
import math
from pathlib import Path
import sys

import yaml
from .coordinates import enu_to_gps
from task_pipeline.task_schema import validate, TASK_IDS


def load_example(path, kind, namespace=None, origin=None):
    data = yaml.safe_load(Path(path).read_text())
    ns = str(namespace or data.get('robot_namespace', 'robot_1')).strip('/')
    task = data.get('task', data)
    if 'params' not in task and 'task_id' in task:
        return ns, validate(task)
    params = task.get('params', task)
    task_id = TASK_IDS[kind]
    points = []
    frame = data.get('coordinate_frame', data.get('coordinate_mode', 'gps'))
    def convert(point):
        if frame == 'enu':
            if origin is None:
                raise ValueError('ENU example needs the live navigation origin; use GPS for offline preview')
            lat, lon, alt = enu_to_gps(point.get('x', point.get('east', 0.)),
                point.get('y', point.get('north', 0.)), point.get('z', 0.), origin)
        else:
            lat = point.get('latitude', point.get('lat'))
            lon = point.get('longitude', point.get('lon'))
            alt = point.get('altitude', point.get('alt', 0.))
        return dict(latitude=lat, longitude=lon, altitude=alt)
    area = {}
    if kind == 'area_search' or (kind == 'target_inspection' and 'rectangle' in params):
        area = dict(center=convert(params['center_enu' if frame == 'enu' else 'center_gps']),
                    **params['rectangle'])
    else:
        points = [convert(p.get(frame, p)) for p in params.get('waypoints', [])]
    flat = lambda p: [p['latitude'], p['longitude'], p['altitude']]
    wire = dict(task_id=task_id, waypoints=[v for p in points for v in flat(p)],
                targets=task.get('targets', data.get('targets', [])))
    if area:
        wire.update(center=flat(area.pop('center')), **area)
    return ns, validate(wire)


def main_for(kind, default_frame, argv=None):
    from ament_index_python.packages import get_package_share_directory
    parser = argparse.ArgumentParser(description='Publish a primitive to /<namespace>/task')
    parser.add_argument('file', nargs='?', type=Path)
    parser.add_argument('--file', dest='task_file', type=Path)
    parser.add_argument('--coordinate-frame', choices=('enu', 'gps'), default=default_frame,
                        help='Select config/examples/<enu|gps>/<primitive>.yaml; --file overrides this selection')
    parser.add_argument('--namespace')
    parser.add_argument('--wait-timeout', type=float, default=10.)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--cancel', action='store_true')
    args, ros_args = parser.parse_known_args(argv)
    try:
        if not math.isfinite(args.wait_timeout) or args.wait_timeout <= 0:
            raise ValueError('--wait-timeout must be positive and finite')
        path = (args.task_file or args.file or
                Path(get_package_share_directory('task_pipeline'))/'config'/'examples'/
                args.coordinate_frame/(kind+'.yaml'))
        raw = yaml.safe_load(path.read_text())
        ns = str(args.namespace or raw.get('robot_namespace', 'robot_1')).strip('/')
        origin = None
        if raw.get('coordinate_frame', raw.get('coordinate_mode')) == 'enu' and not args.check and not args.cancel:
            from .topic_client import resolve_navigation_origin
            origin = resolve_navigation_origin(ns, args.wait_timeout, ros_args)
        if args.check or args.cancel:
            task = raw.get('task', raw)
            task = dict(task_id=TASK_IDS[kind], cancel=args.cancel)
        else:
            ns, task = load_example(path, kind, args.namespace, origin)
        if args.dry_run:
            print(json.dumps(task, ensure_ascii=False, indent=2))
            return 0
        from .topic_client import submit
        return submit(task, ns, args.wait_timeout, args.check, args.cancel, ros_args)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, yaml.YAMLError) as exc:
        print(f'Invalid task: {exc}', file=sys.stderr)
        return 2


def send_point_goal(argv=None):
    return main_for('point_goal', 'enu', argv)


def send_multi_point(argv=None):
    return main_for('multi_point', 'gps', argv)


def send_patrol(argv=None):
    return main_for('patrol_line', 'gps', argv)


def send_area_search(argv=None):
    return main_for('area_search', 'enu', argv)


def send_target_inspection(argv=None):
    return main_for('target_inspection', 'gps', argv)
