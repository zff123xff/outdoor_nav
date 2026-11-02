#!/usr/bin/env python3
"""Submit a YAML point goal to its Action executor and print feedback."""

import argparse
import json
import math
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from task_pipeline.coordinates import enu_to_gps, finite_number, task_coordinate_frame


def _default_task_file() -> Path:
    return Path(get_package_share_directory('task_pipeline')) / 'config' / 'point_goal.yaml'


def load_task(path: Path, namespace_override=None, service_override=None, nav_origin=None):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('YAML root must be a mapping')
    namespace = str(namespace_override or data.get('robot_namespace', '')).strip('/')
    task = data.get('task')
    if not isinstance(task, dict):
        raise ValueError('task must be a mapping')
    params = task.get('params')
    if not isinstance(params, dict):
        raise ValueError('task.params must be a mapping')
    waypoints = params.get('waypoints')
    if not isinstance(waypoints, list) or len(waypoints) != 1 or not isinstance(waypoints[0], dict):
        raise ValueError('task.params.waypoints must contain exactly one point')

    frame = task_coordinate_frame(data)
    point = waypoints[0].get(frame)
    if not isinstance(point, dict):
        raise ValueError(f'task.params.waypoints[0].{frame} must be a mapping')
    if frame == 'gps':
        lat = finite_number(point.get('lat'), 'lat')
        lon = finite_number(point.get('lon'), 'lon')
        alt = finite_number(point.get('alt', 0.0), 'alt')
        output_point = {'lat': lat, 'lon': lon, 'alt': alt}
    else:
        east = finite_number(point.get('x'), 'x/east')
        north = finite_number(point.get('y'), 'y/north')
        up = finite_number(point.get('z', 0.0), 'z/up')
        if nav_origin is None:
            output_point = {'x': east, 'y': north, 'z': up}
        else:
            lat, lon, alt = enu_to_gps(east, north, up, nav_origin)
            output_point = {'lat': lat, 'lon': lon, 'alt': alt}
    if frame == 'gps':
        if not -90.0 <= output_point['lat'] <= 90.0:
            raise ValueError('lat must be in [-90, 90]')
        if not -180.0 <= output_point['lon'] <= 180.0:
            raise ValueError('lon must be in [-180, 180]')

    service_name = service_override or (
        f'/agent/{namespace}/execute_task' if namespace_override is not None
        else data.get('service_name') or f'/agent/{namespace}/execute_task')
    if not isinstance(service_name, str) or not service_name.startswith('/'):
        raise ValueError('service_name must be an absolute ROS service name')
    timeout = finite_number(data.get('wait_timeout_s', 10.0), 'wait_timeout_s')
    if timeout <= 0.0:
        raise ValueError('wait_timeout_s must be greater than zero')

    normalized = {
        'mission_id': str(task.get('mission_id', 'point_goal')),
        'agent_id': str(namespace_override or task.get('agent_id') or namespace),
        'capability_id': 'goto_waypoints',
        'primitive_id': str(task.get('primitive_id', 'point_goal_001')),
        'params': {'waypoint_frame': 'gps' if frame == 'gps' or nav_origin else 'enu',
                   'waypoints': [output_point]},
    }
    return service_name, normalized, timeout


def main(argv=None):
    parser = argparse.ArgumentParser(description='从 YAML 经 task_pipeline 下发定点导航并显示 feedback')
    parser.add_argument('--file', type=Path, default=None,
                        help='YAML task file (default: installed point_goal.yaml)')
    parser.add_argument('--namespace', help='robot namespace; defaults to YAML')
    parser.add_argument('--service', help='override ExecuteTask service name')
    parser.add_argument('--wait-timeout', type=float, help='seconds to wait for the pipeline service and response')
    parser.add_argument('--dry-run', action='store_true', help='validate and preview without ROS')
    parser.add_argument('--check', action='store_true', help='check the pipeline service without submitting')
    args, ros_args = parser.parse_known_args(argv)

    try:
        path = args.file or _default_task_file()
        raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        if not isinstance(raw, dict):
            raise ValueError('YAML root must be a mapping')
        timeout = args.wait_timeout if args.wait_timeout is not None else finite_number(
            raw.get('wait_timeout_s', 10.0), 'wait_timeout_s')
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('--wait-timeout must be finite and greater than zero')
        frame = task_coordinate_frame(raw)
        nav_origin = None
        if frame == 'enu' and not (args.dry_run or args.check):
            from task_pipeline.service_client import resolve_navigation_origin
            namespace = args.namespace or raw.get('robot_namespace', '')
            nav_origin = resolve_navigation_origin(
                namespace, timeout, ros_args)
        service_name, task, configured_timeout = load_task(
            path, namespace_override=args.namespace, service_override=args.service,
            nav_origin=nav_origin)
        timeout = args.wait_timeout if args.wait_timeout is not None else configured_timeout
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('--wait-timeout must be finite and greater than zero')
    except (OSError, ValueError, RuntimeError, yaml.YAMLError) as exc:
        print(f'任务配置错误: {exc}', file=sys.stderr)
        return 2

    print(f'任务文件: {path}\nExecuteTask 服务: {service_name}')
    if args.dry_run:
        print(json.dumps(task, ensure_ascii=False, indent=2))
        return 0

    try:
        from task_pipeline.service_client import submit
    except ImportError as exc:
        print(f'ROS Action 接口不可用，请 source 项目工作区: {exc}', file=sys.stderr)
        return 2

    return submit(task, service_name, args.namespace or task['agent_id'], timeout, args.check, ros_args)


if __name__ == '__main__':
    raise SystemExit(main())
