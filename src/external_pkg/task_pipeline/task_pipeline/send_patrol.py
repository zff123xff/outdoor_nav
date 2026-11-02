#!/usr/bin/env python3
"""Send a YAML patrol mission to the patrol Action executor and print feedback."""

import argparse
import json
import math
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from task_pipeline.coordinates import enu_to_gps, finite_number, task_coordinate_frame


def _default_task_file() -> Path:
    return Path(get_package_share_directory('task_pipeline')) / 'config' / 'patrol_goal.yaml'


def load_task(path, namespace_override=None, service_override=None, nav_origin=None):
    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError('任务文件根节点必须是 YAML 映射')
    task = data.get('task')
    if not isinstance(task, dict):
        raise ValueError('task 必须是 YAML 映射')
    params = task.get('params')
    if not isinstance(params, dict):
        raise ValueError('task.params 必须是 YAML 映射')

    frame = task_coordinate_frame(data)
    waypoints = params.get('waypoints')
    if not isinstance(waypoints, list) or len(waypoints) != 2:
        raise ValueError('巡线任务必须有且仅有两个端点')

    gps_waypoints = []
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict) or not isinstance(waypoint.get(frame), dict):
            raise ValueError(f'waypoints[{index}].{frame} 必须是坐标映射')
        point = waypoint[frame]
        if frame == 'gps':
            lat = finite_number(point.get('lat'), f'waypoints[{index}].gps.lat')
            lon = finite_number(point.get('lon'), f'waypoints[{index}].gps.lon')
            alt = finite_number(point.get('alt', 0.0), f'waypoints[{index}].gps.alt')
            if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
                raise ValueError(f'waypoints[{index}] GPS 经纬度超出范围')
        else:
            east = finite_number(point.get('x'), f'waypoints[{index}].enu.x')
            north = finite_number(point.get('y'), f'waypoints[{index}].enu.y')
            up = finite_number(point.get('z', 0.0), f'waypoints[{index}].enu.z')
            if nav_origin is None:
                gps_waypoints.append({'x': east, 'y': north, 'z': up})
                continue
            lat, lon, alt = enu_to_gps(east, north, up, nav_origin)
        gps_waypoints.append({'lat': lat, 'lon': lon, 'alt': alt})

    params_out = dict(params)
    params_out['waypoint_frame'] = 'gps' if frame == 'gps' or nav_origin else 'enu'
    params_out['waypoints'] = gps_waypoints
    max_laps = finite_number(params.get('max_laps', 1), 'max_laps')
    hold_time = finite_number(params.get('hold_time_at_endpoint_s', 0.0),
                               'hold_time_at_endpoint_s')
    if not max_laps.is_integer() or max_laps < 0 or max_laps > 4294967295:
        raise ValueError('max_laps 必须是 0 到 4294967295 的整数；0 表示无限往返')
    if hold_time < 0.0:
        raise ValueError('hold_time_at_endpoint_s 不能小于 0')

    namespace = str(namespace_override or data.get('robot_namespace', '')).strip('/')
    service_name = service_override or (
        f'/agent/{namespace}/execute_task' if namespace_override is not None
        else data.get('service_name') or f'/agent/{namespace}/execute_task')
    if not isinstance(service_name, str) or not service_name.startswith('/'):
        raise ValueError('service_name 必须是绝对 ROS service 名称')
    timeout = finite_number(data.get('wait_timeout_s', 10.0), 'wait_timeout_s')
    if timeout <= 0.0:
        raise ValueError('wait_timeout_s 必须大于 0')

    normalized = {
        'mission_id': str(task.get('mission_id', 'patrol_demo')),
        'agent_id': str(namespace_override or task.get('agent_id') or namespace),
        'capability_id': 'patrol_line',
        'primitive_id': str(task.get('primitive_id', 'patrol_001')),
        'params': params_out,
    }
    normalized['params']['max_laps'] = int(max_laps)
    normalized['params']['hold_time_at_endpoint_s'] = hold_time
    return service_name, normalized, timeout


def main(argv=None):
    parser = argparse.ArgumentParser(description='从 YAML 经 task_pipeline 下发巡线并实时显示 feedback')
    parser.add_argument('file', nargs='?', help='任务 YAML 路径；默认使用 task_pipeline 包内配置')
    parser.add_argument('--file', dest='task_file', type=Path, help='任务 YAML 路径')
    parser.add_argument('--namespace', help='机器人命名空间；默认读取 YAML')
    parser.add_argument('--service', help='覆盖 ExecuteTask 服务名称')
    parser.add_argument('--wait-timeout', type=float, help='覆盖等待 pipeline 服务和响应的秒数')
    parser.add_argument('--dry-run', action='store_true', help='校验并预览 Action goal，不调用 ROS')
    parser.add_argument('--check', action='store_true', help='只检查 pipeline 服务，不提交任务')
    args, ros_args = parser.parse_known_args(argv)

    try:
        if args.file and args.task_file:
            raise ValueError('请只使用 --file 或位置参数指定一个 YAML')
        path = args.task_file or (Path(args.file) if args.file else _default_task_file())
        with path.open(encoding='utf-8') as stream:
            raw = yaml.safe_load(stream)
        if not isinstance(raw, dict):
            raise ValueError('任务文件根节点必须是 YAML 映射')
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
            raise ValueError('--wait-timeout 必须是大于 0 的有限数值')
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
