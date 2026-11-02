"""Submit/cancel area search through ExecuteTask and print pipeline feedback."""
import argparse
import json
from pathlib import Path
import sys
import yaml

from .area_search_core import validate_task


def load_task(path, namespace=None):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if namespace is not None:
        data['robot_namespace'] = namespace
        data['agent_id'] = namespace
    task = validate_task(data)
    ns = task['robot_namespace']
    if task.get('agent_id', ns) != ns:
        raise ValueError('agent_id must match robot_namespace')
    task['agent_id'] = ns
    primitive_id = str(task.get('primitive_id', 'area_search_001'))
    if not primitive_id:
        raise ValueError('primitive_id must not be empty')
    task['primitive_id'] = primitive_id
    return f'/agent/{ns}/execute_task', dict(mission_id=task['mission_id'],
        agent_id=ns, primitive_id=primitive_id, capability_id='area_search', params=task)


def main(argv=None):
    parser = argparse.ArgumentParser(description='YAML rectangle frontier exploration via ExecuteTask')
    parser.add_argument('--file', type=Path)
    parser.add_argument('--namespace')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--cancel', action='store_true')
    parser.add_argument('--wait-timeout', type=float, default=10)
    args, ros_args = parser.parse_known_args(argv)
    try:
        if args.file is None:
            from ament_index_python.packages import get_package_share_directory
            args.file = Path(get_package_share_directory('task_pipeline'))/'config/area_search.yaml'
        service, task = load_task(args.file, args.namespace)
        if args.wait_timeout <= 0 or not __import__('math').isfinite(args.wait_timeout):
            raise ValueError('--wait-timeout must be positive and finite')
        if args.cancel:
            task['capability_id'] = 'cancel_area_search'
    except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
        print(f'区域任务配置错误: {exc}', file=sys.stderr); return 2
    print(f'Pipeline service: {service}')
    if args.dry_run:
        print(json.dumps(task, ensure_ascii=False, indent=2)); return 0
    from task_pipeline.service_client import submit
    return submit(task, service, task['agent_id'], args.wait_timeout, args.check, ros_args)


if __name__ == '__main__':
    raise SystemExit(main())
