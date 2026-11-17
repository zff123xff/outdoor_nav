"""Read-only ROS task ingress diagnostics; never publishes a task or motion."""
import os
from pathlib import Path

ROS_ENV_KEYS = ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY',
                'ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_USE_QOS_FROM_XML')


def environment(values=None):
    values = os.environ if values is None else values
    return {key: values.get(key, '<unset>') for key in ROS_ENV_KEYS}


def navigation_processes(proc_root='/proc'):
    """Inspect only navigation processes and an explicit environment allowlist."""
    results = []
    try:
        entries = list(Path(proc_root).iterdir())
    except OSError:
        return results
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            argv = [s.decode(errors='replace') for s in (entry/'cmdline').read_bytes().split(b'\0') if s]
            if not argv:
                continue
            executable = Path(argv[0]).name
            if executable.startswith('python') and len(argv) > 1:
                executable = Path(argv[1]).name
            if executable not in ('input_bridge_node', 'pipeline_dispatcher'):
                continue
            item = dict(pid=int(entry.name), executable=executable,
                        namespace=next((s.split(':=', 1)[1] for s in argv if s.startswith('__ns:=')), '/'))
            try:
                raw = (entry/'environ').read_bytes().split(b'\0')
                values = dict(s.decode(errors='replace').split('=', 1) for s in raw if b'=' in s)
                item['environment'] = environment(values)
            except OSError:
                item['environment_error'] = 'Cannot read process environment (permissions)'
            results.append(item)
        except OSError:
            continue
    return sorted(results, key=lambda item: item['pid'])


def origin_service_error(node, target):
    bridges = sorted(ns.rstrip('/')+'/'+name for name, ns in node.get_node_names_and_namespaces()
                     if name == 'nav2_input_bridge')
    visible = ', '.join(bridges) if bridges else 'none'
    return (f'未发现参数服务 {target}/get_parameters; '
            f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0 (unset/default)")}, '
            f'RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION", "<default>")}; '
            f'当前发现的 nav2_input_bridge: {visible}. '
            '节点可能已启动，但此终端与导航进程的 namespace / ROS domain / DDS 配置不一致。'
            '请运行 bash scripts/dev/check_task_ingress.sh 检查；任务尚未发布。')
