"""Bind task geometry and ROS transport to upstream parameters; no search policy."""
import copy
import math
from pathlib import Path
import yaml


def coverage_threshold(task):
    """Observed-cell fraction for successful completion; zero disables it."""
    custom = task.get('co_search', {})
    if not isinstance(custom, dict):
        raise ValueError('co_search must be a mapping')
    value = custom.get('coverage_threshold', .95)
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or not 0 <= value <= 1):
        raise ValueError('co_search.coverage_threshold must be a finite number in 0..1')
    return float(value)


def search_timeout_coefficient(task):
    """Minimum active search seconds per square metre before coverage completion."""
    value = task.get('co_search', {}).get('timeout_coefficient_s_per_m2', 1.0)
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or value < 0):
        raise ValueError('co_search.timeout_coefficient_s_per_m2 must be finite and >= 0')
    return float(value)


def engine_parameters(task, region, map_frame, resource_dir, package_share, package_prefix, start_position=None):
    settings = yaml.safe_load((Path(package_share)/'config/area_search.yaml').read_text())
    base = copy.deepcopy(settings['co_search']['parameters'])
    custom = task.get('co_search', {})
    if not isinstance(custom, dict) or not isinstance(custom.get('parameters', {}), dict):
        raise ValueError('co_search.parameters must be a mapping')
    base.update(copy.deepcopy(custom.get('parameters', {})))
    cloud_filter = custom.get('cloud_filter', {})
    if not isinstance(cloud_filter, dict):
        raise ValueError('co_search.cloud_filter must be a mapping')
    x_enabled = cloud_filter.get('enable_x_filter', True)
    if not isinstance(x_enabled, bool):
        raise ValueError('cloud_filter.enable_x_filter must be boolean')
    filter_values = {}
    for key, default in (('min_x_m', .05), ('max_x_m', 4.0),
                         ('voxel_size_m', .1), ('max_future_offset_s', .5),
                         ('tf_lookup_timeout_s', 2.0), ('accumulation_window_s', 1.0)):
        value = cloud_filter.get(key, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value)):
            raise ValueError('cloud_filter.'+key+' must be a finite number')
        filter_values[key] = float(value)
    if filter_values['min_x_m'] >= filter_values['max_x_m']:
        raise ValueError('cloud_filter requires min_x_m < max_x_m')
    if (filter_values['voxel_size_m'] <= 0 or filter_values['max_future_offset_s'] < 0 or
            filter_values['tf_lookup_timeout_s'] <= 0):
        raise ValueError('cloud_filter requires voxel_size_m > 0, max_future_offset_s >= 0, and tf_lookup_timeout_s > 0')
    if filter_values['accumulation_window_s'] <= 0:
        raise ValueError('cloud_filter.accumulation_window_s must be positive')
    min_points = cloud_filter.get('min_accumulated_points', 10)
    if isinstance(min_points, bool) or not isinstance(min_points, int) or not 1 <= min_points <= 1000000:
        raise ValueError('cloud_filter.min_accumulated_points must be an integer in 1..1000000')
    base['adapter/cloud_min_accumulated_points'] = min_points
    base.update({'adapter/cloud_'+key: value for key, value in filter_values.items()})
    base['adapter/cloud_enable_x_filter'] = x_enabled
    strategy = task.get('search_strategy', 'co_search')
    if strategy not in ('co_search', 'parallel_lines'):
        raise ValueError('search_strategy must be co_search or parallel_lines')
    base['adapter/search_strategy'] = strategy
    lines = custom.get('parallel_lines', {})
    if not isinstance(lines, dict):
        raise ValueError('co_search.parallel_lines must be a mapping')
    for key, default in (('lane_spacing_m', 2.0), ('boundary_margin_m', 0.8),
                         ('waypoint_tolerance_m', 0.5)):
        value = lines.get(key, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or value <= 0):
            raise ValueError('co_search.parallel_lines.'+key+' must be finite and positive')
        base['adapter/parallel_'+key] = float(value)
    ns = task['robot_namespace']
    # Upstream map bounds are centered at zero; data are rigidly transformed to
    # the task rectangle frame, preserving rotated rectangle semantics.
    base.update({
        'exploration/drone_id': 1, 'exploration/drone_num': 1,
        'allocate/region_rows': 1, 'allocate/region_cols': 1,
        'ppu/enable_path_planning': False, 'ppu/enable_region_allocation': False,
        'map_ros/obstacle_source': 'external', 'map_ros/frame_id': ns+'/co_search_frame',
        'map_ros/show_all_map': True, 'map_ros/global_map_pub_period_sec': 1.0,
        'sdf_map/map_size_x': region.length+2.,
        'sdf_map/map_size_y': region.width+2.,
        'sdf_map/box_min_x': -region.length/2, 'sdf_map/box_max_x': region.length/2,
        'sdf_map/box_min_y': -region.width/2, 'sdf_map/box_max_y': region.width/2,
        'exploration/tsp_dir': str(Path(resource_dir)/'tsp'),
        'exploration/mtsp_dir': str(Path(resource_dir)/'mtsp'),
        'exploration/lkh_binary': str(Path(package_prefix)/'lib/co_search_nav/co_search_lkh'),
        'adapter/map_frame': map_frame, 'adapter/center_x': float(region.east),
        'adapter/center_y': float(region.north), 'adapter/region_yaw': float(region.yaw),
        'adapter/odom_topic': '/'+ns+'/odom',
        'adapter/odom_frame': task.get('odom_frame', '{namespace}/odom').format(namespace=ns),
        'adapter/cloud_topic': task.get('topics', {}).get('cloud', 'livox/lidar'),
        'adapter/plan_period_s': float(custom.get('plan_period_s', 1.5)),
        'adapter/map_period_s': float(custom.get('map_period_s', .2)),
        'adapter/path_spacing_m': float(custom.get('path_spacing_m', .1)),
        'adapter/sensor_timeout_s': float(task['options']['sensor_timeout_s']),
    })
    base['adapter/task_length_m'] = float(region.length)
    base['adapter/task_width_m'] = float(region.width)
    entry = custom.get('entry', {})
    if not isinstance(entry, dict):
        raise ValueError('co_search.entry must be a mapping')
    for key, default in (('inset_m', .8), ('sample_spacing_m', .5), ('clearance_m', .7),
                         ('retry_cooldown_s', 5.), ('timeout_s', 30.)):
        value = entry.get(key, default)
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or value < 0 or (key != 'clearance_m' and value == 0)):
            raise ValueError('invalid co_search.entry.'+key)
        base['adapter/entry_'+key] = float(value)
    if base['adapter/entry_inset_m'] <= base['adapter/entry_clearance_m']:
        raise ValueError('entry.inset_m must exceed entry.clearance_m')
    for key, default in (('max_plan_attempts', 4), ('max_execution_failures', 4)):
        value = entry.get(key, default)
        if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 32:
            raise ValueError('entry.'+key+' must be an integer in 1..32')
        base['adapter/entry_'+key] = value
    if start_position is not None:
        sx, sy = region.local(*start_position)
        # Storage may contain the real start for the entry trajectory. The
        # native exploration box remains exactly the requested rectangle.
        for axis, value in (('x', sx), ('y', sy)):
            base['sdf_map/map_size_'+axis] = max(
                base['sdf_map/map_size_'+axis], 2*(abs(value)+1.))
    for name in ('adapter/plan_period_s', 'adapter/map_period_s', 'adapter/path_spacing_m', 'adapter/sensor_timeout_s'):
        if not math.isfinite(base[name]) or base[name] <= 0:
            raise ValueError(name+' must be positive and finite')
    low = base['sdf_map/min_obstacle_height']; high = base['sdf_map/max_obstacle_height']
    if (not isinstance(low, (int, float)) or isinstance(low, bool) or
            not isinstance(high, (int, float)) or isinstance(high, bool) or
            not math.isfinite(low) or not math.isfinite(high) or not 0 <= low < high):
        raise ValueError('sdf_map obstacle height requires finite 0 <= min < max')
    for directory in ('tsp', 'mtsp'):
        (Path(resource_dir)/directory).mkdir(parents=True, exist_ok=True)
    return base


def native_remappings(namespace):
    """Isolate native absolute application topics; keep real navigation TF."""
    names = ['sdf_map/occupancy_all', 'sdf_map/occupancy_local', 'sdf_map/occupancy_local_inflate',
             'sdf_map/update_range', 'sdf_map/depth_cloud', 'sdf_map/unknown', 'sdf_map/esdf',
             'map_ros/cloud', 'map_ros/odom', 'map_ros/pose', 'map_ros/depth',
             'swarm_expl/drone_state', 'swarm_drones/swarm_drone_basecoor',
             'multi_map_manager/chunk_stamps_send', 'multi_map_manager/chunk_stamps_recv',
             'multi_map_manager/chunk_data_send', 'multi_map_manager/chunk_data_recv',
             'planning_vis/trajectory', 'planning_vis/frontier', 'planning_vis/viewpoints',
             'multi_map_manager/chunk_data/visualization', 'multi_map_manager/chunk_data',
             'planning_vis/topo_path', 'planning_vis/prediction', 'planning_vis/visib_constraint',
             'planning/snn_path', 'planning/snn_path_1', 'solve_tsp_1', 'solve_acvrp_1']
    return [('/'+name, '/'+namespace+'/co_search/native/'+name) for name in names]
