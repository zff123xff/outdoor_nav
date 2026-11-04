"""Bind task geometry and ROS transport to upstream parameters; no search policy."""
import copy
import math
from pathlib import Path
import yaml


def engine_parameters(task, region, map_frame, resource_dir, package_share, package_prefix, start_position=None):
    settings = yaml.safe_load((Path(package_share)/'config/area_search.yaml').read_text())
    base = copy.deepcopy(settings['co_search']['parameters'])
    custom = task.get('co_search', {})
    if not isinstance(custom, dict) or not isinstance(custom.get('parameters', {}), dict):
        raise ValueError('co_search.parameters must be a mapping')
    base.update(copy.deepcopy(custom.get('parameters', {})))
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
        'adapter/cloud_topic': task.get('topics', {}).get('cloud', 'livox/lidar'),
        'adapter/plan_period_s': float(custom.get('plan_period_s', 1.5)),
        'adapter/map_period_s': float(custom.get('map_period_s', .2)),
        'adapter/path_spacing_m': float(custom.get('path_spacing_m', .1)),
        'adapter/sensor_timeout_s': float(task['options']['sensor_timeout_s']),
    })
    base['adapter/task_length_m'] = float(region.length)
    base['adapter/task_width_m'] = float(region.width)
    base['adapter/entry_inset_m'] = .1+float(base['sdf_map/resolution'])/2
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
