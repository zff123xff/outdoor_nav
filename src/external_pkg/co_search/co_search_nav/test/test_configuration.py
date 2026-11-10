"""Geometry, native parameter preservation, private resources and provenance."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
import yaml
from co_search_nav.configuration import engine_parameters, native_remappings
from co_search_nav.task_geometry import validate_task, rectangle_for_task

ROOT = Path(__file__).resolve().parents[2]
SHARE = ROOT/'co_search_nav'


def task():
    return validate_task(dict(robot_namespace='robot_1', mission_id='native', search_controller='co_search',
        coordinate_mode='enu', center_enu=dict(east=3., north=-2.), rectangle=dict(length_m=10., width_m=8., yaw_deg=30.),
        options={}, co_search={'parameters': {'optimization/lambda_smooth': 7.}}))


def test_bind_native_parameters_without_legacy_search_policy(tmp_path):
    value = task(); region = rectangle_for_task(value, None)
    params = engine_parameters(value, region, 'robot_1/map', tmp_path, SHARE, '/workspace/install/co_search_nav')
    assert params['exploration/drone_num'] == 1
    assert params['allocate/region_rows'] == params['allocate/region_cols'] == 1
    assert params['adapter/center_x'] == 3. and params['adapter/center_y'] == -2.
    assert params['adapter/region_yaw'] == pytest.approx(region.yaw)
    assert params['adapter/odom_frame'] == 'robot_1/odom'
    assert params['sdf_map/box_min_x'] == -5.
    assert params['sdf_map/box_max_y'] == 4.
    assert params['optimization/lambda_smooth'] == 7.
    assert params['map_ros/obstacle_source'] == 'external'
    assert params['ppu/enable_path_planning'] is False
    assert params['exploration/tsp_dir'] != params['exploration/mtsp_dir']
    assert (tmp_path/'tsp').is_dir() and (tmp_path/'mtsp').is_dir()
    assert not any('revisit_cost_weight' in key or 'fill_enclosed' in key for key in params)


def test_explicit_odometry_fallback_frame(tmp_path):
    value = task()
    value['odom_frame'] = '{namespace}/local_odom'
    params = engine_parameters(value, rectangle_for_task(value, None), 'robot_1/map',
                               tmp_path, SHARE, '/workspace/install/co_search_nav')
    assert params['adapter/odom_frame'] == 'robot_1/local_odom'


@pytest.mark.parametrize('name,value', [('path_spacing_m', 0), ('plan_period_s', -1)])
def test_invalid_adapter_dimensions_are_rejected(tmp_path, name, value):
    data = task(); data['co_search'][name] = value
    with pytest.raises(ValueError):
        engine_parameters(data, rectangle_for_task(data, None), 'robot_1/map', tmp_path, SHARE, '/install')


def test_vendored_upstream_sources_match_manifest_except_documented_forks():
    """upstream/ 是 co-search 原版副本；只有登记在 allowed 里的文件允许本地改动。

    UPSTREAM_SHA256.json 记录的是原版哈希，不因本地 fork 而改写，便于后续重新同步。
    fast_exploration_manager.cpp 与 expl_data.h 的唯一有意改动是目标承诺（视点锁），
    避免车体在近似等价的前沿之间来回切换；机制与参数见 docs/AREA_SEARCH_GOAL_LOCK.md。
    """
    manifest = json.loads((ROOT/'UPSTREAM_SHA256.json').read_text())
    allowed = {'plan_env/CMakeLists.txt', 'bspline_opt/CMakeLists.txt',
               'exploration_manager/CMakeLists.txt', 'plan_manage/CMakeLists.txt',
               'exploration_manager/include/exploration_manager/fast_exploration_manager.h',
               'plan_env/include/plan_env/sdf_map.h', 'plan_env/src/sdf_map.cpp', 'plan_env/src/map_ros.cpp',
               # 目标承诺（视点锁）：单机区域搜索的目标滞回，见 docs/AREA_SEARCH_GOAL_LOCK.md。
               'exploration_manager/src/fast_exploration_manager.cpp',
               'exploration_manager/include/exploration_manager/expl_data.h',
               'active_perception/include/active_perception/graph_node.h',
               'active_perception/src/graph_node.cpp'}
    changed = {name for name, digest in manifest.items()
               if hashlib.sha256((ROOT/'upstream'/name).read_bytes()).hexdigest() != digest}
    assert changed <= allowed
    for name in ('active_perception/src/frontier_finder.cpp', 'path_searching/src/snn_path.cpp',
                 'plan_manage/src/planner_manager.cpp', 'bspline_opt/src/bspline_optimizer.cpp'):
        assert name not in changed


def test_native_application_topics_are_isolated_but_navigation_tf_is_consumed():
    pairs = native_remappings('test_robot')
    assert all(target.startswith('/test_robot/co_search/') for _, target in pairs)
    assert not any(source in ('/tf', '/tf_static', '/cmd_vel') for source, _ in pairs)


def test_search_box_does_not_expand_for_outside_start_or_old_margin_option(tmp_path):
    value = task(); value['co_search']['mapping_margin_m'] = 2.
    region = rectangle_for_task(value, None)
    params = engine_parameters(value, region, 'robot_1/map', tmp_path, SHARE,
                               '/install', start_position=(100., 100.))
    assert params['sdf_map/box_min_x'] == -5.
    assert params['sdf_map/box_max_x'] == 5.
    assert params['sdf_map/box_min_y'] == -4.
    assert params['sdf_map/box_max_y'] == 4.
    assert params['sdf_map/map_size_x'] > region.length


def test_search_uses_height_band_from_its_single_runtime_configuration(tmp_path):
    mission = yaml.safe_load((SHARE/'config/area_search.yaml').read_text())
    source = mission['co_search']['parameters']
    params = engine_parameters(validate_task(mission), rectangle_for_task(mission, None),
                               'robot_1/map', tmp_path, SHARE, '/install')
    assert params['sdf_map/min_obstacle_height'] == source['sdf_map/min_obstacle_height']
    assert params['sdf_map/max_obstacle_height'] == source['sdf_map/max_obstacle_height']


@pytest.mark.parametrize('low,high', [(.8, .4), (.4, .4), (-.1, .8), (float('nan'), .8)])
def test_invalid_height_band_is_rejected_before_starting_native_engine(tmp_path, low, high):
    data = task(); data['co_search']['parameters'].update({
        'sdf_map/min_obstacle_height': low, 'sdf_map/max_obstacle_height': high})
    with pytest.raises(ValueError, match='obstacle height'):
        engine_parameters(data, rectangle_for_task(data, None), 'robot_1/map', tmp_path, SHARE, '/install')


def test_cloud_filter_parameters_are_forwarded(tmp_path):
    value = task()
    value['co_search']['cloud_filter'] = dict(enable_x_filter=False, min_x_m=-8., max_x_m=8.,
                                             voxel_size_m=.2, max_future_offset_s=.1)
    params = engine_parameters(value, rectangle_for_task(value, None), 'robot_1/map',
                               tmp_path, SHARE, '/workspace/install/co_search_nav')
    assert params['adapter/cloud_enable_x_filter'] is False
    assert params['adapter/cloud_min_x_m'] == -8.
    assert params['adapter/cloud_max_x_m'] == 8.
    assert params['adapter/cloud_voxel_size_m'] == .2
    assert params['adapter/cloud_max_future_offset_s'] == .1


@pytest.mark.parametrize('settings', [
    {'enable_x_filter': 'false'}, {'min_x_m': 4., 'max_x_m': 4.},
    {'min_x_m': float('nan')}, {'max_x_m': float('inf')},
    {'voxel_size_m': 0.}, {'voxel_size_m': True}, {'max_future_offset_s': -.1},
])
def test_invalid_cloud_filter_is_rejected(tmp_path, settings):
    value = task()
    value['co_search']['cloud_filter'] = settings
    with pytest.raises(ValueError):
        engine_parameters(value, rectangle_for_task(value, None), 'robot_1/map',
                          tmp_path, SHARE, '/workspace/install/co_search_nav')
