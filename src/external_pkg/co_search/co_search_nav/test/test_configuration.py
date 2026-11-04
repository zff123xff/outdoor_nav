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
    assert params['sdf_map/box_min_x'] == -5.
    assert params['sdf_map/box_max_y'] == 4.
    assert params['optimization/lambda_smooth'] == 7.
    assert params['map_ros/obstacle_source'] == 'external'
    assert params['ppu/enable_path_planning'] is False
    assert params['exploration/tsp_dir'] != params['exploration/mtsp_dir']
    assert (tmp_path/'tsp').is_dir() and (tmp_path/'mtsp').is_dir()
    assert not any('revisit_cost_weight' in key or 'fill_enclosed' in key for key in params)


@pytest.mark.parametrize('name,value', [('path_spacing_m', 0), ('plan_period_s', -1)])
def test_invalid_adapter_dimensions_are_rejected(tmp_path, name, value):
    data = task(); data['co_search'][name] = value
    with pytest.raises(ValueError):
        engine_parameters(data, rectangle_for_task(data, None), 'robot_1/map', tmp_path, SHARE, '/install')


def test_search_path_and_optimization_sources_are_copied_byte_for_byte():
    manifest = json.loads((ROOT/'UPSTREAM_SHA256.json').read_text())
    allowed = {'plan_env/CMakeLists.txt', 'bspline_opt/CMakeLists.txt',
               'exploration_manager/CMakeLists.txt', 'plan_manage/CMakeLists.txt',
               'exploration_manager/include/exploration_manager/fast_exploration_manager.h',
               'plan_env/include/plan_env/sdf_map.h', 'plan_env/src/sdf_map.cpp', 'plan_env/src/map_ros.cpp'}
    changed = {name for name, digest in manifest.items()
               if hashlib.sha256((ROOT/'upstream'/name).read_bytes()).hexdigest() != digest}
    assert changed <= allowed
    for name in ('exploration_manager/src/fast_exploration_manager.cpp',
                 'active_perception/src/frontier_finder.cpp', 'path_searching/src/snn_path.cpp',
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
