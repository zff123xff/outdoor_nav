"""Safety/algorithm invariants for rectangle frontier exploration."""
from pathlib import Path
import copy
import math
import numpy as np
import pytest
import yaml
from task_pipeline.area_search_core import (Rectangle, ExplorationMap, bfs, reconstruct,
    gps_to_enu, rectangle_for_task, task_signature, validate_task)
from task_pipeline.send_area_search import load_task, main

CONFIG = Path(__file__).parents[1]/'config/area_search.yaml'


def task():
    return validate_task(yaml.safe_load(CONFIG.read_text()))


def test_partition_natural_order_and_rotated_coordinates():
    region = Rectangle(10, -5, 30, 12, math.pi/3)
    names = ['robot_10', 'robot_2', 'robot_1']
    strips = [region.partition(names, n) for n in ['robot_1', 'robot_2', 'robot_10']]
    assert [round(region.local(s.east, s.north)[0]) for s in strips] == [-10, 0, 10]
    assert all(s.length == 10 and s.width == 12 for s in strips)
    assert sum(s.length*s.width for s in strips) == region.length*region.width
    for strip in strips:
        for x, y in strip.corners():
            u, v = region.local(x, y)
            assert abs(u) <= 15+1e-9 and abs(v) <= 6+1e-9


def test_gps_origin_and_enu_select_same_rectangle():
    t = task(); origin = (40.0, 113.0, 1200.0)
    assert gps_to_enu(*origin, origin) == pytest.approx((0, 0, 0))
    east, north, _ = gps_to_enu(40.0001, 113.0001, 1200, origin)
    assert east > 0 and north > 0
    t['coordinate_mode'] = 'gps'; t['center_gps'] = dict(latitude=40.0001, longitude=113.0001, altitude=1200)
    a = rectangle_for_task(t, origin)
    t['coordinate_mode'] = 'enu'; t['center_enu'] = dict(east=east, north=north)
    assert a == rectangle_for_task(t, origin)


def test_team_signature_independent_of_self_and_detects_mission_mismatch():
    a = task(); b = copy.deepcopy(a); b['robot_namespace'] = 'robot_2'
    team = ['robot_1', 'robot_2']; origin = (0, 0, 0)
    assert task_signature(a, origin, team) == task_signature(b, origin, team[::-1])
    b['rectangle']['width_m'] += 1
    assert task_signature(a, origin, team) != task_signature(b, origin, team)
    assert task_signature(a, origin, team) != task_signature(a, (1, 0, 0), team)


def test_bfs_does_not_cut_obstacle_corners():
    free = np.array([[True, False], [False, True]])
    dist, parent = bfs(free, (0, 0))
    assert dist[1, 1] == -1
    assert reconstruct(parent, (0, 0), (1, 1)) == []


def test_bfs_routes_through_wall_opening():
    free = np.ones((12, 12), dtype=bool); free[:10, 5] = False
    dist, parent = bfs(free, (2, 2)); path = reconstruct(parent, (2, 2), (9, 2))
    assert dist[2, 9] > 0 and max(y for _, y in path) >= 10
    assert all(free[y, x] for x, y in path)


def test_unobserved_and_outside_remain_unknown():
    t = task(); m = ExplorationMap(Rectangle(-3, -2, 8, 6, .3), t['options'])
    assert np.all(m.planar() == -1)
    m.integrate((0, 0, .5), [])
    assert np.all(m.planar() == -1)  # no hit returns never fabricate a free disk


def test_obstacle_endpoint_and_occluded_space():
    t = task(); m = ExplorationMap(Rectangle(0, 0, 12, 10), t['options'])
    m.integrate((0, 0, .25), [(2, 0, .25)])
    x, y = m.cell(2, 0)
    assert m.planar()[y, x] == 100
    x, y = m.cell(3, 0)
    assert m.planar()[y, x] == -1
    x, y = m.cell(1, 0)
    # One high ray cannot certify the whole robot-height column free.
    assert m.planar()[y, x] == -1


def test_ground_endpoint_not_an_obstacle():
    t = task(); m = ExplorationMap(Rectangle(0, 0, 12, 10), t['options'])
    m.integrate((0, 0, .5), [(2, 0, 0)])
    assert not np.any(m.planar() == 100)


def test_frontier_target_stays_in_assigned_strip_and_known_reachable_space():
    t = task(); m = ExplorationMap(Rectangle(0, 0, 12, 10), t['options'])
    # Synthetic observed footprint, no simulator/real robot receives these data.
    observed = m.mask(Rectangle(0, 0, 6, 6))
    m.seen[observed] = True; m.evidence[observed] = -2
    assigned = m.region.partition(['robot_1', 'robot_2'], 'robot_2')
    path, ratio, count, target = m.plan((0, 0, 0), assigned, set())
    assert path and count > 0 and 0 < ratio < 1
    assert assigned.contains(*m.position(*target))
    assert all(m.region.contains(x, y, t['options']['robot_radius_m']) for x, y in path)
    assert all(m.planar()[m.cell(x, y)[1], m.cell(x, y)[0]] == 0 for x, y in path)


def test_all_unknown_is_not_completed():
    t = task(); m = ExplorationMap(Rectangle(0, 0, 12, 10), t['options'])
    path, ratio, count, target = m.plan((0, 0, 0), m.region, set())
    assert not path and ratio == 0 and count == 0 and target is None


def test_area_metrics_count_unique_observed_cells_only():
    t = task(); region = Rectangle(0, 0, 8, 6)
    m = ExplorationMap(region, t['options'])
    own = np.argwhere(m.mask(region) & m.roi)
    observed = own[:len(own)//2]
    layers = min(m.nz, max(1, math.ceil((t['options']['free_space_height_m']-
                                         t['options']['min_obstacle_height_m'])/m.res)))
    for y, x in observed:
        m.seen[y, x, :layers] = True
        m.evidence[y, x, :layers] = -2
    metrics = m.area_metrics(region)
    assert metrics['assigned_area_m2'] > 0
    assert metrics['searched_area_m2'] == pytest.approx(metrics['assigned_area_m2'] * 0.5, abs=m.res**2)
    assert metrics['observed_ratio'] == pytest.approx(0.5, abs=1/len(own))


@pytest.mark.parametrize('key,value', [('resolution_m', 0), ('sensor_timeout_s', -1),
    ('max_cloud_points', 1.5), ('completion_observed_ratio', 1.1), ('robot_radius_m', True)])
def test_invalid_options_rejected(key, value):
    t = task(); t['options'][key] = value
    with pytest.raises(ValueError):
        validate_task(t)


def test_invalid_mode_and_huge_map():
    t = task(); t['coordinate_mode'] = 'wrong'
    with pytest.raises(ValueError): validate_task(t)
    t = task(); t['rectangle']['length_m'] = 10000
    with pytest.raises(ValueError): validate_task(t)


def test_sender_uses_pipeline_namespace_override():
    service, request = load_task(CONFIG, 'robot_2')
    assert service == '/agent/robot_2/execute_task'
    assert request['capability_id'] == 'area_search'
    assert request['params']['robot_namespace'] == request['agent_id'] == 'robot_2'


def test_dry_run_does_not_import_ros(capsys):
    assert main(['--file', str(CONFIG), '--namespace', 'robot_2', '--dry-run']) == 0
    assert '/agent/robot_2/execute_task' in capsys.readouterr().out


def test_robot_outside_its_strip_explores_toward_entry_instead_of_finishing():
    t = task(); m = ExplorationMap(Rectangle(0, 0, 20, 16), t['options'])
    observed = m.mask(Rectangle(0, 0, 6, 6))
    m.seen[observed] = True; m.evidence[observed] = -2
    team = ['robot_'+str(i) for i in range(1, 7)]
    assigned = m.region.partition(team, 'robot_1')
    assert not assigned.contains(0, 0)
    path, ratio, count, target = m.plan((0, 0, 0), assigned, set())
    assert path and count > 0 and ratio == 0
    assert m.position(*target)[0] < 0


def test_continuous_plan_retains_useful_viewpoint_and_rejects_observed_frontier():
    options = task()['options']; options['frontier_switch_score_ratio'] = 1000.0
    m = ExplorationMap(Rectangle(0, 0, 12, 12), options)
    observed = m.mask(Rectangle(0, 0, 6, 6))
    m.seen[observed] = True; m.evidence[observed] = -2
    current = m.cell(1.0, 0.0)
    path, _, count, target = m.plan((0, 0, 0), m.region, set(), current_target=current)
    assert count > 0 and path and target == current
    m.seen[m.roi] = True; m.evidence[m.roi] = -2
    assert m.plan((0, 0, 0), m.region, set(), current_target=current)[3] is None


def test_small_reachable_island_uses_short_progress_instead_of_no_frontier():
    options = task()['options']
    options.update(resolution_m=.1, robot_radius_m=.1, min_goal_distance_m=.8,
                   min_progress_distance_m=.2)
    m = ExplorationMap(Rectangle(0, 0, 10, 10), options)
    observed = m.mask(Rectangle(0, 0, .8, .8))
    m.seen[observed] = True; m.evidence[observed] = -2
    path, _, count, target = m.plan((0, 0, 0), m.region, set())
    assert len(path) >= 2 and count > 0
    assert .2 <= math.hypot(*m.position(*target)) < .8
    assert m.plan_diagnostics['used_short_step']
    assert m.path_is_valid((0, 0, 0), path)


def test_farther_viewpoint_is_preferred_when_available():
    m = ExplorationMap(Rectangle(0, 0, 12, 12), task()['options'])
    observed = m.mask(Rectangle(0, 0, 6, 6))
    m.seen[observed] = True; m.evidence[observed] = -2
    path, _, _, target = m.plan((0, 0, 0), m.region, set())
    assert path and math.hypot(*m.position(*target)) >= m.opt['min_goal_distance_m']
    assert not m.plan_diagnostics['used_short_step']


def test_ground_observation_marks_only_measured_column_and_low_hit_still_wins():
    options = task()['options']; options.update(min_obstacle_height_m=.05, max_obstacle_height_m=.35, free_space_height_m=.3)
    m = ExplorationMap(Rectangle(0, 0, 10, 10), options)
    m.integrate((0, 0, .4), [(1.0, 0, 0.0)])
    x, y = m.cell(1.0, 0)
    assert m.planar()[y, x] == 0
    assert np.count_nonzero(m.planar() == -1) > m.w*m.h//2
    m.integrate((0, 0, .4), [(1.0, 0, 0.0), (1.0, 0, .2)])
    assert m.planar()[y, x] == 100


def test_observed_area_history_survives_current_occupancy_uncertainty():
    m = ExplorationMap(Rectangle(0, 0, 10, 10), task()['options'])
    x, y = m.cell(1, 0)
    m.evidence[y, x, 0] = 3; m.seen[y, x, 0] = True
    observed = m.area_metrics(m.region)['searched_area_m2']
    m.evidence[y, x, 0] = 0
    assert m.area_metrics(m.region)['searched_area_m2'] >= observed


def test_revisit_penalty_prefers_available_detour_without_crossing_obstacles():
    m = ExplorationMap(Rectangle(0, 0, 10, 10), task()['options'])
    free = np.ones((m.h, m.w), dtype=bool)
    start = (10, 10); goal = (20, 10)
    m.visits[10, 12:19] = 5
    _, parent = m.travel_cost(free, start)
    path = reconstruct(parent, start, goal)
    assert path and all(not (y == 10 and 12 <= x < 19) for x, y in path)


def test_remaining_path_revalidation_ignores_passed_cells_but_blocks_new_obstacle():
    m = ExplorationMap(Rectangle(0, 0, 12, 12), task()['options'])
    m.seen[m.roi] = True; m.evidence[m.roi] = -2
    path = [(-2.0, 0.0), (-1.0, 0.0), (0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    pose = (0.0, 0.0, 0.0)
    assert m.path_is_valid(pose, path)
    x, y = m.cell(-2.0, 0.0); m.evidence[y, x] = 3
    assert m.path_is_valid(pose, path)
    x, y = m.cell(1.0, 0.0); m.evidence[y, x] = 3
    assert not m.path_is_valid(pose, path)


@pytest.mark.parametrize('local,expected', [
    ((-14, 2), (-10, 2)), ((14, 11), (10, 8)),
    ((0, 0), (0, 0)), ((-10, -8), (-10, -8)),
])
def test_nearest_region_point_handles_rotated_edges_and_corners(local, expected):
    region = Rectangle(4, -2, 20, 16, math.pi/3)
    point = region.closest_point(*region.world(*local))
    assert point == pytest.approx(region.world(*expected))


def test_near_boundary_start_can_plan_inward_without_removing_obstacles():
    t = task(); region = Rectangle(0, 0, 12, 10)
    m = ExplorationMap(region, t['options'])
    observed = m.mask(Rectangle(-4, 0, 4, 6))
    m.seen[observed] = True; m.evidence[observed] = -2
    pose = (-5.9, 0, 0)  # only 0.1 m from edge; no startup margin required
    path, _, count, target = m.plan(pose, region, set())
    assert path and count > 0
    assert region.contains(*m.position(*target), t['options']['robot_radius_m'])
    assert all(region.contains(x, y) for x, y in path)
    # The same starting cell occupied by an obstacle must still block motion.
    cx, cy = m.cell(*pose[:2]); m.evidence[cy, cx] = 3
    assert not m.plan(pose, region, set())[0]


@pytest.mark.parametrize('local', [(-6, 0), (6, 0), (0, -5), (0, 5), (-6, -5), (6, 5)])
def test_exact_edge_start_uses_observed_inner_cell(local):
    t = task(); region = Rectangle(0, 0, 12, 10)
    m = ExplorationMap(region, t['options'])
    yy, xx = np.indices((m.h, m.w))
    wx = m.ox+(xx+.5)*m.res; wy = m.oy+(yy+.5)*m.res
    observed = ((wx-local[0])**2 + (wy-local[1])**2 < 3**2) & m.roi
    m.seen[observed] = True; m.evidence[observed] = -2
    path, _, count, target = m.plan((*local, 0), region, set())
    assert path and count > 0
    assert region.contains(*m.position(*target), t['options']['robot_radius_m'])


@pytest.mark.parametrize('ground', [0.0, -1.2, 3.0])
def test_overhead_hits_do_not_leak_into_rounded_top_voxel(ground):
    options = task()['options']
    options.update(ground_z_m=ground, max_obstacle_height_m=0.60)
    grid = ExplorationMap(Rectangle(0, 0, 12, 10), options)
    # ceil allocates a top voxel extending to 0.65 m. Its extra storage must
    # never turn an overhead hit at 0.625 m into a body-height obstacle.
    grid.integrate((0, 0, ground+0.5), [(2, 0, ground+0.625)])
    assert not np.any(grid.planar() == 100)
    # Lower beams and bridge supports remain obstacles.
    grid.integrate((0, 0, ground+0.5), [(2, 0, ground+0.55)])
    cx, cy = grid.cell(2, 0)
    assert grid.planar()[cy, cx] == 100


@pytest.mark.parametrize('reverse', [False, True])
def test_lowest_surface_preserves_low_beams_and_bridge_supports(reverse):
    options = task()['options']
    grid = ExplorationMap(Rectangle(0, 0, 12, 10), options)
    endpoints = [(2, 0, .6), (2, 0, .7),  # bridge deck with clearance
                 (2, 1, .6), (2, 1, .2),  # connected support/low obstruction
                 (2, 2, .35)]             # threshold itself is still blocked
    grid.integrate((0, 0, .3), endpoints[::-1] if reverse else endpoints)
    for y, expected in [(0, 0), (1, 100), (2, 100)]:
        cx, cy = grid.cell(2, y)
        assert grid.planar()[cy, cx] == expected
    # Only observed columns become known; the space behind a deck is unknown.
    cx, cy = grid.cell(3, 0)
    assert grid.planar()[cy, cx] == -1


def test_high_only_scan_does_not_erase_previously_seen_low_obstacle():
    grid = ExplorationMap(Rectangle(0, 0, 12, 10), task()['options'])
    grid.integrate((0, 0, .3), [(2, 0, .2)])
    grid.integrate((0, 0, .3), [(2, 0, .8)])
    cx, cy = grid.cell(2, 0)
    assert grid.planar()[cy, cx] == 100


def test_ray_downsampling_never_discards_a_lowest_obstacle_hit():
    options = task()['options']; options['max_cloud_points'] = 1
    grid = ExplorationMap(Rectangle(0, 0, 12, 10), options)
    grid.integrate((0, 0, .3), [(2, 0, .8), (2, 0, .2), (2, 1, .8)])
    cx, cy = grid.cell(2, 0)
    assert grid.planar()[cy, cx] == 100


def test_navigation_and_search_clearance_bands_are_valid():
    nav = Path(__file__).resolve().parents[3]/'robot_launch/config/go2w_nav2.yaml'
    params = yaml.safe_load(nav.read_text())['local_costmap']['local_costmap']['ros__parameters']
    source = params['obstacle_layer']['pointcloud']
    options = task()['options']
    assert 0 <= source['min_obstacle_height'] < source['max_obstacle_height']
    assert 0 <= options['min_obstacle_height_m'] < options['max_obstacle_height_m']


def test_search_boundary_is_absent_from_navigation_costmap():
    nav = Path(__file__).resolve().parents[3]/'robot_launch/config/go2w_nav2.yaml'
    params = yaml.safe_load(nav.read_text())['local_costmap']['local_costmap']['ros__parameters']
    assert params['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert 'area_search_boundary' not in params


@pytest.mark.parametrize('degrees', [-25.0, 0.0, 25.0])
def test_imu_ground_tilt_uses_height_normal_to_slope(degrees):
    slope = math.radians(degrees)
    for height, expected in [(0.0, None), (.2, 100), (.6, 0)]:
        grid = ExplorationMap(Rectangle(0, 0, 12, 10), task()['options'])
        origin = (0, 0, .3/math.cos(slope))
        point = (2, 0, -2*math.tan(slope)+height/math.cos(slope))
        grid.integrate(origin, [point], origin_height=.3, endpoint_heights=[height])
        cx, cy = grid.cell(2, 0)
        if expected is None:
            assert grid.planar()[cy, cx] != 100  # sloped ground is not an obstacle
        else:
            assert grid.planar()[cy, cx] == expected


def test_ground_offset_is_measured_in_body_frame():
    options = task()['options']; options['ground_z_m'] = -.2
    grid = ExplorationMap(Rectangle(0, 0, 12, 10), options)
    # Body origin stands 0.2 m above terrain; this hit is 0.2 m above ground.
    grid.integrate((0, 0, 8), [(2, 0, 7)], origin_height=.1, endpoint_heights=[0.0])
    cx, cy = grid.cell(2, 0)
    assert grid.planar()[cy, cx] == 100
