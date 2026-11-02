"""ROI coverage and observed transit space are separate constraints."""
from pathlib import Path
import math
import numpy as np
import pytest
import yaml
from task_pipeline.area_search_core import ExplorationMap, Rectangle, reconstruct, validate_task


def make_map(margin=1.0, yaw=0.0):
    config = Path(__file__).parents[1]/'config/area_search.yaml'
    options = validate_task(yaml.safe_load(config.read_text()))['options']
    options.update(resolution_m=.2, robot_radius_m=.1, outside_margin_m=margin)
    return ExplorationMap(Rectangle(0, 0, 4, 4, yaw), options)


def mark_free(m, mask):
    m.seen[mask] = True; m.evidence[mask] = -2


@pytest.mark.parametrize('yaw', [0., math.pi/4])
def test_observed_outer_band_allows_detour_around_wall_crossing_roi(yaw):
    m = make_map(yaw=yaw)
    mark_free(m, m.mapping_roi)
    yy, xx = np.indices((m.h, m.w))
    dx = m.ox+(xx+.5)*m.res; dy = m.oy+(yy+.5)*m.res
    local_x = math.cos(yaw)*dx+math.sin(yaw)*dy
    local_y = -math.sin(yaw)*dx+math.cos(yaw)*dy
    wall = m.roi & (np.abs(local_x) < .15)
    m.evidence[wall] = 3
    pose = (*m.region.world(-1, 0), yaw)
    goal = m.cell(*m.region.world(1, 0))
    free, inside = m.navigation_masks(m.planar(), pose)
    start = m.cell(*pose[:2]); distance, parent = m.travel_cost(free, start)
    path = reconstruct(parent, start, goal)
    assert distance[goal[1], goal[0]] > 0 and path
    assert inside[goal[1], goal[0]]
    assert any(not m.roi[y, x] for x, y in path)
    assert all(free[y, x] and m.mapping_roi[y, x] for x, y in path)
    assert m.path_is_valid(pose, [m.position(x, y) for x, y in path])
    # Forcing the same graph inside the ROI makes the wall impassable.
    restricted, _ = m.travel_cost(free & m.roi, start)
    assert restricted[goal[1], goal[0]] < 0


def test_unobserved_outer_band_never_becomes_a_shortcut():
    m = make_map(); mark_free(m, m.roi)
    grid = m.planar(); free, _ = m.navigation_masks(grid, (0, 0, 0))
    assert not np.any(free & ~m.roi)
    assert np.all(grid[m.mapping_roi & ~m.roi] == -1)


def test_obstacles_outside_roi_are_retained_for_collision_checks():
    m = make_map(); mark_free(m, m.mapping_roi)
    x, y = m.cell(2.5, 0)
    m.evidence[y, x] = 3
    grid = m.planar(); free, _ = m.navigation_masks(grid, (0, 0, 0))
    assert not m.roi[y, x] and grid[y, x] == 100 and not free[y, x]
    assert not m.path_is_valid((1.5, 0, 0), [(1.5, 0), (2.5, 0)])


def test_real_ground_hit_updates_outer_band_without_expanding_coverage_denominator():
    m = make_map(); m.integrate((0, 0, .3), [(2.5, 0, 0)])
    x, y = m.cell(2.5, 0)
    assert not m.roi[y, x] and m.planar()[y, x] == 0
    mark_free(m, m.roi)
    before = m.area_metrics(m.region)
    mark_free(m, m.mapping_roi)
    after = m.area_metrics(m.region)
    assert before == after
    assert after['assigned_area_m2'] == pytest.approx(16.)


def test_planning_uses_actual_outside_pose_and_keeps_goal_inside_roi():
    m = make_map(); mark_free(m, m.mapping_roi)
    x, y = m.cell(1.1, 0)
    m.seen[y-2:y+3, x:x+2] = False; m.evidence[y-2:y+3, x:x+2] = 0
    pose = (2.4, 0, math.pi)
    path, ratio, clusters, target = m.plan(pose, m.region, {})
    assert path and clusters and target is not None
    assert math.dist(path[0], pose[:2]) < m.res
    assert m.region.contains(*m.position(*target))
    assert m.path_is_valid(pose, path)


def test_zero_margin_disables_outer_transit_and_negative_margin_is_rejected():
    m = make_map(margin=0.0); mark_free(m, m.mapping_roi)
    free, _ = m.navigation_masks(m.planar(), (0, 0, 0))
    assert not np.any(free & ~m.roi)
    config = yaml.safe_load((Path(__file__).parents[1]/'config/area_search.yaml').read_text())
    config['options']['outside_margin_m'] = -1
    with pytest.raises(ValueError):
        validate_task(config)
