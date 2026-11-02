from pathlib import Path
import numpy as np
import pytest
import yaml
from task_pipeline.area_search_core import enclosed_unknown_mask, ExplorationMap, Rectangle, validate_task


def ring():
    g = np.zeros((9, 9), dtype=np.int8)
    g[2:7, 2:7] = 100
    g[3:6, 3:6] = -1
    return g


def test_closed_obstacle_ring_fills_only_unknown_interior():
    g = ring(); mask = enclosed_unknown_mask(g, np.ones_like(g, bool))
    assert mask.sum() == 9
    assert not np.any(mask & (g != -1))


@pytest.mark.parametrize('opening', [(2, 4), (2, 2)])
def test_free_or_diagonal_opening_keeps_hole_unknown(opening):
    g = ring(); g[opening] = 0
    assert not enclosed_unknown_mask(g, np.ones_like(g, bool)).any()


def test_boundary_assisted_inference_can_be_disabled():
    g = ring(); roi = np.ones_like(g, bool); roi[3, 3] = False
    assert not enclosed_unknown_mask(g, roi, include_boundary=False).any()


def test_obstacle_and_region_edge_enclose_unknown_pocket():
    g = np.zeros((9, 9), dtype=np.int8)
    roi = np.zeros_like(g, bool); roi[1:8, 1:8] = True
    g[1:8, 5] = 100; g[1:8, 6:8] = -1
    filled = enclosed_unknown_mask(g, roi)
    assert filled.sum() == 14
    assert np.all(filled[1:8, 6:8])
    assert not np.any(filled & ~roi)
    assert not enclosed_unknown_mask(g, roi, include_boundary=False).any()
    g[4, 5] = 0  # an opening to measured free space prevents filling
    assert not enclosed_unknown_mask(g, roi).any()


def test_boundary_alone_or_sparse_initial_hits_cannot_fill_unobserved_rectangle():
    g = np.full((9, 9), -1, dtype=np.int8); roi = np.ones_like(g, bool)
    assert not enclosed_unknown_mask(g, roi).any()
    g[0, 4] = 100
    assert not enclosed_unknown_mask(g, roi).any()


def test_free_space_inside_obstacle_ring_is_preserved():
    g = ring(); g[4, 4] = 0
    assert not enclosed_unknown_mask(g, np.ones_like(g, bool)).any()


def map_with_obstacle_ring():
    task = validate_task(yaml.safe_load((Path(__file__).parents[1]/'config/area_search.yaml').read_text()))
    task['options']['resolution_m'] = .2
    task['options']['outside_margin_m'] = 0.0
    m = ExplorationMap(Rectangle(0, 0, 4, 4), task['options'])
    m.seen[m.roi] = True; m.evidence[m.roi] = -2
    m.seen[6:11, 6:11] = False; m.evidence[6:11, 6:11] = 0
    m.seen[5:12, 5:12, 0] = True; m.evidence[5:12, 5:12, 0] = 3
    m.seen[6:11, 6:11] = False; m.evidence[6:11, 6:11] = 0
    return m


def test_filled_cells_count_in_coverage_without_double_counting_direct_observations():
    m = map_with_obstacle_ring()
    assert np.all(m.planar()[6:11, 6:11] == 100)
    assert not m.observed_ever[6:11, 6:11].any()
    metrics = m.area_metrics(m.region)
    assert metrics['inferred_obstacle_area_m2'] == pytest.approx(25*m.res**2)
    assert metrics['observed_ratio'] == 1.0
    assert metrics['searched_area_m2'] == pytest.approx(metrics['direct_observed_area_m2']+metrics['inferred_obstacle_area_m2'])
    # A later real observation replaces inference, rather than being counted twice.
    m.seen[8, 8] = True; m.evidence[8, 8] = 3
    updated = m.area_metrics(m.region)
    assert updated['searched_area_m2'] == pytest.approx(metrics['searched_area_m2'])
    assert updated['inferred_obstacle_area_m2'] == pytest.approx(24*m.res**2)
    assert m.plan((0, 0, 0), m.region, {})[1] == 1.0


def test_opening_ring_recomputes_fill_and_does_not_leave_phantom_obstacles():
    m = map_with_obstacle_ring(); m.planar()
    m.seen[5, 8] = True; m.evidence[5, 8] = -2
    assert np.all(m.planar()[6:11, 6:11] == -1)
    assert not m.inferred_obstacles.any()


def test_option_can_disable_enclosure_inference():
    m = map_with_obstacle_ring(); m.opt['fill_enclosed_unknown'] = False
    assert np.all(m.planar()[6:11, 6:11] == -1)
    assert m.area_metrics(m.region)['inferred_obstacle_area_m2'] == 0


def test_boundary_filled_pocket_counts_once_and_can_be_reopened():
    m = map_with_obstacle_ring()
    m.seen[m.roi] = True; m.evidence[m.roi] = -2
    yy, xx = np.indices(m.roi.shape)
    cx, _ = m.cell(0, 0)
    wall = m.roi & (xx == cx); pocket = m.roi & (xx > cx)
    m.evidence[wall] = 3; m.seen[pocket] = False; m.evidence[pocket] = 0
    metrics = m.area_metrics(m.region)
    assert metrics['observed_ratio'] == 1.
    assert metrics['inferred_obstacle_area_m2'] == pytest.approx(pocket.sum()*m.res**2)
    assert metrics['searched_area_m2'] == pytest.approx(metrics['direct_observed_area_m2']+metrics['inferred_obstacle_area_m2'])
    # A real opening to the observed side invalidates boundary-assisted inference.
    _, cy = m.cell(0, 0)
    m.evidence[cy, cx] = -2
    reopened = m.area_metrics(m.region)
    assert reopened['observed_ratio'] < 1.
    assert not m.inferred_obstacles.any()
