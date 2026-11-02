"""Refined paths remain inside measured, inflated free space."""
import math
import numpy as np
from task_pipeline.path_refinement import refine_path, segment_is_free


def cell(x, y):
    return math.floor(x/.1), math.floor(y/.1)


def turn_energy(points):
    v = np.diff(np.asarray(points), axis=0)
    v = v[np.linalg.norm(v, axis=1)>1e-8]
    v /= np.linalg.norm(v, axis=1)[:, None]
    angles = np.arccos(np.clip(np.sum(v[:-1]*v[1:], axis=1), -1, 1))
    return float(np.sum(angles**2))


def test_curve_is_smoothed_densified_and_preserves_endpoints():
    free = np.ones((50, 50), bool)
    guide = [(1., 1.), (1., 2.), (2., 2.), (2., 3.)]
    path = refine_path(guide, free, cell, .1, lambda x, y: 1.)
    assert path[0] == guide[0] and path[-1] == guide[-1]
    assert max(math.dist(a, b) for a, b in zip(path, path[1:])) <= .1000001
    assert turn_energy(path) < turn_energy(guide)
    assert all(segment_is_free(a, b, free, cell, .1) for a, b in zip(path, path[1:]))


def test_spline_cannot_cut_through_obstacle_inside_corner():
    free = np.ones((50, 50), bool); free[15:25, 15:25] = False
    guide = [(1.05, 1.05), (1.05, 3.05), (3.05, 3.05)]
    path = refine_path(guide, free, cell, .1, lambda x, y: 1.)
    assert all(segment_is_free(a, b, free, cell, .1) for a, b in zip(path, path[1:]))
    assert path[0] == guide[0] and path[-1] == guide[-1]


def test_diagonally_touching_obstacles_cannot_be_crossed():
    free = np.ones((10, 10), bool); free[0, 1] = False; free[1, 0] = False
    assert not segment_is_free((.05, .05), (.15, .15), free, cell, .1)


def test_shortcut_preserves_penalized_route_around_visited_area():
    free = np.ones((50, 50), bool)
    guide = [(1.05, 1.05), (1.05, 2.55), (2.55, 2.55), (2.55, 1.05)]
    def weight(x, y):
        return 100. if 12 <= x <= 23 and 9 <= y <= 20 else 1.
    path = refine_path(guide, free, cell, .1, weight)
    assert all(weight(*cell(*p)) == 1. for p in path)


def test_empty_single_duplicate_points_and_straight_line_are_supported():
    free = np.ones((50, 50), bool)
    assert refine_path([], free, cell, .1, lambda x, y: 1.) == []
    assert refine_path([(1., 1.), (1., 1.)], free, cell, .1, lambda x, y: 1.) == [(1., 1.)]
    path = refine_path([(1., 1.), (2., 1.)], free, cell, .1, lambda x, y: 1.)
    assert all(p[1] == 1. for p in path)
