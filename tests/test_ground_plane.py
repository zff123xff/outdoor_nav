import numpy as np
from robot_launch.ground_plane import fit_ground_plane


def test_imu_constrained_plane_rejects_roof_and_low_obstacle_outliers():
    rng = np.random.default_rng(42)
    xy = rng.uniform(-3, 3, (1200, 2))
    ground = np.column_stack((xy, .04*xy[:, 0]-.03*xy[:, 1]+.13+rng.normal(0, .008, len(xy))))
    obstacles = ground[:250].copy(); obstacles[:, 2] += .17
    roof = ground[:300].copy(); roof[:, 2] += .8
    plane = fit_ground_plane(np.vstack((ground, obstacles, roof)), [0, 0, 0], [0, 0, 1])
    np.testing.assert_allclose(plane, [.04, -.03, .13], atol=.012)


def test_no_ground_returns_never_promote_overhead_structure_to_ground():
    rng = np.random.default_rng(1)
    roof = np.column_stack((rng.uniform(-3, 3, (200, 2)), np.full(200, .7)))
    assert fit_ground_plane(roof, [0, 0, 0], [0, 0, 1]) is None
