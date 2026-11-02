"""Robust local ground plane from measured returns, constrained by IMU tilt."""
import math
import numpy as np


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def fit_ground_plane(points, body_position, imu_normal, max_offset=.35,
                     max_tilt_deg=20.0, residual=.035):
    p = np.asarray(points)-np.asarray(body_position)
    radius = np.linalg.norm(p[:, :2], axis=1)
    signed = p @ np.asarray(imu_normal)
    p = p[(radius > .8) & (radius < 4.0) & (np.abs(signed) < max_offset)]
    if len(p) < 40:
        return None
    # Equalize dense scan directions before fitting; rooftops outside the
    # IMU-constrained ground band cannot win the ground hypothesis.
    keys = np.floor(p[:, :2]/.15).astype(int)
    _, indices = np.unique(keys, axis=0, return_index=True)
    p = p[indices]
    if len(p) < 25:
        return None
    design = np.column_stack((p[:, :2], np.ones(len(p))))
    rng = np.random.default_rng(0)
    best = None; best_count = 0
    normal_prior = np.asarray(imu_normal)/np.linalg.norm(imu_normal)
    for _ in range(45):
        sample = rng.choice(len(p), 3, replace=False)
        if abs(np.linalg.det(design[sample])) < .01:
            continue
        coefficients = np.linalg.solve(design[sample], p[sample, 2])
        normal = np.array([-coefficients[0], -coefficients[1], 1.0])
        normal /= np.linalg.norm(normal)
        if np.dot(normal, normal_prior) < math.cos(math.radians(max_tilt_deg)):
            continue
        if abs(coefficients[2]) > max_offset:
            continue
        inliers = np.abs(design @ coefficients-p[:, 2]) < residual
        count = int(inliers.sum())
        if count > best_count:
            best_count = count; best = inliers
    if best is None or best_count < max(25, int(.45*len(p))):
        return None
    coefficients = np.linalg.lstsq(design[best], p[best, 2], rcond=None)[0]
    a, b, c = coefficients
    normal = np.array([-a, -b, 1.0]); normal /= np.linalg.norm(normal)
    if np.dot(normal, normal_prior) < math.cos(math.radians(max_tilt_deg)) or abs(c) > max_offset:
        return None
    return np.array([a, b, c+body_position[2]-a*body_position[0]-b*body_position[1]])
