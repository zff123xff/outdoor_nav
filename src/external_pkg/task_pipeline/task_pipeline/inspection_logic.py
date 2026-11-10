"""Geometry and atomic assignment validation for observation-only target visits."""
import math


def angle_error(desired, actual):
    return math.atan2(math.sin(desired-actual), math.cos(desired-actual))


def validate_batch(message):
    batch = {}
    for item in message.targets:
        key = item.target_id
        point = item.target_position
        xyz = (point.x, point.y, point.z)
        if (isinstance(key, bool) or not isinstance(key, int) or not 0 < key < 2**64 or
                key in batch or not all(math.isfinite(v) for v in xyz)):
            raise ValueError('target IDs must be positive and unique; ENU coordinates must be finite')
        batch[key] = xyz
    return batch


def nearest(pose, pending):
    return min(pending, key=lambda key: (math.dist(pose[:3], pending[key]), key))


def visit_pose(robot, target, stand_off=9., max_range=10.):
    """Approach only when outside range; keep a margin for navigation tolerance."""
    dx, dy, dz = (target[i]-robot[i] for i in range(3))
    horizontal = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if horizontal > 1e-6 else robot[3]
    if math.dist(robot[:3], target) <= max_range:
        return (*robot[:3], yaw)
    if abs(dz) >= max_range:
        raise ValueError('target height cannot be reached within inspection range')
    radius = max(stand_off, (abs(dz) + max_range)/2.)
    if radius >= max_range or horizontal < 1e-6:
        raise ValueError('no feasible horizontal inspection pose')
    travel = max(0., horizontal - math.sqrt(radius*radius-dz*dz))
    return (robot[0]+travel*dx/horizontal, robot[1]+travel*dy/horizontal, robot[2], yaw)
