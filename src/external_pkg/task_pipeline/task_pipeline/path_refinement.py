"""Collision-constrained 2D adaptation of co-search's path-to-spline stages.

The output is a geometric path for Nav2, not a timed aerial trajectory. Unknown
space is blocked. Every shortcut and spline subdivision is checked before use.
"""
import math
import numpy as np


def segment_is_free(a, b, free, cell, resolution):
    previous = None
    for t in np.linspace(0., 1., max(2, math.ceil(math.dist(a, b)/(resolution/3))+1)):
        x, y = cell(a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
        if not (0 <= y < free.shape[0] and 0 <= x < free.shape[1] and free[y, x]):
            return False
        if previous is not None:
            px, py = previous
            if px != x and py != y and not (free[py, x] and free[y, px]):
                return False
        previous = x, y
    return True


def refine_path(points, free, cell, resolution, weight, iterations=2, max_segment=1.5):
    """Shorten a guide path, then perform checked quadratic B-spline subdivision.

    Weighted route cost protects the upstream revisit/outside penalties. A spline
    update that crosses an obstacle, unknown cell or blocked corner is rejected;
    the last valid guide path is retained. Endpoints are fixed throughout.
    """
    points = [tuple(p) for i, p in enumerate(points) if i == 0 or math.dist(p, points[i-1]) > 1e-8]
    if len(points) < 2:
        return points

    def safe(path):
        return all(segment_is_free(a, b, free, cell, resolution) for a, b in zip(path, path[1:]))

    def cost(path):
        total = 0.
        for a, b in zip(path, path[1:]):
            length = math.dist(a, b); count = max(1, math.ceil(length/(resolution/2)))
            for i in range(count):
                t = (i+.5)/count
                x, y = cell(a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
                total += length/count*weight(x, y)
        return total

    if not safe(points):
        return points  # caller retains the original path/failure checks
    short = [points[0]]; i = 0
    while i < len(points)-1:
        # Bound work and segment length; don't shortcut away a cost-avoiding detour.
        chosen = i+1
        for j in range(i+2, len(points)):
            if math.dist(points[i], points[j]) > max_segment:
                break
            if (segment_is_free(points[i], points[j], free, cell, resolution) and
                    cost([points[i], points[j]]) <= cost(points[i:j+1])*1.02):
                chosen = j
        short.append(points[chosen]); i = chosen

    current = np.array(short, dtype=float)
    for _ in range(iterations):
        # Chaikin corner cutting converges to a quadratic uniform B-spline.
        # Add fixed endpoints so spline smoothing cannot move a task target.
        proposed = [current[0]]
        for a, b in zip(current, current[1:]):
            proposed.extend((.75*a+.25*b, .25*a+.75*b))
        proposed.append(current[-1]); proposed = np.array(proposed)
        if not safe(proposed) or cost(proposed) > cost(current)*1.02:
            break
        current = proposed

    # Uniform-distance sampling keeps the pure-pursuit lookahead independent of
    # the density of control points and preserves the exact final endpoint.
    result = [tuple(current[0])]
    for a, b in zip(current, current[1:]):
        count = max(1, math.ceil(math.dist(a, b)/resolution))
        result.extend(tuple(a+(b-a)*i/count) for i in range(1, count+1))
    return result
