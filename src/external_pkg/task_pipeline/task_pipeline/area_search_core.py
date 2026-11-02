"""Rectangle frontier exploration; pure geometry/map/search code without ROS.

Like co-search: known-free/unknown frontiers -> observation points -> 8-neighbour
weighted graph search. This is not precomputed lawnmower coverage or a trained SNN.
"""
from collections import deque
from dataclasses import dataclass
import hashlib
import heapq
import json
import math
import re

import numpy as np
from .path_refinement import refine_path

DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def number(value, name, minimum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    value = float(value)
    if minimum is not None and value < minimum:
        raise ValueError(f'{name} must be >= {minimum}')
    return value


def validate_task(data):
    if not isinstance(data, dict):
        raise ValueError('area search YAML must be a mapping')
    result = json.loads(json.dumps(data, allow_nan=False))
    ns = str(result.get('robot_namespace', '')).strip('/')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', ns):
        raise ValueError('robot_namespace must be a single non-empty ROS namespace')
    result['robot_namespace'] = ns
    mode = result.get('coordinate_mode', 'enu')
    if mode not in ('enu', 'gps'):
        raise ValueError('coordinate_mode must be gps or enu')
    result['coordinate_mode'] = mode
    for value in result.get('topics', {}).values():
        if not isinstance(value, str) or not value:
            raise ValueError('topics must be non-empty ROS topic names')
    rect = result.setdefault('rectangle', {})
    for name in ('length_m', 'width_m'):
        rect[name] = number(rect.get(name), 'rectangle.' + name, 1.0)
    rect['yaw_deg'] = number(rect.get('yaw_deg', 0.0), 'rectangle.yaw_deg')
    if mode == 'enu':
        point = result.get('center_enu', {})
        for name in ('east', 'north'):
            number(point.get(name), 'center_enu.' + name)
    else:
        point = result.get('center_gps', {})
        lat = number(point.get('latitude'), 'center_gps.latitude')
        lon = number(point.get('longitude'), 'center_gps.longitude')
        number(point.get('altitude', 0.0), 'center_gps.altitude')
        if abs(lat) > 90 or abs(lon) > 180:
            raise ValueError('GPS latitude/longitude out of range')
    options = result.setdefault('options', {})
    for key in ('use_imu_ground_tilt', 'fill_enclosed_unknown', 'fill_boundary_enclosed_unknown', 'refine_path'):
        if not isinstance(options.get(key, True), bool):
            raise ValueError('options.'+key+' must be boolean')
        options.setdefault(key, True)
    defaults = {
        'resolution_m': 0.2, 'robot_radius_m': 0.65,
        'sensor_range_m': 5.0, 'ground_z_m': 0.0,
        'min_obstacle_height_m': 0.05, 'max_obstacle_height_m': 0.35,
        'free_space_height_m': 0.30,
        'min_frontier_cells': 3, 'min_goal_distance_m': 0.8,
        'min_progress_distance_m': 0.2,
        'viewpoint_distance_m': 1.2, 'planning_period_s': 0.5,
        'map_update_period_s': 0.2, 'max_cloud_points': 1200,
        'path_refresh_distance_m': 1.0,
        'outside_margin_m': 2.0, 'outside_cost_weight': 2.0,
        'path_spline_iterations': 2, 'path_shortcut_max_segment_m': 1.5,
        'local_path_length_m': 4.0, 'goal_timeout_s': 90.0,
        'heartbeat_timeout_s': 4.0, 'team_stable_s': 3.0,
        'team_timeout_s': 60.0, 'sensor_timeout_s': 3.0,
        'completion_observed_ratio': 0.90, 'no_frontier_confirmations': 16,
        'max_path_failures': 5, 'frontier_cooldown_s': 30.0,
        'frontier_switch_score_ratio': 1.3, 'frontier_switch_distance_m': 1.0,
        'revisit_cost_weight': 1.5, 'heading_cost_weight': 1.5,
    }
    options = result.setdefault('options', {})
    for key, default in defaults.items():
        minimum = None if key == 'ground_z_m' else 0.0
        options[key] = number(options.get(key, default), 'options.' + key, minimum)
    for key in ('resolution_m', 'sensor_range_m', 'planning_period_s',
                'map_update_period_s', 'local_path_length_m', 'goal_timeout_s',
                'heartbeat_timeout_s', 'team_stable_s', 'team_timeout_s', 'sensor_timeout_s'):
        if options[key] <= 0:
            raise ValueError('options.' + key + ' must be positive')
    for key in ('min_frontier_cells', 'max_cloud_points', 'no_frontier_confirmations', 'max_path_failures', 'path_spline_iterations'):
        if options[key] != int(options[key]) or options[key] < 1:
            raise ValueError('options.' + key + ' must be a positive integer')
        options[key] = int(options[key])
    if options['path_spline_iterations'] > 5 or options['path_shortcut_max_segment_m'] <= 0:
        raise ValueError('path refinement requires 1..5 iterations and a positive shortcut length')
    if not 0 < options['completion_observed_ratio'] <= 1:
        raise ValueError('completion_observed_ratio must be in (0, 1]')
    if options['frontier_switch_score_ratio'] <= 1:
        raise ValueError('frontier_switch_score_ratio must exceed 1')
    if not 0 < options['min_progress_distance_m'] <= options['min_goal_distance_m']:
        raise ValueError('min_progress_distance_m must be positive and <= min_goal_distance_m')
    if options['max_obstacle_height_m'] <= options['min_obstacle_height_m']:
        raise ValueError('obstacle height band is empty')
    if not options['min_obstacle_height_m'] < options['free_space_height_m'] <= options['max_obstacle_height_m']:
        raise ValueError('free_space_height_m must lie in the obstacle height band')
    if rect['length_m'] * rect['width_m'] / options['resolution_m'] ** 2 > 500_000:
        raise ValueError('rectangle/resolution exceeds 500000 planar cells')
    if min(rect['length_m'], rect['width_m']) <= 2 * options['robot_radius_m']:
        raise ValueError('rectangle is narrower than the robot safety footprint')
    result['mission_id'] = str(result.get('mission_id', 'rectangle_search'))
    if not result['mission_id']:
        raise ValueError('mission_id must not be empty')
    return result


def natural_key(namespace):
    return tuple((1, int(t)) if t.isdigit() else (0, t) for t in re.split(r'(\d+)', namespace))


def gps_to_enu(lat, lon, alt, origin):
    """Same spherical azimuthal-equidistant conversion as nav2_input_bridge."""
    lat, lon, lat0, lon0 = map(math.radians, (lat, lon, origin[0], origin[1]))
    d = lon - lon0
    arg = max(-1.0, min(1.0, math.sin(lat0) * math.sin(lat) + math.cos(lat0) * math.cos(lat) * math.cos(d)))
    c = math.acos(arg)
    k = c / math.sin(c) if abs(c) > 1e-6 else 1.0
    return (6378137.0 * k * math.cos(lat) * math.sin(d),
            6378137.0 * k * (math.cos(lat0) * math.sin(lat) - math.sin(lat0) * math.cos(lat) * math.cos(d)),
            alt - origin[2])


@dataclass(frozen=True)
class Rectangle:
    east: float
    north: float
    length: float
    width: float
    yaw: float = 0.0

    def world(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.east + c*x - s*y, self.north + s*x + c*y

    def local(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x-self.east, y-self.north
        return c*dx+s*dy, -s*dx+c*dy

    def contains(self, x, y, margin=0):
        u, v = self.local(x, y)
        return abs(u) <= self.length/2-margin and abs(v) <= self.width/2-margin

    def closest_point(self, x, y, margin=0.0):
        """Orthogonal projection, including rotated edges and corners."""
        u, v = self.local(x, y)
        half_x = max(0.0, self.length/2-margin)
        half_y = max(0.0, self.width/2-margin)
        return self.world(max(-half_x, min(half_x, u)),
                          max(-half_y, min(half_y, v)))

    def corners(self):
        return [self.world(x, y) for x, y in [(-self.length/2, -self.width/2),
                (self.length/2, -self.width/2), (self.length/2, self.width/2), (-self.length/2, self.width/2)]]

    def partition(self, team, namespace):
        names = sorted(set(team), key=natural_key)
        i = names.index(namespace)
        width = self.length / len(names)
        x, y = self.world(-self.length/2 + (i+0.5)*width, 0)
        return Rectangle(x, y, width, self.width, self.yaw)


def rectangle_for_task(task, origin):
    if task['coordinate_mode'] == 'gps':
        p = task['center_gps']
        x, y, _ = gps_to_enu(p['latitude'], p['longitude'], p.get('altitude', origin[2]), origin)
    else:
        x, y = task['center_enu']['east'], task['center_enu']['north']
    r = task['rectangle']
    return Rectangle(x, y, r['length_m'], r['width_m'], math.radians(r['yaw_deg']))


def task_signature(task, origin, team):
    # Ignore dormant coordinate block and per-robot routing; compare the actual mission.
    region = rectangle_for_task(task, origin)
    payload = [task['mission_id'], region.__dict__, list(origin), task['options'], sorted(team, key=natural_key)]
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def line_cells(a, b):
    x, y = a; tx, ty = b
    dx, dy = abs(tx-x), abs(ty-y)
    sx, sy = (1 if x < tx else -1), (1 if y < ty else -1)
    err = dx-dy
    while True:
        yield x, y
        if x == tx and y == ty:
            return
        e = 2*err
        if e > -dy:
            err -= dy; x += sx
        if e < dx:
            err += dx; y += sy


def bfs(free, start):
    """8-neighbour BFS; diagonal corner cutting forbidden, co-search local rule."""
    h, w = free.shape
    distance = np.full((h, w), -1, dtype=np.int32)
    parent = {}
    x, y = start
    if not (0 <= x < w and 0 <= y < h and free[y, x]):
        return distance, parent
    queue = deque([start]); distance[y, x] = 0
    while queue:
        x, y = queue.popleft()
        for dx, dy in DIRS:
            nx, ny = x+dx, y+dy
            if not (0 <= nx < w and 0 <= ny < h) or not free[ny, nx] or distance[ny, nx] >= 0:
                continue
            if dx and dy and not (free[y, nx] and free[ny, x]):
                continue
            distance[ny, nx] = distance[y, x]+1
            parent[(nx, ny)] = (x, y)
            queue.append((nx, ny))
    return distance, parent


def reconstruct(parent, start, goal):
    path = [goal]
    while path[-1] != start:
        if path[-1] not in parent:
            return []
        path.append(parent[path[-1]])
    return path[::-1]


def inflate(blocked, radius):
    out = blocked.copy()
    h, w = blocked.shape
    for dy in range(-radius, radius+1):
        for dx in range(-radius, radius+1):
            if dx*dx+dy*dy > radius*radius:
                continue
            ya, yb = max(0, dy), min(h, h+dy)
            xa, xb = max(0, dx), min(w, w+dx)
            if ya < yb and xa < xb:
                out[ya:yb, xa:xb] |= blocked[ya-dy:yb-dy, xa-dx:xb-dx]
    return out


def enclosed_unknown_mask(grid, roi, include_boundary=True):
    """Infer unknown components enclosed by obstacles and optional ROI edges.

    A free neighbour, including diagonal openings, always prevents filling.
    Boundary-assisted inference needs both obstacle contact and known free
    space elsewhere in the ROI; an unobserved rectangle is never filled.
    """
    unknown = (grid == -1) & roi
    visited = np.zeros_like(unknown)
    enclosed = np.zeros_like(unknown)
    has_free = bool(np.any((grid == 0) & roi))
    h, w = grid.shape
    for y, x in zip(*np.where(unknown)):
        if visited[y, x]:
            continue
        queue = deque([(x, y)]); visited[y, x] = True
        component = []; closed = True; boundary = False; obstacle = False
        while queue:
            cx, cy = queue.popleft(); component.append((cx, cy))
            for dx, dy in DIRS:
                nx, ny = cx+dx, cy+dy
                if not (0 <= nx < w and 0 <= ny < h) or not roi[ny, nx]:
                    boundary = True
                elif grid[ny, nx] == 0:
                    closed = False
                elif grid[ny, nx] == 100:
                    obstacle = True
                elif unknown[ny, nx] and not visited[ny, nx]:
                    visited[ny, nx] = True; queue.append((nx, ny))
        if closed and obstacle and (not boundary or (include_boundary and has_free)):
            for cx, cy in component:
                enclosed[cy, cx] = True
    return enclosed


class ExplorationMap:
    """Persistent 3D ray evidence projected conservatively into a planar map.

    Existing PointCloud2 supplies HIT rays only; missing returns are NOT assumed
    free. Ground hits clear the observed height band but do not become obstacles.
    """
    def __init__(self, region, options):
        self.region = region; self.opt = options; self.res = options['resolution_m']
        margin = options.get('outside_margin_m', 2.0)
        self.mapping_region = Rectangle(region.east, region.north,
                                        region.length+2*margin, region.width+2*margin, region.yaw)
        corners = self.mapping_region.corners()
        self.ox = min(p[0] for p in corners)-self.res
        self.oy = min(p[1] for p in corners)-self.res
        self.w = math.ceil((max(p[0] for p in corners)-self.ox)/self.res)+1
        self.h = math.ceil((max(p[1] for p in corners)-self.oy)/self.res)+1
        self.z0 = options['ground_z_m']+options['min_obstacle_height_m']
        self.z_max = options['ground_z_m']+options['max_obstacle_height_m']
        self.nz = math.ceil((options['max_obstacle_height_m']-options['min_obstacle_height_m'])/self.res)
        if self.w*self.h*self.nz > 5_000_000:
            raise ValueError('3D observation grid exceeds 5000000 voxels')
        self.evidence = np.zeros((self.h, self.w, self.nz), dtype=np.int16)
        self.seen = np.zeros_like(self.evidence, dtype=bool)
        self.roi = self.mask(region)
        self.mapping_roi = self.mask(self.mapping_region)
        self.updates = 0
        self.observed_ever = np.zeros((self.h, self.w), dtype=bool)
        self.inferred_obstacles = np.zeros((self.h, self.w), dtype=bool)
        self._enclosure_key = None
        self.visits = np.zeros((self.h, self.w), dtype=np.uint16)
        self.last_visit = None

    def cell(self, x, y):
        return math.floor((x-self.ox)/self.res), math.floor((y-self.oy)/self.res)

    def position(self, x, y):
        return self.ox+(x+0.5)*self.res, self.oy+(y+0.5)*self.res

    def mask(self, rect):
        yy, xx = np.indices((self.h, self.w))
        dx = self.ox+(xx+0.5)*self.res-rect.east
        dy = self.oy+(yy+0.5)*self.res-rect.north
        c, s = math.cos(rect.yaw), math.sin(rect.yaw)
        return (np.abs(c*dx+s*dy) < rect.length/2) & (np.abs(-s*dx+c*dy) < rect.width/2)

    def seed_footprint(self, x, y, yaw):
        cell = self.cell(x, y)
        if cell != self.last_visit:
            cx, cy = cell
            if 0 <= cx < self.w and 0 <= cy < self.h:
                self.visits[cy, cx] = min(100, int(self.visits[cy, cx])+1)
            self.last_visit = cell
        # Known physical footprint only, never an arbitrary sensor-range free disk.
        yy, xx = np.indices((self.h, self.w))
        dx = self.ox+(xx+0.5)*self.res-x; dy = self.oy+(yy+0.5)*self.res-y
        c, s = math.cos(yaw), math.sin(yaw)
        body = (np.abs(c*dx+s*dy) <= .40) & (np.abs(-s*dx+c*dy) <= .23) & self.mapping_roi
        unknown = body[:, :, None] & ~self.seen
        self.evidence[unknown] = -1; self.seen[unknown] = True

    def integrate(self, origin, endpoints, origin_height=None, endpoint_heights=None):
        ox, oy, oz = origin
        if origin_height is None:
            origin_height = oz
        if endpoint_heights is None:
            endpoint_heights = [p[2] for p in endpoints]
        lowest_hits = {}
        ground_hits = set()
        ray_stride = max(1, math.ceil(len(endpoints)/self.opt['max_cloud_points']))
        for index, (px, py, pz) in enumerate(endpoints):
            hit_height = endpoint_heights[index]
            length = math.dist(origin, (px, py, pz))
            if not (0.05 < length <= self.opt['sensor_range_m']+0.1):
                continue
            hit_x, hit_y = self.cell(px, py)
            if (self.opt['ground_z_m']-.10 <= hit_height and 0 <= hit_x < self.w and 0 <= hit_y < self.h
                    and self.mapping_roi[hit_y, hit_x]):
                key = (hit_y, hit_x)
                if hit_height < self.z0:
                    ground_hits.add(key)
                else:
                    lowest_hits[key] = min(lowest_hits.get(key, math.inf), hit_height)
            # Limit expensive ray integration, never the lowest-hit decision.
            if index % ray_stride:
                continue
            n = max(2, math.ceil(length/self.res*2))
            cells = set()
            for t in np.linspace(0, 1, n, endpoint=False):
                x, y = self.cell(ox+(px-ox)*t, oy+(py-oy)*t)
                height = origin_height+(hit_height-origin_height)*t
                z = math.floor((height-self.z0)/self.res)
                if self.z0 <= height <= self.z_max and 0 <= x < self.w and 0 <= y < self.h and 0 <= z < self.nz and self.mapping_roi[y, x]:
                    cells.add((y, x, z))
            hit_z = math.floor((hit_height-self.z0)/self.res)
            cells.discard((hit_y, hit_x, hit_z))
            for key in cells:
                self.evidence[key] = max(-5, int(self.evidence[key])-1); self.seen[key] = True
        # Project each observed column by its lowest non-ground surface.
        # An overhead-only column has measured clearance. It may establish
        # previously unknown free space, but cannot erase an existing low hit.
        # Low returns in the same scan win regardless of point ordering.
        for y, x in ground_hits-lowest_hits.keys():
            unknown = ~self.seen[y, x]
            self.evidence[y, x, unknown] = -1
            self.seen[y, x, unknown] = True
        for (y, x), lowest in lowest_hits.items():
            if lowest <= self.z_max:
                z = math.floor((lowest-self.z0)/self.res)
                if 0 <= z < self.nz:
                    self.evidence[y, x, z] = min(5, int(self.evidence[y, x, z])+3)
                    self.seen[y, x, z] = True
            else:
                unknown = ~self.seen[y, x]
                self.evidence[y, x, unknown] = -1
                self.seen[y, x, unknown] = True
        self.updates += 1

    def planar(self):
        occupied = np.any(self.evidence >= 2, axis=2)
        # Body-height rays or an observed overhead-only column establish free
        # space. Existing low occupancy always overrides overhead clearance.
        layers = min(self.nz, max(1, math.ceil((self.opt['free_space_height_m']-self.opt['min_obstacle_height_m'])/self.res)))
        free = np.all(self.seen[:, :, :layers] & (self.evidence[:, :, :layers] < 2), axis=2) & ~occupied
        out = np.full((self.h, self.w), -1, dtype=np.int8)
        out[free & self.mapping_roi] = 0; out[occupied & self.mapping_roi] = 100
        # Preserve direct observation history BEFORE adding inferred obstacles.
        self.observed_ever |= out >= 0
        if self.opt.get('fill_enclosed_unknown', True):
            include_boundary = self.opt.get('fill_boundary_enclosed_unknown', True)
            key = (out.tobytes(), include_boundary)
            if key != self._enclosure_key:
                self.inferred_obstacles = enclosed_unknown_mask(out, self.roi, include_boundary)
                self._enclosure_key = key
            out[self.inferred_obstacles] = 100
        else:
            self.inferred_obstacles.fill(False)
            self._enclosure_key = None
        return out

    def area_metrics(self, assigned):
        """Count direct observations and enclosed-obstacle inference once per cell."""
        own = self.mask(assigned) & self.roi
        total_cells = int(np.count_nonzero(own))
        self.planar()
        direct_cells = int(np.count_nonzero(own & self.observed_ever))
        inferred_cells = int(np.count_nonzero(own & self.inferred_obstacles & ~self.observed_ever))
        observed_cells = direct_cells+inferred_cells
        cell_area = self.res * self.res
        return {
            'searched_area_m2': observed_cells * cell_area,
            'assigned_area_m2': total_cells * cell_area,
            'observed_ratio': observed_cells / total_cells if total_cells else 0.0,
            'direct_observed_area_m2': direct_cells*cell_area,
            'inferred_obstacle_area_m2': inferred_cells*cell_area,
        }

    def navigation_masks(self, grid, pose):
        occupied = inflate(grid == 100, math.ceil(self.opt['robot_radius_m']/self.res))
        yy, xx = np.indices(grid.shape)
        wx = self.ox+(xx+.5)*self.res; wy = self.oy+(yy+.5)*self.res
        c, s = math.cos(self.region.yaw), math.sin(self.region.yaw)
        dx, dy = wx-self.region.east, wy-self.region.north
        radius = self.opt['robot_radius_m']
        inside = (np.abs(c*dx+s*dy) <= self.region.length/2-radius) & (np.abs(-s*dx+c*dy) <= self.region.width/2-radius)
        known_free = (grid == 0) & ~occupied & self.mapping_roi
        # Goals stay inside the search ROI; transit can use the observed outer
        # band. Retain footprint clearance at the outer limit, not the ROI edge.
        transit_inside = ((np.abs(c*dx+s*dy) <= self.mapping_region.length/2-radius) &
                          (np.abs(-s*dx+c*dy) <= self.mapping_region.width/2-radius))
        departure = (wx-pose[0])**2 + (wy-pose[1])**2 <= (2*radius+self.res)**2
        free = known_free & (transit_inside | departure)
        return free, inside

    def path_is_valid(self, pose, path):
        if not path:
            return False
        free, _ = self.navigation_masks(self.planar(), pose)
        nearest = min(range(len(path)), key=lambda i: math.dist(pose[:2], path[i]))
        points = [pose[:2]] + list(path[nearest:])
        previous = None
        for a, b in zip(points, points[1:]):
            for x, y in line_cells(self.cell(*a), self.cell(*b)):
                if not (0 <= x < self.w and 0 <= y < self.h and free[y, x]):
                    return False
                if previous is not None:
                    px, py = previous
                    if px != x and py != y and not (free[py, x] and free[y, px]):
                        return False
                previous = (x, y)
        return True

    def plan(self, pose, assigned, blacklist, current_target=None):
        grid = self.planar(); own = self.mask(assigned) & self.roi
        free, inside = self.navigation_masks(grid, pose)
        # Inclusive geometric edges can fall in an outside grid cell. Use the
        # adjacent inner cell, still requiring real free-space evidence there.
        start = self.cell(*pose[:2])
        sx, sy = start
        if (self.region.contains(*pose[:2]) and 0 <= sx < self.w and 0 <= sy < self.h
                and not self.roi[sy, sx]):
            start = self.cell(*self.region.closest_point(pose[0], pose[1], self.res/2))
        transit = not assigned.contains(pose[0], pose[1])
        distance, parent = self.travel_cost(free, start, start_yaw=None if transit else pose[2])
        diagnostics = {
            'start_cell': list(start),
            'reachable_cells': int(np.count_nonzero(distance >= 0)),
            'frontier_cells': 0, 'small_clusters': 0,
            'outside_region': 0, 'unreachable': 0, 'cooldown': 0,
            'too_close': 0, 'not_visible': 0,
            'short_step_candidates': 0, 'used_short_step': False,
        }
        self.plan_diagnostics = diagnostics
        task_mask = self.roi if transit else own
        unknown = (grid == -1) & task_mask
        near_unknown = np.zeros_like(unknown)
        for dx, dy in DIRS[:4]:
            near_unknown |= np.roll(np.roll(unknown, dx, axis=1), dy, axis=0)
        near_unknown[[0, -1], :] = False; near_unknown[:, [0, -1]] = False
        frontier = (grid == 0) & task_mask & near_unknown
        diagnostics['frontier_cells'] = int(np.count_nonzero(frontier))
        visited = set(); clusters = []
        for y, x in zip(*np.where(frontier)):
            if (x, y) in visited:
                continue
            q = deque([(x, y)]); visited.add((x, y)); cluster = []
            while q:
                a, b = q.popleft(); cluster.append((a, b))
                for dx, dy in DIRS:
                    nx, ny = a+dx, b+dy
                    if 0 <= nx < self.w and 0 <= ny < self.h and frontier[ny, nx] and (nx, ny) not in visited:
                        visited.add((nx, ny)); q.append((nx, ny))
            if len(cluster) >= self.opt['min_frontier_cells']:
                clusters.append(cluster)
            else:
                diagnostics['small_clusters'] += 1
        best = None; best_score = -math.inf; current_score = None
        short_best = None; short_score = -math.inf
        for cluster in clusters:
            center = np.mean(cluster, axis=0)
            cx, cy = self.position(center[0], center[1])
            candidates = list(cluster[::max(1, len(cluster)//8)])
            if current_target is not None:
                candidates.append(current_target)
            for angle in np.linspace(0, 2*math.pi, 16, endpoint=False):
                candidates.append(self.cell(cx+self.opt['viewpoint_distance_m']*math.cos(angle), cy+self.opt['viewpoint_distance_m']*math.sin(angle)))
            for gx, gy in candidates:
                if not (0 <= gx < self.w and 0 <= gy < self.h and task_mask[gy, gx]) or not inside[gy, gx]:
                    diagnostics['outside_region'] += 1
                    continue
                if distance[gy, gx] < 0:
                    diagnostics['unreachable'] += 1
                    continue
                if (gx, gy) in blacklist:
                    diagnostics['cooldown'] += 1
                    continue
                wx, wy = self.position(gx, gy)
                goal_distance = math.hypot(wx-pose[0], wy-pose[1])
                if (gx, gy) != current_target and goal_distance < self.opt['min_progress_distance_m']:
                    diagnostics['too_close'] += 1
                    continue
                visible = 0
                for target in cluster[::max(1, len(cluster)//30)]:
                    if all(grid[y, x] == 0 for x, y in line_cells((gx, gy), target)):
                        visible += 1
                if not visible:
                    diagnostics['not_visible'] += 1
                    continue
                heading = math.atan2(wy-pose[1], wx-pose[0])
                turn = abs(math.atan2(math.sin(heading-pose[2]), math.cos(heading-pose[2])))
                heading_cost = 0.0 if transit else self.opt['heading_cost_weight']*turn
                score = visible/(1+distance[gy, gx]*self.res+heading_cost)
                if transit:
                    score -= .2*math.hypot(wx-assigned.east, wy-assigned.north)
                    if own[gy, gx]:
                        score += 100.0
                if (gx, gy) != current_target and goal_distance < self.opt['min_goal_distance_m']:
                    diagnostics['short_step_candidates'] += 1
                    if score > short_score:
                        short_score = score; short_best = (gx, gy)
                    continue
                if (gx, gy) == current_target:
                    current_score = max(current_score if current_score is not None else -math.inf, score)
                if score > best_score:
                    best_score = score; best = (gx, gy)
        if best is None and short_best is not None:
            best = short_best; best_score = short_score
            diagnostics['used_short_step'] = True
        if best is None:
            best = self.cleanup_viewpoint(grid, distance, inside, pose, task_mask, blacklist)
            diagnostics['cleanup_viewpoint'] = best is not None
        # Keep a useful current viewpoint unless a distinctly better, separated
        # viewpoint appears. It need not be in this cycle's sampled candidates.
        if current_score is not None and best is not None:
            separation = math.dist(self.position(*best), self.position(*current_target))
            if (best_score < current_score*self.opt['frontier_switch_score_ratio'] or
                    separation < self.opt['frontier_switch_distance_m']):
                best = current_target
        ratio = float(np.count_nonzero((self.observed_ever | self.inferred_obstacles) & own))/max(1, np.count_nonzero(own))
        if best is None:
            return [], ratio, len(clusters), None
        cells = reconstruct(parent, start, best)
        guide = [self.position(*cell) for cell in cells]
        if self.opt.get('refine_path', True):
            guide = refine_path(guide, free, self.cell, self.res,
                                self.cell_travel_weight,
                                iterations=self.opt.get('path_spline_iterations', 2),
                                max_segment=self.opt.get('path_shortcut_max_segment_m', 1.5))
        # Clip the refined path by travelled arc length, not Euclidean radius.
        path = []; length = 0.0
        for p in guide:
            if path:
                length += math.dist(path[-1], p)
                if length > self.opt['local_path_length_m']:
                    break
            path.append(p)
        return path, ratio, len(clusters), best

    def cleanup_viewpoint(self, grid, distance, inside, pose, task_mask, blacklist):
        """Look at remaining unknown cells from reachable, measured free space.

        Handles tiny edge fragments with no usable regular frontier centroid.
        Visibility estimates guide motion; they never mark cells as observed.
        """
        yy, xx = np.where(task_mask & ~self.observed_ever & (grid == -1))
        if not len(xx):
            return None
        targets = list(zip(xx, yy))[::max(1, len(xx)//24)]
        candidates = set()
        for tx, ty in targets:
            wx, wy = self.position(tx, ty)
            for radius in (.6, 1.5, 3.0):
                for angle in np.linspace(0, 2*math.pi, 12, endpoint=False):
                    candidates.add(self.cell(wx+radius*math.cos(angle), wy+radius*math.sin(angle)))
        best = None; best_score = -math.inf
        for gx, gy in candidates:
            if not (0 <= gx < self.w and 0 <= gy < self.h and inside[gy, gx] and
                    task_mask[gy, gx] and distance[gy, gx] >= 0) or (gx, gy) in blacklist:
                continue
            wx, wy = self.position(gx, gy)
            if math.hypot(wx-pose[0], wy-pose[1]) < self.opt['min_progress_distance_m']:
                continue
            visible = sum(1 for tx, ty in targets
                          if math.hypot(tx-gx, ty-gy)*self.res <= self.opt['sensor_range_m'] and
                          all(grid[y, x] != 100 for x, y in line_cells((gx, gy), (tx, ty))))
            if not visible:
                continue
            angle = math.atan2(wy-pose[1], wx-pose[0])-pose[2]
            turn = abs(math.atan2(math.sin(angle), math.cos(angle)))
            score = visible/(1+distance[gy, gx]*self.res+self.opt['heading_cost_weight']*turn)
            if score > best_score:
                best = (gx, gy); best_score = score
        return best

    def cell_travel_weight(self, x, y):
        outside = 0.0 if self.roi[y, x] else self.opt.get('outside_cost_weight', 2.0)
        return 1+outside+self.opt['revisit_cost_weight']*min(5, int(self.visits[y, x]))

    def travel_cost(self, free, start, start_yaw=None):
        """Dijkstra travel cost with a soft penalty on already traversed cells."""
        distance = np.full(free.shape, -1.0); parent = {}
        x, y = start
        if not (0 <= x < self.w and 0 <= y < self.h and free[y, x]):
            return distance, parent
        distance[y, x] = 0.0; queue = [(0.0, x, y)]
        while queue:
            cost, x, y = heapq.heappop(queue)
            if cost > distance[y, x]+1e-9:
                continue
            for dx, dy in DIRS:
                nx, ny = x+dx, y+dy
                if not (0 <= nx < self.w and 0 <= ny < self.h and free[ny, nx]):
                    continue
                if dx and dy and not (free[y, nx] and free[ny, x]):
                    continue
                new_cost = cost+math.hypot(dx, dy)*self.cell_travel_weight(nx, ny)
                if (x, y) == start and start_yaw is not None:
                    angle = math.atan2(dy, dx)-start_yaw
                    turn = abs(math.atan2(math.sin(angle), math.cos(angle)))
                    new_cost += self.opt['heading_cost_weight']*turn/self.res
                if distance[ny, nx] < 0 or new_cost < distance[ny, nx]:
                    distance[ny, nx] = new_cost; parent[(nx, ny)] = (x, y)
                    heapq.heappush(queue, (new_cost, nx, ny))
        return distance, parent
