"""Camera/lidar geometry and stable IDs for simulation observations."""
import math
import numpy as np


def transform_matrix(transform):
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError('invalid TF quaternion')
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = [t.x, t.y, t.z]
    if not np.isfinite(matrix).all():
        raise ValueError('nonfinite TF')
    return matrix


def apply_transform(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def project(points, camera):
    """Project optical-frame XYZ into the raw image, including distortion."""
    k = np.asarray(camera.k, dtype=float).reshape(3, 3)
    d = np.asarray(camera.d, dtype=float)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('camera intrinsics are missing or invalid')
    if camera.binning_x > 1 or camera.binning_y > 1 or camera.roi.x_offset or camera.roi.y_offset:
        raise ValueError('binned/cropped camera calibration is unsupported')
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if not np.isfinite(points).all() or np.any(points[:, 2] <= 0):
        raise ValueError('camera points must be finite and in front of the camera')
    x, y = points[:, 0]/points[:, 2], points[:, 1]/points[:, 2]
    r2 = x*x+y*y
    if camera.distortion_model == 'equidistant':
        if len(d) != 4:
            raise ValueError('equidistant calibration requires four distortion coefficients')
        theta = np.arctan(np.sqrt(r2))
        t2 = theta*theta
        theta_d = theta*(1+d[0]*t2+d[1]*t2**2+d[2]*t2**3+d[3]*t2**4)
        scale = np.divide(theta_d, np.sqrt(r2), out=np.ones_like(theta), where=r2>1e-12)
        x, y = x*scale, y*scale
    elif camera.distortion_model in ('', 'plumb_bob', 'rational_polynomial'):
        if len(d) not in (0, 4, 5, 8):
            raise ValueError('unsupported raw-camera distortion coefficient count')
        coeff = np.zeros(8)
        coeff[:len(d)] = d
        k1,k2,p1,p2,k3,k4,k5,k6 = coeff
        numerator = 1+k1*r2+k2*r2**2+k3*r2**3
        denominator = 1+k4*r2+k5*r2**2+k6*r2**3
        if np.any(np.abs(denominator)<1e-12):
            raise ValueError('singular distortion model')
        radial = numerator/denominator
        x, y = (x*radial+2*p1*x*y+p2*(r2+2*x*x),
                y*radial+p1*(r2+2*y*y)+2*p2*x*y)
    else:
        raise ValueError('unsupported camera distortion model')
    return np.column_stack((k[0,0]*x+k[0,1]*y+k[0,2], k[1,0]*x+k[1,1]*y+k[1,2]))


def box_position(box, uv, camera_points, map_points, min_points=3, max_depth_spread=.5):
    """Use measured returns near box center; reject sparse or mixed-depth ROIs."""
    inside = ((uv[:, 0] >= box.x_min) & (uv[:, 0] <= box.x_max) &
              (uv[:, 1] >= box.y_min) & (uv[:, 1] <= box.y_max))
    indices = np.flatnonzero(inside)
    if len(indices) < min_points:
        return None
    cx, cy = (box.x_min+box.x_max)/2., (box.y_min+box.y_max)/2.
    scales = max(1., box.x_max-box.x_min), max(1., box.y_max-box.y_min)
    scores = ((uv[indices, 0]-cx)/scales[0])**2+((uv[indices, 1]-cy)/scales[1])**2
    indices = indices[np.argsort(scores)[:30]]
    depths = camera_points[indices, 2]
    median = np.median(depths)
    indices = indices[np.abs(depths-median) <= max_depth_spread]
    if len(indices) < min_points:
        return None
    return np.median(map_points[indices], axis=0)


def enu_to_gps(point, origin):
    east, north, up = point
    lat0, lon0, alt0 = origin
    radius = math.hypot(east, north)
    if radius < 1e-9:
        return float(lat0), float(lon0), float(alt0+up)
    angle = radius/6378137.
    lat0, lon0 = math.radians(lat0), math.radians(lon0)
    lat = math.asin(math.cos(angle)*math.sin(lat0)+north*math.sin(angle)*math.cos(lat0)/radius)
    lon = lon0+math.atan2(east*math.sin(angle), radius*math.cos(lat0)*math.cos(angle)-north*math.sin(lat0)*math.sin(angle))
    return math.degrees(lat), (math.degrees(lon)+180.)%360.-180., float(alt0+up)


class Tracks:
    """One-to-one nearest-position association, with monotonically allocated IDs."""
    def __init__(self, radius=1.):
        self.radius = radius
        self.next_id = 1
        self.items = {}

    def update(self, observations, now):
        assigned = {}
        candidates = []
        for index, (kind, point) in enumerate(observations):
            for target_id, track in self.items.items():
                if track['kind'] == kind:
                    distance = float(np.linalg.norm(point-track['point']))
                    if distance <= self.radius:
                        candidates.append((distance, index, target_id))
        used = set()
        for _, index, target_id in sorted(candidates):
            if index not in assigned and target_id not in used:
                assigned[index] = target_id
                used.add(target_id)
        result = []
        for index, (kind, point) in enumerate(observations):
            target_id = assigned.get(index)
            if target_id is None:
                target_id = self.next_id
                self.next_id += 1
            self.items[target_id] = dict(kind=kind, point=np.asarray(point), stamp=now)
            result.append(target_id)
        return result
