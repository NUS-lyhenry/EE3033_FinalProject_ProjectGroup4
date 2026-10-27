"""ROS-independent geometry and temporal target confirmation."""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np


def angle_difference(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def quaternion_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def rotation_matrix(x, y, z, w):
    q = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid transform quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


@dataclass(frozen=True)
class Observation:
    stamp: float
    x: float
    y: float
    confidence: float
    label: str
    sigma: float = 0.08


class TargetTracker:
    """Consecutive-frame association plus a robust median position estimate.

    An absent frame, timestamp reversal, long gap, class change, or position jump
    breaks the run. Large jumps never extend an existing confirmation streak.
    """
    def __init__(self, frames=4, confidence=0.6, association_radius=0.30,
                 max_gap=0.8, window=9, max_spread=0.12):
        if frames < 2 or window < frames or not 0 < confidence <= 1:
            raise ValueError('Invalid confirmation parameters')
        self.frames = frames
        self.confidence = confidence
        self.radius = association_radius
        self.max_gap = max_gap
        self.max_spread = max_spread
        self.samples = deque(maxlen=window)

    def reset(self):
        self.samples.clear()

    def defer(self, stamp):
        """A visible target during bounded motion is neither a hit nor a miss."""
        if self.samples and (not math.isfinite(stamp) or stamp < self.samples[-1].stamp
                             or stamp-self.samples[-1].stamp > self.max_gap):
            self.reset()

    def update(self, observation):
        if observation is None:
            self.reset()
            return None
        o = observation
        if (not all(math.isfinite(v) for v in
                    (o.stamp, o.x, o.y, o.confidence, o.sigma))
                or not self.confidence <= o.confidence <= 1.0 or o.sigma <= 0):
            self.reset()
            return None
        if self.samples:
            prev = self.samples[-1]
            # Duplicate image messages cannot count as additional frames.
            if o.stamp == prev.stamp:
                return None
            center = np.median([(s.x, s.y) for s in self.samples], axis=0)
            if (o.stamp < prev.stamp or o.stamp - prev.stamp > self.max_gap
                    or o.label != prev.label
                    or np.linalg.norm(np.array([o.x, o.y]) - center) > self.radius):
                self.reset()
        self.samples.append(o)
        if len(self.samples) < self.frames:
            return None
        points = np.array([(s.x, s.y) for s in self.samples])
        center = np.median(points, axis=0)
        residual = np.linalg.norm(points - center, axis=1)
        if np.max(residual) > self.max_spread:
            return None
        # Keep a sensor uncertainty floor; repeated observations are correlated.
        sigma = max(float(np.median([s.sigma for s in self.samples])),
                    float(np.sqrt(np.mean(residual ** 2))))
        return {'x': float(center[0]), 'y': float(center[1]),
                'variance': sigma ** 2, 'frames': len(self.samples),
                'confidence': float(np.mean([s.confidence for s in self.samples])),
                'label': o.label, 'stamp': o.stamp}


def associate_lidar(camera_points, laser_points, scan_indices, bbox, camera_matrix,
                    distortion, min_points=2, cluster_gap=0.12, max_width=0.50):
    """Project lidar into the image and select a compact central foreground cluster.

    Returns a point in the laser frame. Both horizontal and vertical bbox bounds
    must agree; a lidar plane above a short object is deliberately rejected.
    """
    import cv2
    if len(camera_points) == 0:
        return None
    front = camera_points[:, 2] > 0.08
    ids = np.flatnonzero(front)
    if len(ids) == 0:
        return None
    pixels, _ = cv2.projectPoints(camera_points[ids], np.zeros(3), np.zeros(3),
                                  camera_matrix, distortion)
    pixels = pixels.reshape(-1, 2)
    x1, y1, x2, y2 = bbox
    width = x2 - x1
    inside = ((pixels[:, 0] >= x1 + 0.15 * width)
              & (pixels[:, 0] <= x2 - 0.15 * width)
              & (pixels[:, 1] >= y1) & (pixels[:, 1] <= y2))
    ids = ids[inside]
    if len(ids) < min_points:
        return None
    clusters, current = [], [int(ids[0])]
    for idx in ids[1:]:
        prev = current[-1]
        if (int(scan_indices[idx]) - int(scan_indices[prev]) > 2
                or np.linalg.norm(laser_points[idx] - laser_points[prev]) > cluster_gap):
            clusters.append(current)
            current = []
        current.append(int(idx))
    clusters.append(current)
    options = []
    for c in clusters:
        if len(c) < min_points:
            continue
        p = np.median(laser_points[c], axis=0)
        if np.max(np.linalg.norm(laser_points[c] - p, axis=1)) * 2 > max_width:
            continue
        # Favor the nearest compact cluster; background walls are still a known
        # limitation when the object is transparent or does not intersect lidar.
        options.append((float(np.median(camera_points[c, 2])), p))
    return min(options, key=lambda item: item[0])[1] if options else None


def ground_intersection(ray, rotation, translation, ground_z=0.0, max_range=3.0):
    direction = rotation @ np.asarray(ray, dtype=float)
    origin = np.asarray(translation, dtype=float)
    if direction[2] >= -1e-5:
        return None
    distance = (ground_z - origin[2]) / direction[2]
    point = origin + distance * direction
    if distance <= 0 or np.linalg.norm(point[:2] - origin[:2]) > max_range:
        return None
    return point
