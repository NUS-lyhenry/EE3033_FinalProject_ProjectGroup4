"""Shared circular footprint and continuous swept-motion checks, without ROS.

Planning reserves must not be substituted for the physical footprint. Coordinates
are transformed using the occupancy grid's origin, including its yaw. Obstacles
are closed cell squares, rather than occupied cell centres.
"""
from dataclasses import dataclass
import math
import numpy as np

from .robot_geometry import PHYSICAL_RADIUS, FOOTPRINT_PADDING, FOOTPRINT_RADIUS
OCCUPIED_THRESHOLD = 25


@dataclass(frozen=True)
class MotionCheck:
    allowed: bool
    reason: str


def limit_twist(linear, angular, max_linear, max_angular):
    """Apply one positive scale to both components, preserving signed curvature."""
    if (not all(math.isfinite(v) for v in (linear, angular, max_linear, max_angular))
            or max_linear <= 0 or max_angular <= 0):
        raise ValueError('Invalid velocity or speed limits')
    scale = min(1., max_linear / abs(linear) if linear else 1.,
                max_angular / abs(angular) if angular else 1.)
    return linear * scale, angular * scale, scale


def swept_arc(linear, angular, horizon, step=.025, pose=(0., 0., 0.)):
    """Analytic unicycle trajectory sampled by centre travel, including t=0."""
    if (len(pose) != 3 or not all(math.isfinite(v) for v in (linear, angular, horizon, step, *pose))
            or horizon < 0 or step <= 0):
        raise ValueError('Invalid motion geometry')
    count = max(1, math.ceil(abs(linear) * horizon / step))
    t = np.linspace(0., horizon, count + 1)
    if abs(angular) < 1e-9:
        x, y = linear * t, np.zeros_like(t)
    else:
        x = linear / angular * np.sin(angular * t)
        y = linear / angular * (1. - np.cos(angular * t))
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.column_stack((pose[0] + c*x - s*y, pose[1] + s*x + c*y,
                            pose[2] + angular*t))


def _sample_polyline(points, step):
    points = np.asarray(points, dtype=float)
    if (points.ndim != 2 or points.shape[1] < 2 or not len(points)
            or not np.isfinite(points).all() or not math.isfinite(step) or step <= 0):
        raise ValueError('Invalid path')
    chunks = [points[:1, :2]]
    for a, b in zip(points[:-1, :2], points[1:, :2]):
        count = max(1, math.ceil(float(np.linalg.norm(b-a)) / step))
        chunks.append(np.linspace(a, b, count + 1)[1:])
    return np.concatenate(chunks)


def polyline_map_clearance(grid, resolution, origin, points, radius=FOOTPRINT_RADIUS,
                           occupied_threshold=OCCUPIED_THRESHOLD):
    """Check every square touched by a swept disc; unknown/outside fail closed.

    Samples are at most half a cell apart. Half the actual sample spacing is
    added to the disc so no unsampled connector can jump through a thin wall.
    """
    try:
        grid = np.asarray(grid)
        if (grid.ndim != 2 or not grid.size or resolution <= 0 or radius <= 0
                or not all(math.isfinite(v) for v in (resolution, radius, *origin))
                or not np.isfinite(grid).all() or np.any((grid < -1) | (grid > 100))):
            return MotionCheck(False, 'invalid_map_geometry')
        points = _sample_polyline(points, resolution / 2.)
        ox, oy, yaw = origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = points[:, 0] - ox, points[:, 1] - oy
        local = np.column_stack((c*dx+s*dy, -s*dx+c*dy))
        spacing = float(np.max(np.linalg.norm(np.diff(points, axis=0), axis=1))) if len(points)>1 else 0.
        envelope = radius + spacing / 2.
        height, width = grid.shape
        for x, y in local:
            if x-envelope < 0 or y-envelope < 0 or x+envelope > width*resolution or y+envelope > height*resolution:
                return MotionCheck(False, 'swept_footprint_outside_map')
            c0, c1 = math.floor((x-envelope)/resolution), math.floor((x+envelope)/resolution)
            r0, r1 = math.floor((y-envelope)/resolution), math.floor((y+envelope)/resolution)
            c0,c1=max(0,c0),min(width-1,c1);r0,r1=max(0,r0),min(height-1,r1)
            rr, cc = np.mgrid[r0:r1+1, c0:c1+1]
            # Exact point-to-axis-aligned-square distance in the map frame.
            ex = np.maximum(np.maximum(cc*resolution-x, x-(cc+1)*resolution), 0.)
            ey = np.maximum(np.maximum(rr*resolution-y, y-(rr+1)*resolution), 0.)
            values = grid[r0:r1+1, c0:c1+1][ex*ex+ey*ey <= envelope*envelope+1e-12]
            if np.any(values < 0):
                return MotionCheck(False, 'swept_footprint_unknown')
            if np.any(values >= occupied_threshold):
                return MotionCheck(False, 'swept_footprint_obstacle')
        return MotionCheck(True, 'swept_map_clear')
    except (ValueError, TypeError, IndexError, OverflowError):
        return MotionCheck(False, 'invalid_map_geometry')


def swept_map_clearance(grid, resolution, origin, pose, linear, angular, horizon,
                        radius=FOOTPRINT_RADIUS):
    try:
        arc = swept_arc(linear, angular, horizon, resolution/4., pose)
        return polyline_map_clearance(grid, resolution, origin, arc[:, :2], radius)
    except (ValueError, TypeError, OverflowError):
        return MotionCheck(False, 'invalid_motion_geometry')


def swept_scan_clearance(points, linear, angular, horizon, radius=FOOTPRINT_RADIUS,
                         step=.01):
    """Collision-check valid laser hit endpoints already transformed to base.

    Absence of returns cannot establish free space: the independent known-map
    sweep supplies that proof, particularly for bounded reversing.
    """
    try:
        points = np.asarray(points, dtype=float)
        if points.size == 0:
            return MotionCheck(True, 'no_laser_hits')
        if (points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all()
                or not math.isfinite(radius) or radius <= 0):
            return MotionCheck(False, 'invalid_laser_geometry')
        arc = swept_arc(linear, angular, horizon, step)
        a, b = arc[:-1, :2], arc[1:, :2]
        d = b-a
        length = np.sum(d*d, axis=1)
        delta = points[:, None, :] - a[None, :, :]
        along = np.clip(np.sum(delta*d[None, :, :], axis=2) / np.maximum(length, 1e-20), 0., 1.)
        distance2 = np.sum((delta-along[:, :, None]*d[None, :, :])**2, axis=2)
        # Arc/chord separation is bounded by half centre travel per sample.
        allowance = float(np.max(np.sqrt(length)))/2.
        if np.any(distance2 <= (radius+allowance)**2):
            return MotionCheck(False, 'swept_laser_obstacle')
        return MotionCheck(True, 'swept_laser_clear')
    except (ValueError, TypeError, IndexError, OverflowError):
        return MotionCheck(False, 'invalid_laser_geometry')
