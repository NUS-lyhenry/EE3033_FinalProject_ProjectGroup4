"""One range anchor, timestamped relative poses, and held-out camera views.

Odometry supplies continuous metric displacement; the saved map/localization
checks the route. The bottle stays fixed. Its bearing and camera height are
estimated, so the operator does not measure offsets or camera mounting height.
"""
import math
from datetime import datetime, timezone

import numpy as np
from scipy.optimize import least_squares

from .calibration import (number, validate_setup, validate_sample, observed_points,
                          predicted_points, floor_point)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def relative_pose(origin, pose):
    dx, dy = pose[0]-origin[0], pose[1]-origin[1]
    c, s = math.cos(origin[2]), math.sin(origin[2])
    return [c*dx+s*dy, -s*dx+c*dy, wrap(pose[2]-origin[2])]


def bottle_sample(setup, distance, bearing, pose, box):
    target = np.array([setup['camera_x']+distance*math.cos(bearing),
                       setup['camera_y']+distance*math.sin(bearing)])
    dx, dy = target-np.array(pose[:2])
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return dict(distance=float(c*dx+s*dy-setup['camera_x']),
                lateral=float(-s*dx+c*dy-setup['camera_y']), box=list(box))


def fit_automatic(setup, camera, anchor_range, observations):
    """Fit training images only; reserve two route positions for floor checks."""
    setup = validate_setup(setup)
    anchor_range = number(anchor_range, 'Measured floor distance (m)', .45, 2.5)
    if len(observations) < 12:
        raise ValueError('Automatic collection needs at least twelve views')
    views = []
    for view in observations:
        pose = np.asarray(view['pose'], dtype=float)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise ValueError('Invalid timestamped camera pose')
        validate_sample(dict(distance=1., lateral=0., box=view['box']), camera)
        views.append({**view, 'pose': pose.tolist()})
    training = [v for v in views if not v.get('holdout')]
    checks = [v for v in views if v.get('holdout')]
    if len(training) < 10 or len(checks) < 2:
        raise ValueError('Training and two held-out positions are required')
    xy = np.array([v['pose'][:2] for v in training])
    if max(np.linalg.norm(a-b) for a in xy for b in xy) < .6:
        raise ValueError('Automatic route did not provide 0.60 m of translation')
    if np.ptp([v['pose'][2] for v in training]) < .2:
        raise ValueError('Automatic route needs different viewing angles')
    if any(min(np.linalg.norm(np.array(v['pose'][:2])-np.array(t['pose'][:2]))
               for t in training) < .065 for v in checks):
        raise ValueError('Held-out positions are too close to training positions')
    w, h = camera['width'], camera['height']
    weights = np.array([[1., 1.], [1., 1.], [.35, 0.], [.35, 0.]])

    def samples(parameters, selected):
        return [bottle_sample(setup, anchor_range, parameters[5], v['pose'], v['box']) for v in selected]

    def residual(parameters):
        geometry = {**setup, 'camera_height': parameters[4]}
        return np.concatenate([((predicted_points(parameters[:4], geometry, camera, s)
                                 -observed_points(s['box']))*weights).ravel()
                               for s in samples(parameters, training)])

    scale = np.array([w, h, 1., 1., .3, 1.])
    bounds = ([w*.2, h*.2, -.5, -.4, .05, -.65],
              [w*4., h*5., 1., .4, 1., .65])
    initial = np.array([camera['k'][0], camera['k'][4], .1, 0., setup['camera_height'], 0.])
    initial = np.clip(initial, np.array(bounds[0])+1e-5, np.array(bounds[1])-1e-5)
    result = least_squares(residual, initial, bounds=bounds, loss='soft_l1', f_scale=2.,
                           x_scale=scale, max_nfev=1200)
    fitted_setup = {**setup, 'camera_height': float(result.x[4])}
    train_samples = samples(result.x, training)
    errors = np.array([np.linalg.norm(predicted_points(result.x[:4], fitted_setup, camera, s)[:2]
                                     -observed_points(s['box'])[:2], axis=1).max() for s in train_samples])
    rms = float(np.sqrt(np.mean(errors**2)))
    condition = float(np.linalg.cond(result.jac*scale))
    if (not result.success or np.any(result.active_mask) or not math.isfinite(condition)
            or condition > 1e4 or rms > 8 or errors.max() > 16):
        raise ValueError(f'Automatic fit rejected ({rms:.1f} px RMS); keep the bottle fixed and check the measured distance')
    matrix = list(camera['k'])
    matrix[0], matrix[4] = float(result.x[0]), float(result.x[1])
    distances = [s['distance'] for s in train_samples]
    bearings = [math.atan2(s['lateral'], s['distance']) for s in train_samples]
    profile = dict(schema=1, kind='measured_object_floor',
        created_utc=datetime.now(timezone.utc).isoformat(), camera={**camera, 'k': matrix},
        setup=fitted_setup, pitch=float(result.x[2]), yaw=float(result.x[3]),
        base_frame='base_footprint', training=train_samples, validation=[], validated=False,
        range_m=[min(distances), max(distances)], bearing_rad=[min(bearings), max(bearings)],
        quality=dict(reprojection_rms_px=rms, max_error_px=float(errors.max()), condition=condition),
        automatic=dict(method='one_range_pose_bundle', anchor_range_m=anchor_range,
            anchor_bearing_rad=float(result.x[5]), observations=views,
            pose_source='timestamped odometry, guarded by map localization',
            validation_source='held-out pose-derived positions; not independent surveyed ground truth'))
    for sample in samples(result.x, checks):
        x1, _, x2, y2 = sample['box']
        point = floor_point(profile, [(x1+x2)/2, y2])
        expected = np.array([sample['distance']+setup['camera_x'], sample['lateral']+setup['camera_y']])
        error = float(np.linalg.norm(point[:2]-expected))
        profile['validation'].append({**sample, 'estimated_base_xy': point[:2].tolist(),
                                      'error_m': error, 'passed': error <= .10})
    if not all(v['passed'] for v in profile['validation']):
        raise ValueError('Automatic floor checks failed the 0.10 m limit; profile was not saved')
    return profile


def corridor_clear(grid, origin, heading, length=.78, radius=.24):
    """Reject unknown, occupied or out-of-map cells in the whole swept corridor."""
    if grid is None or grid.header.frame_id != 'map' or grid.info.resolution <= 0:
        return False
    info = grid.info
    q = info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    step = min(info.resolution/2, .025)
    for along in np.arange(-radius, length+radius+step, step):
        for across in np.arange(-radius, radius+step, step):
            x = origin[0]-along*math.cos(heading)-across*math.sin(heading)-info.origin.position.x
            y = origin[1]-along*math.sin(heading)+across*math.cos(heading)-info.origin.position.y
            col = math.floor((math.cos(yaw)*x+math.sin(yaw)*y)/info.resolution)
            row = math.floor((-math.sin(yaw)*x+math.cos(yaw)*y)/info.resolution)
            if (not 0 <= col < info.width or not 0 <= row < info.height
                    or grid.data[row*info.width+col] != 0):
                return False
    return True


class AutomaticRoute:
    """Small straight corridor; centre + yaw views, then two unseen distances.

    tick returns velocities and a capture request. The node owns heartbeat,
    sensor/map checks, exact image/pose pairing, fitting, and final stop gate.
    """
    def __init__(self, now, pose):
        self.started = self.changed = now
        self.origin = list(pose)
        self.heading = pose[2]
        self.phase = 'align'
        self.index = 0
        self.stable_since = None
        self.goals = [(d, a, False) for d in (0., .18, .36, .54, .72) for a in (0., -.13, .13)]
        self.goals += [(.45, 0., True), (.27, 0., True)]
        self.reason = 'Automatically centring the bottle'

    def tick(self, now, pose, error, capture_ready):
        if now-self.started > 210 or now-self.changed > 35:
            raise ValueError('Automatic movement timed out; check clearance and bottle visibility')
        if self.phase == 'align':
            if math.hypot(pose[0]-self.origin[0], pose[1]-self.origin[1]) > .12:
                raise ValueError('Robot drifted while centring the bottle')
            if abs(wrap(pose[2]-self.origin[2])) > .6:
                raise ValueError('Bottle is too far off centre; place it ahead of the camera')
            if abs(error) > .018:
                self.stable_since = None
                return 0., -math.copysign(min(.15, max(.055, abs(error)*.6)), error), False
            if self.stable_since is None:
                self.stable_since = now
            if now-self.stable_since < .4 or not capture_ready:
                return 0., 0., False
            self.heading = pose[2]
            self.origin = list(pose)
            self.phase, self.changed = 'translate', now
            self.stable_since = None
        if self.index >= len(self.goals):
            self.phase = 'done'
            return 0., 0., False
        distance, yaw, holdout = self.goals[self.index]
        dx, dy = pose[0]-self.origin[0], pose[1]-self.origin[1]
        progress = -(dx*math.cos(self.heading)+dy*math.sin(self.heading))
        lateral = -dx*math.sin(self.heading)+dy*math.cos(self.heading)
        if abs(lateral) > .08 or math.hypot(dx, dy) > .85:
            raise ValueError('Robot drifted outside the calibration corridor')
        if self.phase == 'translate':
            angle = wrap(self.heading-pose[2])
            self.reason = f'Moving to view {self.index+1}/{len(self.goals)}'
            if abs(angle) > .025:
                return 0., math.copysign(min(.12, max(.055, abs(angle)*.8)), angle), False
            delta = distance-progress
            if abs(delta) > .012:
                return -math.copysign(.035, delta), angle*.6, False
            self.phase, self.changed = 'turn', now
        angle = wrap(self.heading+yaw-pose[2])
        if abs(angle) > .018:
            self.stable_since = None
            return 0., math.copysign(min(.12, max(.055, abs(angle)*.8)), angle), False
        if self.stable_since is None:
            self.stable_since = now
        self.reason = f'Capturing {"check" if holdout else "training"} view {self.index+1}/{len(self.goals)}'
        return 0., 0., now-self.stable_since >= .4 and capture_ready

    def captured(self, now):
        self.index += 1
        self.changed, self.stable_since = now, None
        self.phase = 'translate'


class FreshDataPause:
    """Always command zero during a brief data gap, then require a settled view.

    This is a collection pause, not a motion lease extension. Ownership,
    heartbeat, localization jumps and confirmed target loss remain hard stops.
    """
    def __init__(self):
        self.since = None

    def waiting(self, now, missing, settled):
        if missing and self.since is None:
            self.since = now
        if self.since is None:
            return False
        if now-self.since > 2.:
            raise ValueError('Fresh sensor/detection data did not recover within two seconds')
        if not missing and settled:
            self.since = None
            return False
        return True
