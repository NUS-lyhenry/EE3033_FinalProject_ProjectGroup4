"""Measured-object camera/floor calibration, independent of ROS and the UI.

This estimates effective focal lengths and mounting pitch/yaw. Principal point
and lens distortion are retained from CameraInfo (or explicitly assumed). The
upright bottle is approximated by a vertical centreline and a diameter at its
mid-height; it is not a replacement for checkerboard lens calibration.
"""
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

from .assessment import atomic_json


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be {low}–{high}')
    return float(value)


def mounting_rotation(pitch, yaw):
    """Optical (right, down, forward) to base (forward, left, up)."""
    c, s, a, b = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[a, -b, 0], [b, a, 0], [0, 0, 1.]]) @ np.array(
        [[0, -s, c], [-1, 0, 0], [0, -c, -s]])


def validate_setup(data):
    return {key: number(data.get(key), key, *bounds) for key, bounds in {
        'bottle_height': (.05, 1.), 'bottle_diameter': (.02, .3),
        'camera_height': (.05, 1.), 'camera_x': (-.5, .5), 'camera_y': (-.5, .5),
    }.items()}


def camera_model(width, height, frame_id, k=None, d=None, distortion_model='plumb_bob'):
    if not frame_id:
        raise ValueError('Camera image needs a frame ID')
    calibrated = k is not None and len(k) == 9 and k[0] > 0 and k[4] > 0
    if calibrated:
        if distortion_model not in ('plumb_bob', 'rational_polynomial'):
            raise ValueError('Only plumb_bob / rational_polynomial lens models are supported')
        matrix = np.asarray(k, dtype=float).reshape(3, 3)
        distortion = list(d) if d is not None and len(d) else [0.] * 5
    else:
        matrix = np.array([[width, 0, width / 2], [0, width, height / 2], [0, 0, 1.]])
        distortion = [0.] * 5
    if (not np.isfinite(matrix).all() or not np.isfinite(distortion).all()
            or len(distortion) not in (4, 5, 8, 12, 14)):
        raise ValueError('Invalid camera intrinsics or distortion coefficients')
    return dict(width=int(width), height=int(height), frame_id=frame_id,
                k=matrix.ravel().tolist(), d=distortion,
                source='CameraInfo' if calibrated else 'assumed image centre / zero distortion')


def validate_sample(data, camera):
    box = data.get('box')
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        raise ValueError('Select the complete upright bottle in the image')
    x1, y1, x2, y2 = [number(v, 'box coordinate', 0, max(camera['width'], camera['height'])) for v in box]
    if not (2 <= x1 < x2 <= camera['width']-2 and 2 <= y1 < y2 <= camera['height']-2
            and x2-x1 >= 8 and y2-y1 >= 20):
        raise ValueError('Bottle must be fully visible, away from the image edges, and large enough')
    return dict(distance=number(data.get('distance'), 'forward distance (m)', .3, 4.),
                lateral=number(data.get('lateral', 0.), 'left offset (m)', -2., 2.),
                box=[x1, y1, x2, y2])


def observed_points(box):
    x1, y1, x2, y2 = box
    return np.array([[(x1+x2)/2, y2], [(x1+x2)/2, y1],
                     [x1, (y1+y2)/2], [x2, (y1+y2)/2]])


def predicted_points(parameters, setup, camera, sample):
    fx, fy, pitch, yaw = parameters
    rotation = mounting_rotation(pitch, yaw)
    d, y, h, radius = sample['distance'], sample['lateral'], setup['bottle_height'], setup['bottle_diameter']/2
    # A diameter perpendicular to the horizontal viewing direction approximates
    # the visible silhouette. Height/diameter are required physical measurements.
    side = np.array([-y, d, 0.]) / math.hypot(d, y) * radius
    centre = np.array([d, y, h/2-setup['camera_height']])
    points = np.array([[d, y, -setup['camera_height']],
                       [d, y, h-setup['camera_height']], centre+side, centre-side]) @ rotation
    if np.any(points[:, 2] <= .05):
        return np.full((4, 2), 1e5)
    matrix = np.array(camera['k']).reshape(3, 3).copy()
    matrix[0, 0], matrix[1, 1] = fx, fy
    projected, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3), matrix, np.array(camera['d']))
    return projected.reshape(-1, 2)


def fit_profile(setup, camera, samples):
    setup = validate_setup(setup)
    samples = [validate_sample(s, camera) for s in samples]
    if len(samples) < 6:
        raise ValueError('Capture at least six training positions')
    distances = [s['distance'] for s in samples]
    bearings = [math.atan2(s['lateral'], s['distance']) for s in samples]
    if max(distances)-min(distances) < .6:
        raise ValueError('Training distances must span at least 0.60 m')
    if max(bearings)-min(bearings) < .20:
        raise ValueError('Include measured left/right offsets spanning at least 12 degrees')
    width, height = camera['width'], camera['height']
    weights = np.array([[1., 1.], [1., 1.], [.35, 0.], [.35, 0.]])

    def residual(parameters):
        return np.concatenate([((predicted_points(parameters, setup, camera, s)-observed_points(s['box']))*weights).ravel() for s in samples])

    initial = [camera['k'][0], camera['k'][4], 0., 0.]
    bounds = ([width*.2, height*.2, -.5, -.4], [width*4., height*5., 1., .4])
    initial[:2] = np.clip(initial[:2], np.array(bounds[0])[:2]+1, np.array(bounds[1])[:2]-1)
    result = least_squares(residual, initial, bounds=bounds, loss='soft_l1', f_scale=2.,
                           x_scale=[width, height, 1., 1.], max_nfev=600)
    scaled_jac = result.jac * np.array([width, height, 1., 1.])
    condition = float(np.linalg.cond(scaled_jac))
    errors = np.array([np.linalg.norm(predicted_points(result.x, setup, camera, s)[:2]-observed_points(s['box'])[:2], axis=1).max() for s in samples])
    rms = float(np.sqrt(np.mean(errors**2)))
    if (not result.success or not math.isfinite(condition) or condition > 1e4
            or np.any(result.active_mask) or rms > 8 or float(errors.max()) > 16):
        raise ValueError(f'Calibration fit rejected ({rms:.1f} px RMS); check dimensions, distances and boxes')
    matrix = list(camera['k'])
    matrix[0], matrix[4] = float(result.x[0]), float(result.x[1])
    return dict(schema=1, kind='measured_object_floor', created_utc=datetime.now(timezone.utc).isoformat(),
                camera={**camera, 'k': matrix}, setup=setup,
                pitch=float(result.x[2]), yaw=float(result.x[3]), base_frame='base_footprint',
                training=samples, validation=[], validated=False,
                quality=dict(reprojection_rms_px=rms, max_error_px=float(errors.max()), condition=condition),
                range_m=[min(distances), max(distances)], bearing_rad=[min(bearings), max(bearings)])


def floor_point(profile, pixel, max_range=4.):
    """Project a contact pixel to the floor in base_footprint coordinates."""
    c, s = profile['camera'], profile['setup']
    ray = cv2.undistortPoints(np.array([[pixel]], dtype=float), np.array(c['k']).reshape(3, 3), np.array(c['d'])).reshape(2)
    direction = mounting_rotation(profile['pitch'], profile['yaw']) @ np.r_[ray, 1.]
    if direction[2] >= -1e-5:
        raise ValueError('Contact ray is above or too close to the horizon')
    point = direction * (-s['camera_height']/direction[2])
    distance = math.hypot(point[0], point[1])
    if point[0] <= 0 or distance > max_range:
        raise ValueError('Projected floor position is outside the usable range')
    return point + np.array([s['camera_x'], s['camera_y'], s['camera_height']])


def in_envelope(profile, distance, lateral, margin=.05):
    bearing = math.atan2(lateral, distance)
    return (profile['range_m'][0]-margin <= distance <= profile['range_m'][1]+margin
            and profile['bearing_rad'][0]-.02 <= bearing <= profile['bearing_rad'][1]+.02)


def validate_holdout(profile, sample):
    sample = validate_sample(sample, profile['camera'])
    if not in_envelope(profile, sample['distance'], sample['lateral']):
        raise ValueError('Validation position must be inside the training distance/bearing range')
    if any(math.hypot(sample['distance']-s['distance'], sample['lateral']-s['lateral']) < .08
           for s in profile['training']+profile['validation']):
        raise ValueError('Use a new independently measured position at least 8 cm from earlier samples')
    x1, _, x2, y2 = sample['box']
    point = floor_point(profile, [(x1+x2)/2, y2])
    expected = np.array([sample['distance']+profile['setup']['camera_x'], sample['lateral']+profile['setup']['camera_y']])
    error = float(np.linalg.norm(point[:2]-expected))
    return {**sample, 'estimated_base_xy': point[:2].tolist(), 'error_m': error, 'passed': error <= .10}


def save_profile(profile, directory):
    if len(profile['validation']) < 2 or not all(v['passed'] for v in profile['validation']):
        raise ValueError('Two independent validation positions must each pass the 0.10 m error limit')
    profile = {**profile, 'validated': True}
    # Unique filenames preserve every prior calibration. No live parameters or
    # active mission are modified when saving.
    import uuid
    path = Path(directory).expanduser()/('camera_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')+'_'+uuid.uuid4().hex[:8]+'.json')
    atomic_json(path, profile)
    return str(path)


def load_profile(path):
    profile = json.loads(Path(path).expanduser().read_text())
    if (profile.get('schema') != 1 or profile.get('kind') != 'measured_object_floor'
            or profile.get('validated') is not True or profile.get('simulated')):
        raise ValueError('Camera profile must be a validated measured-object calibration')
    validate_setup(profile['setup'])
    c = profile['camera']
    for key in ('width', 'height'):
        number(c[key], key, 16, 16384)
    model = camera_model(c['width'], c['height'], c['frame_id'], c['k'], c['d'])
    if (model['source'] != 'CameraInfo' or c['k'] != model['k']
            or not 0 < c['k'][2] < c['width'] or not 0 < c['k'][5] < c['height']
            or c['k'][1] != 0 or c['k'][3] != 0 or c['k'][6:] != [0., 0., 1.]):
        raise ValueError('Invalid calibrated pinhole matrix')
    number(profile['pitch'], 'pitch', -.5, 1.)
    number(profile['yaw'], 'yaw', -.4, .4)
    if (profile.get('base_frame') != 'base_footprint' or len(profile.get('validation', [])) < 2
            or any(v.get('passed') is not True or number(v['error_m'], 'validation error', 0, .1) > .1 for v in profile['validation'])):
        raise ValueError('Invalid calibration validation evidence')
    for key in ('range_m', 'bearing_rad'):
        bounds = np.asarray(profile[key], dtype=float)
        if bounds.shape != (2,) or not np.isfinite(bounds).all() or bounds[0] >= bounds[1]:
            raise ValueError('Invalid calibration coverage')
    return profile
