"""Capture-time roll/pitch from the fresh-packet-only controller topic.

No fallback to /robot_euler: that legacy driver topic republishes cached data.
Angles are radians, in base_footprint axes; yaw belongs to localization TF.
"""
from collections import deque
import hashlib
import json
import math
from pathlib import Path

import numpy as np

ATTITUDE_TOPIC = '/project/body_attitude'
MOUNT_FIELDS = ('camera_height', 'camera_pitch', 'camera_yaw', 'camera_x',
                'camera_y', 'use_pitch_override')


def mount_fingerprint(settings):
    values = {key: bool(settings.get(key, False)) if key == 'use_pitch_override'
              else float(settings.get(key, 0.)) for key in MOUNT_FIELDS}
    return hashlib.sha256(json.dumps(values, sort_keys=True).encode()).hexdigest()


def tilt_rotation(roll, pitch):
    if not np.isfinite([roll, pitch]).all():
        raise ValueError('Nonfinite controller attitude')
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return np.array([[cp, sp*sr, sp*cr], [0., cr, -sr], [-sp, cp*sr, cp*cr]])


def compensated_mount(rotation, translation, pivot, reference, current):
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    translation, pivot = (np.asarray(value, dtype=float).reshape(3)
                          for value in (translation, pivot))
    if not np.isfinite([*translation, *pivot, *rotation.ravel()]).all():
        raise ValueError('Invalid reference mounting geometry')
    delta = tilt_rotation(*current) @ tilt_rotation(*reference).T
    return delta @ rotation, pivot + delta @ (translation-pivot)


def default_config():
    return dict(version=1, mode='fixed', topic=ATTITUDE_TOPIC, pivot=None, reference=None)


def validate_config(data):
    if not isinstance(data, dict) or data.get('version') != 1 or data.get('mode') not in ('fixed', 'controller'):
        raise ValueError('Invalid attitude configuration version or mode')
    if data.get('topic') != ATTITUDE_TOPIC:
        raise ValueError('Attitude requires the fresh-packet-only body_attitude source')
    pivot = data.get('pivot')
    if pivot is not None and (not isinstance(pivot, list) or len(pivot) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in pivot)
            or not (-.5 <= pivot[0] <= .5 and -.5 <= pivot[1] <= .5 and 0 <= pivot[2] <= 1.)):
        raise ValueError('Provide a measured pivot [x,y,z] in metres')
    ref = data.get('reference')
    if ref is not None:
        try:
            values = [ref['roll'], ref['pitch'], ref['stamp'], *ref['translation'], *np.asarray(ref['rotation']).ravel()]
            r = np.asarray(ref['rotation'], dtype=float).reshape(3, 3)
            valid = (np.isfinite(values).all() and len(ref['translation']) == 3
                     and np.allclose(r.T@r, np.eye(3), atol=1e-6)
                     and math.isclose(np.linalg.det(r), 1., abs_tol=1e-6)
                     and .05 <= ref['translation'][2] <= 1.5
                     and isinstance(ref['mount_fingerprint'], str)
                     and ref['frame'] == 'base_footprint'
                     and ref['source'] == ATTITUDE_TOPIC)
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            raise ValueError('Invalid captured attitude reference')
    return dict(data)


def load_config(path):
    if not path:
        return default_config()
    # JSON is also YAML; a deliberately small format avoids ROS parameter coercion.
    return validate_config(json.loads(Path(path).expanduser().read_text()))


def reference_valid(config, settings):
    ref = config.get('reference')
    return bool(config.get('pivot') is not None and ref
                and ref.get('mount_fingerprint') == mount_fingerprint(settings))


class AttitudeBuffer:
    """Bounded interpolation; duplicates never extend the fresh sample window."""
    def __init__(self, max_gap=.15, max_age=.20, history_seconds=6.):
        # The fresh-only STM32 stream is ~10 Hz (measured 100–121 ms gaps).
        # Leave bounded jitter margin without extending the freshness timeout.
        if not all(math.isfinite(v) and v > 0 for v in (max_gap, max_age, history_seconds)):
            raise ValueError('Invalid attitude time limits')
        self.max_gap, self.max_age, self.history_seconds = max_gap, max_age, history_seconds
        self.samples = deque()

    def add(self, stamp, roll, pitch, frame='base_footprint'):
        if (not np.isfinite([stamp, roll, pitch]).all() or stamp <= 0
                or frame != 'base_footprint' or max(abs(roll), abs(pitch)) > math.pi/2):
            self.samples.clear()
            return False
        if self.samples and stamp <= self.samples[-1][0]:
            if stamp < self.samples[-1][0]:
                self.samples.clear()
            return False
        self.samples.append((float(stamp), float(roll), float(pitch)))
        while len(self.samples) > 2 and stamp-self.samples[0][0] > self.history_seconds:
            self.samples.popleft()
        return True

    def latest(self, now):
        if not self.samples or not 0 <= now-self.samples[-1][0] <= self.max_age:
            raise ValueError('attitude_missing_or_stale')
        stamp, roll, pitch = self.samples[-1]
        return dict(stamp=stamp, roll=roll, pitch=pitch, age=now-stamp,
                    frame='base_footprint', source_freshness_verified=True)

    def at(self, stamp):
        if not math.isfinite(stamp):
            raise ValueError('attitude_invalid_capture_time')
        samples = list(self.samples)
        for sample in samples:
            if abs(sample[0]-stamp) < 1e-8:
                return dict(stamp=stamp, roll=sample[1], pitch=sample[2], gap=0., rate=0.)
        for a, b in zip(samples, samples[1:]):
            if a[0] < stamp < b[0]:
                gap = b[0]-a[0]
                if gap > self.max_gap:
                    raise ValueError('attitude_capture_gap')
                fraction = (stamp-a[0])/gap
                change = np.arctan2(np.sin(np.asarray(b[1:])-a[1:]), np.cos(np.asarray(b[1:])-a[1:]))
                roll, pitch = np.asarray(a[1:])+fraction*change
                return dict(stamp=stamp, roll=float(roll), pitch=float(pitch), gap=gap,
                            rate=float(np.linalg.norm(change)/gap))
        raise ValueError('attitude_capture_unbracketed')

    def reference(self, now, seconds=1., max_span=math.radians(1.5)):
        self.latest(now)
        end = self.samples[-1][0]
        points = [p for p in self.samples if end-seconds-self.max_gap <= p[0] <= end]
        if len(points) < 5 or points[-1][0]-points[0][0] < seconds:
            raise ValueError('Collect one complete second of fresh attitude samples')
        if max(b[0]-a[0] for a, b in zip(points, points[1:])) > self.max_gap:
            raise ValueError('Attitude reference has a missing-sample gap')
        values = np.asarray([p[1:] for p in points])
        span = np.ptp(values, axis=0)
        if np.any(span > max_span):
            raise ValueError('Body attitude is not stable enough to capture a reference')
        mean = np.mean(values, axis=0)
        return dict(roll=float(mean[0]), pitch=float(mean[1]), stamp=end,
                    samples=len(points), span=span.tolist(), frame='base_footprint',
                    source=ATTITUDE_TOPIC)
