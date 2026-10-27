"""Shared, ROS-independent camera geometry for floor localization and coverage.

Mount transforms are optical -> base_footprint, in metres. No fitted bottle
profile is needed. Explicit FOV intrinsics are an approximation, never labelled
calibrated. All geometry refers to the original image, not YOLO's resized input.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np


def validate_mount_settings(values):
    values = dict(values) if isinstance(values, dict) else values
    override = values.pop('use_pitch_override', False) if isinstance(values, dict) else False
    if type(override) is not bool:
        raise ValueError('use_pitch_override must be a boolean')
    bounds = dict(camera_height=(0., 1.5), camera_pitch=(-1.05, 1.05),
                  camera_yaw=(-math.pi, math.pi), camera_x=(-.5, .5), camera_y=(-.5, .5),
                  horizontal_fov=(0., 2.8), camera_min_range=(.2, 1.),
                  camera_max_range=(.5, 3.))
    if not isinstance(values, dict) or set(values) != set(bounds):
        raise ValueError('Provide the complete camera mount and range settings')
    result = {}
    for name, (low, high) in bounds.items():
        value = values[name]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(name+' is outside its valid range')
        result[name] = float(value)
    if override:
        result['use_pitch_override'] = True
    if (0 < result['camera_height'] < .05 or 0 < result['horizontal_fov'] < .2
            or result['camera_min_range'] >= result['camera_max_range']):
        raise ValueError('Use a measured height/FOV or zero for existing TF/CameraInfo')
    return result


def override_pitch(rotation, pitch):
    """Replace base-frame pitch while retaining mounting yaw and optical roll."""
    forward = np.asarray(rotation)[:, 2]
    old = math.atan2(-forward[2], math.hypot(forward[0], forward[1]))
    yaw = math.atan2(forward[1], forward[0])
    a = pitch-old
    axis = np.array([-math.sin(yaw), math.cos(yaw), 0.])
    skew = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]],
                     [-axis[1], axis[0], 0.]])
    change = np.eye(3)+math.sin(a)*skew+(1-math.cos(a))*(skew@skew)
    return change@rotation


def camera_matrix(width, height, k=None, d=None, horizontal_fov=0.):
    matrix = np.asarray(k if k is not None else [0.]*9, dtype=float).reshape(3, 3)
    source = 'CameraInfo'
    if horizontal_fov != 0 or not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        if not math.isfinite(horizontal_fov) or not .2 <= horizontal_fov <= 2.8:
            raise ValueError('Valid CameraInfo or an explicitly measured horizontal FOV is required')
        focal = width/(2*math.tan(horizontal_fov/2))
        matrix = np.array([[focal, 0, width/2], [0, focal, height/2], [0, 0, 1.]])
        source = 'explicit FOV approximation'
        d = []
    distortion = np.asarray(d if d is not None and len(d) else [0.]*5, dtype=float)
    if (width < 16 or height < 16 or not 0 <= matrix[0, 2] < width
            or not 0 <= matrix[1, 2] < height
            or not np.allclose(matrix[2], [0, 0, 1])
            or len(distortion) not in (4, 5, 8, 12, 14)
            or not np.isfinite(distortion).all()):
        raise ValueError('Invalid camera intrinsics, distortion or image dimensions')
    return matrix, distortion, source


@dataclass
class FloorCamera:
    width: int
    height: int
    k: np.ndarray
    d: np.ndarray
    rotation: np.ndarray
    translation: np.ndarray
    source: str = 'CameraInfo + timestamped TF'
    min_range: float = .35
    max_range: float = 2.
    margin: float = 8.
    pixel_sigma: float = 2.
    height_sigma: float = .015
    pitch_sigma: float = math.radians(2.)
    bearing_limits: object = None

    def __post_init__(self):
        self.k = np.asarray(self.k, dtype=float).reshape(3, 3)
        self.d = np.asarray(self.d, dtype=float)
        self.rotation = np.asarray(self.rotation, dtype=float).reshape(3, 3)
        self.translation = np.asarray(self.translation, dtype=float).reshape(3)
        camera_matrix(self.width, self.height, self.k.ravel(), self.d)
        values = [self.min_range, self.max_range, self.margin, self.pixel_sigma,
                  self.height_sigma, self.pitch_sigma]
        if (not np.isfinite(values).all() or not np.isfinite(self.translation).all()
                or not np.isfinite(self.rotation).all()
                or not np.allclose(self.rotation.T@self.rotation, np.eye(3), atol=1e-5)
                or not math.isclose(np.linalg.det(self.rotation), 1., abs_tol=1e-5)
                or not .05 <= self.translation[2] <= 1.5
                or not 0 < self.min_range < self.max_range <= 5.
                or not 0 <= self.margin < min(self.width, self.height)/4
                or min(self.pixel_sigma, self.height_sigma, self.pitch_sigma) <= 0):
            raise ValueError('Invalid camera mounting geometry or uncertainty')
        if self.bearing_limits is not None and (
                len(self.bearing_limits) != 2 or not np.isfinite(self.bearing_limits).all()
                or self.bearing_limits[0] > self.bearing_limits[1]):
            raise ValueError('Invalid legacy bearing envelope')

    def metadata(self):
        return {name: value.tolist() if isinstance(value, np.ndarray) else value
                for name, value in vars(self).items()}

    @classmethod
    def from_metadata(cls, values):
        return cls(**values)

    @property
    def yaw(self):
        return math.atan2(self.rotation[1, 2], self.rotation[0, 2])

    @property
    def fov(self):
        pixels = np.array([[[self.margin, self.height/2],
                            [self.width-self.margin, self.height/2]]], dtype=float)
        rays = cv2.undistortPoints(pixels, self.k, self.d).reshape(-1, 2)
        return abs(math.atan(rays[1, 0])-math.atan(rays[0, 0]))

    def intersection(self, pixel, rotation=None, translation=None):
        ray = cv2.undistortPoints(np.array([[pixel]], dtype=float), self.k, self.d).reshape(2)
        direction = (self.rotation if rotation is None else rotation)@np.r_[ray, 1.]
        origin = self.translation if translation is None else translation
        if direction[2] >= -1e-5:
            raise ValueError('Contact ray is above or too close to the horizon')
        return origin-direction*(origin[2]/direction[2])

    def project_box(self, box, coarse_max_range=None):
        x1, y1, x2, y2 = [float(v) for v in box]
        if (not np.isfinite([x1, y1, x2, y2]).all()
                or not self.margin <= x1 < x2 <= self.width-self.margin
                or not self.margin <= y1 < y2 <= self.height-self.margin
                or x2-x1 < 4 or y2-y1 < 8):
            raise ValueError('Target is too small or clipped; seek a better view')
        pixel = np.array([(x1+x2)/2, y2])
        point = self.intersection(pixel)
        distance = np.linalg.norm(point[:2]-self.translation[:2])
        limit = self.max_range if coarse_max_range is None else max(self.max_range, coarse_max_range)
        if not self.min_range <= distance <= limit:
            raise ValueError('Target contact is outside the reliable camera range')
        derivatives = []
        for axis in range(2):
            offset = np.zeros(2); offset[axis] = self.pixel_sigma
            derivatives.append((self.intersection(pixel+offset)-self.intersection(pixel-offset))[:2]/2)
        offset = np.array([0., 0., self.height_sigma])
        derivatives.append((self.intersection(pixel, translation=self.translation+offset)-
                            self.intersection(pixel, translation=self.translation-offset))[:2]/2)
        a = self.pitch_sigma
        rotation = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0],
                             [-math.sin(a), 0, math.cos(a)]])
        derivatives.append((self.intersection(pixel, rotation=rotation@self.rotation)-
                            self.intersection(pixel, rotation=rotation.T@self.rotation))[:2]/2)
        jacobian = np.asarray(derivatives).T
        sigma = math.sqrt(max(.0001, float(np.linalg.eigvalsh(jacobian@jacobian.T).max())))
        return point, sigma, float(distance)

    def view_requirements(self, box):
        """Describe rejected near/cropped views without making a measurement.

        A floor-contact range, when available, is only a steering hint here.
        It never bypasses project_box's complete-box or reliable-range checks.
        """
        result=dict(reframe_required=False,reframe_reasons=[],range_state='unknown',
            range_hint_m=None,range_hint_is_measurement=False,
            camera_min_range=float(self.min_range),camera_max_range=float(self.max_range))
        try:
            x1,y1,x2,y2=map(float,box)
            if not np.isfinite([x1,y1,x2,y2]).all() or not x1<x2 or not y1<y2:
                result['reframe_reasons']=['invalid_box']
                return result
            horizontal=x1<self.margin or x2>self.width-self.margin
            vertical=y1<self.margin or y2>self.height-self.margin
            if horizontal:result['reframe_reasons'].append('horizontal_clipping')
            if vertical:result['reframe_reasons'].append('vertical_clipping')
            # An incomplete bottom edge or horizontal centre is not a contact
            # range hint. Retain only the independent image-bearing route.
            if not horizontal and y2<=self.height-self.margin and x2-x1>=4:
                point=self.intersection([(x1+x2)/2,y2])
                distance=float(np.linalg.norm(point[:2]-self.translation[:2]))
                if math.isfinite(distance):
                    result['range_hint_m']=distance
                    result['range_state']=('too_near' if distance<self.min_range else
                        'too_far' if distance>self.max_range else 'within_range')
                    if distance<self.min_range:result['reframe_reasons'].append('contact_too_near')
            result['reframe_required']=horizontal or vertical or result['range_state']=='too_near'
            if result['reframe_required']:result['reframe_action']='move_away'
        except (ValueError,TypeError,cv2.error):
            result['reframe_reasons'].append('contact_hint_unavailable')
        return result

    def contains_floor(self, points):
        """Contact points inside both vertical/horizontal FOV and reliable range."""
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        xyz = np.column_stack([points, np.zeros(len(points))])
        relative = (xyz-self.translation)@self.rotation
        valid = relative[:, 2] > 1e-5
        distance = np.linalg.norm(points-self.translation[:2], axis=1)
        valid &= (distance >= self.min_range) & (distance <= self.max_range)
        if self.bearing_limits is not None:
            bearing = np.arctan2(points[:, 1]-self.translation[1], points[:, 0]-self.translation[0])
            valid &= (bearing >= self.bearing_limits[0]) & (bearing <= self.bearing_limits[1])
        pixels, _ = cv2.projectPoints(relative, np.zeros(3), np.zeros(3), self.k, self.d)
        pixels = pixels.reshape(-1, 2)
        valid &= ((pixels[:, 0] >= self.margin) & (pixels[:, 0] <= self.width-self.margin)
                  & (pixels[:, 1] >= self.margin) & (pixels[:, 1] <= self.height-self.margin))
        return valid
