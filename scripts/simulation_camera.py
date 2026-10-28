"""Physical camera model shared by rendered pixels, TF and CameraInfo.

Mount angles are radians, downward pitch is positive. Min/max ranges describe
algorithm measurement validity only; they never clip the physical renderer.
"""
import math
import numpy as np
from ee3033_project.calibration import mounting_rotation
from ee3033_project.vision_geometry import validate_mount_settings

NATIVE_K=np.array([[450.,0.,320.],[0.,450.,240.],[0.,0.,1.]])


def default_camera_settings():
    return dict(camera_height=.22,camera_pitch=math.radians(10.),camera_yaw=0.,
        camera_x=.05,camera_y=0.,horizontal_fov=0.,camera_min_range=.35,camera_max_range=2.)


def validate_camera_settings(settings):
    """Validate the complete existing mount form as an actual physical camera."""
    result=validate_mount_settings(settings)
    result.pop('use_pitch_override',None)
    if result['camera_height']<=.05:
        raise ValueError('Physical simulation camera height must be greater than 0.05 m')
    return result


def camera_intrinsics(settings=None):
    camera=default_camera_settings() if settings is None else validate_camera_settings(settings)
    fov=camera['horizontal_fov']
    if fov==0:return NATIVE_K.copy()
    focal=320./math.tan(fov/2.)
    return np.array([[focal,0.,320.],[0.,focal,240.],[0.,0.,1.]])


def camera_mount(settings=None):
    camera=default_camera_settings() if settings is None else validate_camera_settings(settings)
    translation=np.array([camera['camera_x'],camera['camera_y'],camera['camera_height']])
    return translation,mounting_rotation(camera['camera_pitch'],camera['camera_yaw'])
