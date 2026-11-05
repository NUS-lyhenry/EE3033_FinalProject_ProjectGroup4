"""Physical camera model shared by rendered pixels, TF and CameraInfo.

Mount angles are radians, downward pitch is positive. Min/max ranges describe
algorithm measurement validity only; they never clip the physical renderer.
"""
import math
from functools import lru_cache
import cv2
import numpy as np
from ee3033_project.calibration import mounting_rotation
from ee3033_project.bottom_range import CALIBRATION_K, CALIBRATION_D
from ee3033_project.vision_geometry import validate_mount_settings

NATIVE_K=np.asarray(CALIBRATION_K,dtype=float).reshape(3,3)
NATIVE_D=np.asarray(CALIBRATION_D,dtype=float)


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


def camera_distortion(settings=None):
    camera=default_camera_settings() if settings is None else validate_camera_settings(settings)
    # Native means the original, unrectified sensor used for bottle collection.
    # A manually selected FOV remains an explicit ideal pinhole alternative.
    return NATIVE_D.copy() if camera['horizontal_fov']==0 else np.zeros(5)


@lru_cache(maxsize=8)
def _render_maps(k_values,d_values,width,height):
    k=np.asarray(k_values,dtype=float).reshape(3,3)
    d=np.asarray(d_values,dtype=float)
    if not np.any(d):return k,None,None,width,height
    vv,uu=np.mgrid[:height,:width]
    pixels=np.stack((uu,vv),axis=-1).astype(np.float64).reshape(-1,1,2)
    # For each distorted OUTPUT pixel, locate its undistorted INPUT pixel.
    # This is the inverse direction of the usual camera-undistortion remap.
    source=cv2.undistortPoints(pixels,k,d,P=k).reshape(height,width,2)
    low=np.min(source,axis=(0,1));high=np.max(source,axis=(0,1))
    padding=int(math.ceil(max(0.,-low[0],-low[1],high[0]-(width-1),high[1]-(height-1))))+2
    padding+=padding%2
    render_k=k.copy();render_k[:2,2]+=padding
    map_x=np.ascontiguousarray(source[:,:,0]+padding,dtype=np.float32)
    map_y=np.ascontiguousarray(source[:,:,1]+padding,dtype=np.float32)
    for value in (render_k,map_x,map_y):value.setflags(write=False)
    # Overscan supplies real scene rays at distorted sensor edges rather than
    # filling them with invented grey/black pixels from a cropped pinhole view.
    return render_k,map_x,map_y,width+2*padding,height+2*padding


def camera_render_geometry(settings=None,width=640,height=480):
    k=camera_intrinsics(settings);d=camera_distortion(settings)
    return _render_maps(tuple(k.ravel()),tuple(d),width,height)


def distort_camera_frame(frame,settings=None):
    _,map_x,map_y,width,height=camera_render_geometry(settings)
    if frame.shape[:2]!=(height,width):raise ValueError('Camera render dimensions do not match the optical model')
    return frame if map_x is None else cv2.remap(frame,map_x,map_y,cv2.INTER_LINEAR)
