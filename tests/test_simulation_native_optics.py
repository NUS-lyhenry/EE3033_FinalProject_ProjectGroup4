"""Native bottle optics are physical distorted pixels, not a metadata shortcut."""
from pathlib import Path
import sys
import cv2
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from simulation_camera import (default_camera_settings, camera_intrinsics, camera_distortion,
    camera_render_geometry, distort_camera_frame, camera_mount)
from ee3033_project.bottom_range import CALIBRATION_K, CALIBRATION_D
from ee3033_project.vision_geometry import FloorCamera
from test_simulation_camera import robot_stub
from simulation_geometry import target_dimensions


def test_native_optics_use_the_owning_collection_constants_and_explicit_fov_is_pinhole():
    assert camera_intrinsics().ravel() == pytest.approx(CALIBRATION_K)
    assert camera_distortion() == pytest.approx(CALIBRATION_D)
    manual=dict(default_camera_settings(),horizontal_fov=np.deg2rad(70.))
    assert np.array_equal(camera_distortion(manual),np.zeros(5))
    k, x, y, width, height=camera_render_geometry(manual)
    assert (width,height)==(640,480) and x is None and y is None


@pytest.mark.parametrize('pixel',[(100.,100.),(320.,240.),(540.,380.)])
def test_physical_distortion_maps_pinhole_pixels_to_projected_raw_sensor_pixels(pixel):
    k=camera_intrinsics();d=camera_distortion()
    render_k,map_x,map_y,width,height=camera_render_geometry()
    assert np.min(map_x)>=0 and np.max(map_x)<width-1
    assert np.min(map_y)>=0 and np.max(map_y)<height-1
    point=np.linalg.inv(k)@np.r_[pixel,1.]
    expected=cv2.projectPoints(point.reshape(1,1,3),np.zeros(3),np.zeros(3),k,d)[0].ravel()
    source=render_k@point
    frame=np.zeros((height,width,3),np.uint8)
    cv2.circle(frame,tuple(np.round(source[:2]/source[2]).astype(int)),3,(255,255,255),-1)
    raw=distort_camera_frame(frame)
    weights=raw[:,:,0].astype(float)
    vv,uu=np.mgrid[:480,:640]
    center=np.array([(uu*weights).sum(),(vv*weights).sum()])/weights.sum()
    assert center==pytest.approx(expected,abs=1.)
    # The cache stores sensor geometry only, never poses or rendered images.
    assert camera_render_geometry()[1] is map_x


def test_measured_distortion_roundtrips_ground_points_with_the_saved_mount():
    camera=dict(default_camera_settings(),camera_height=.305,camera_pitch=np.deg2rad(28.6),
                camera_yaw=.1,camera_x=-.01)
    translation,rotation=camera_mount(camera)
    k=camera_intrinsics(camera);d=camera_distortion(camera)
    model=FloorCamera(640,480,k,d,rotation,translation)
    for point in (np.array([.7,.05,0.]),np.array([.9,-.1,0.])):
        optical=(point-translation)@rotation
        pixel=cv2.projectPoints(optical.reshape(1,1,3),np.zeros(3),np.zeros(3),k,d)[0].ravel()
        assert model.intersection(pixel)==pytest.approx(point,abs=1e-5)


def test_native_published_camera_info_matches_the_rendered_image_model():
    robot,_,images,infos=robot_stub()
    robot.camera_tick()
    assert infos[-1].k==pytest.approx(CALIBRATION_K)
    assert infos[-1].d==pytest.approx(CALIBRATION_D)
    assert images[-1].frame.shape==(480,640,3)
    assert images[-1].header is infos[-1].header


def test_physical_target_keeps_the_photograph_aspect_and_eighteen_centimetre_height():
    width,height=target_dimensions(np.zeros((313,115,3),np.uint8))
    assert height==.18
    assert width==pytest.approx(.18*115/313)
    assert width/height==pytest.approx(115/313)
    with pytest.raises(ValueError,match='nonzero'):
        target_dimensions(np.zeros((0,115,3),np.uint8))
