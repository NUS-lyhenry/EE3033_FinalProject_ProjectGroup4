"""Real perception geometry methods with deterministic exposure-time samples."""
import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest

from ee3033_project.attitude_geometry import (ATTITUDE_TOPIC, AttitudeBuffer, default_config,
    mount_fingerprint, reference_valid, compensated_mount)
from ee3033_project.calibration import mounting_rotation
from ee3033_project.vision_geometry import FloorCamera, camera_matrix, override_pitch


def perception(mode='controller', legacy=False):
    path = Path(__file__).parents[1]/'src/ee3033_project/ee3033_project/perception.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and
                n.name in ('capture_attitude', 'attitude_ready', 'camera_geometry', 'locate')]
    scope = dict(math=math, np=np, cv2=cv2, ATTITUDE_TOPIC=ATTITUDE_TOPIC,
        reference_valid=reference_valid, compensated_mount=compensated_mount,
        mounting_rotation=mounting_rotation, FloorCamera=FloorCamera, camera_matrix=camera_matrix,
        override_pitch=override_pitch, stamp_seconds=lambda stamp: stamp,
        Time=NS(from_msg=lambda stamp: stamp),
        transform_arrays=lambda transform: (np.eye(3), np.zeros(3)),
        floor_point=lambda *a: (_ for _ in ()).throw(AssertionError('Legacy geometry bypassed compensation')))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), scope)
    node = scope['Perception']()
    node.p = dict(localization_method='hybrid', camera_height=.33, camera_pitch=.499,
        camera_yaw=0., camera_x=0., camera_y=0., use_pitch_override=True,
        horizontal_fov=0., camera_min_range=.35, camera_max_range=2., pixel_margin=8.,
        pixel_sigma=2., height_sigma=.015, pitch_sigma=.035, max_range=3.,
        map_frame='map', base_frame='base_footprint', camera_optical_frame='camera')
    node.lock = threading.Lock()
    node.attitude_buffer = AttitudeBuffer()
    node.attitude_buffer.add(10., .0, .0)
    node.attitude_buffer.add(10.08, .0, .1)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_100_000_000))
    node.attitude_config = dict(default_config(), mode=mode, pivot=[0., 0., .033],
        reference=dict(roll=0., pitch=0., rotation=mounting_rotation(.499, 0.).tolist(),
            translation=[-.01, 0., .33], mount_fingerprint=mount_fingerprint(node.p)))
    node.pose_variance = (.001, .001)
    node.buffer = NS(lookup_transform=lambda *a: None)
    node.calibration = None
    camera = NS(width=640, height=480, header=NS(frame_id='camera'),
        k=[350., 0., 320., 0., 350., 240., 0., 0., 1.], d=[0.]*5, distortion_model='plumb_bob')
    image = NS(width=640, height=480, header=NS(stamp=10.04, frame_id='camera'))
    if legacy:
        node.calibration = dict(camera=dict(width=640, height=480, frame_id='camera', k=camera.k, d=camera.d),
            setup=dict(camera_x=0., camera_y=0., camera_height=.33),
            pitch=.499, yaw=0., range_m=[.35, 2.], bearing_rad=[-.7, .7])
    return node, image, camera


def test_perception_uses_exposure_attitude_and_moves_camera_origin():
    node, image, camera = perception()
    model = node.camera_geometry(image, camera)
    r, t = compensated_mount(mounting_rotation(.499, 0.), [-.01, 0., .33], [0., 0., .033],
                              (0., 0.), (0., .05))
    assert model.rotation == pytest.approx(r)
    assert model.translation == pytest.approx(t)
    assert node.attitude_check['pitch'] == pytest.approx(.05)
    assert node.attitude_check['valid']


def test_no_legacy_fallback_when_source_missing_or_mount_changed():
    node, image, camera = perception()
    node.attitude_buffer.samples.clear()
    with pytest.raises(ValueError, match='missing_or_stale'):
        node.camera_geometry(image, camera)
    assert not node.attitude_check['valid']
    node, image, camera = perception()
    node.p['camera_pitch'] = .4
    with pytest.raises(ValueError, match='mount_changed'):
        node.camera_geometry(image, camera)


def test_fixed_mode_preserves_existing_mount_without_any_attitude_source():
    node, image, camera = perception(mode='fixed')
    node.attitude_buffer.samples.clear()
    model = node.camera_geometry(image, camera)
    assert model.translation == pytest.approx([0., 0., .33])
    assert model.rotation == pytest.approx(mounting_rotation(.499, 0.))
    assert model.attitude_pivot is None


def test_legacy_profile_cannot_bypass_compensated_geometry_in_locate():
    node, image, camera = perception(legacy=True)
    node.floor_camera = node.camera_geometry(image, camera)
    box = [280., 160., 330., 260.]
    expected = node.floor_camera.project_box(box, coarse_max_range=3.)[0]
    assert node.locate(box, image, camera, []) == pytest.approx(expected)
    assert node.projection_sigma > 0
