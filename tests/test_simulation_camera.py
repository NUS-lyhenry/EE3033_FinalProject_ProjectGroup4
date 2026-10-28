"""Physical simulation mount, rendered pixels and published geometry agree."""
import ast
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from simulation_camera import default_camera_settings,validate_camera_settings,camera_intrinsics,camera_mount
import simulation_geometry as world
from ee3033_project.calibration import mounting_rotation
from ee3033_project.vision_geometry import FloorCamera


def test_default_physical_camera_preserves_original_geometry():
    camera=default_camera_settings()
    assert camera['camera_height']==.22 and camera['camera_pitch']==math.radians(10.)
    assert np.array_equal(camera_intrinsics(camera),world.K)
    assert np.array_equal(camera_mount(camera)[0],world.CAMERA_OFFSET)
    old=world.camera_pose([.3,-.2,.8]);explicit=world.camera_pose([.3,-.2,.8],camera)
    assert all(np.array_equal(a,b) for a,b in zip(old,explicit))


def test_full_camera_form_changes_projection_and_roundtrips_actual_floor_contact():
    camera=default_camera_settings();camera.update(camera_height=.31,camera_pitch=math.radians(28.6),
        camera_yaw=.25,camera_x=.12,camera_y=-.04,horizontal_fov=math.radians(70.))
    k=camera_intrinsics(camera)
    assert k[0,0]==pytest.approx(320/math.tan(math.radians(35))) and k[1,1]==k[0,0]
    pose=[.3,-.2,.4];translation,rotation=world.camera_pose(pose,camera)
    mount,optical=camera_mount(camera)
    body=Rotation.from_euler('z',pose[2]).as_matrix()
    assert translation==pytest.approx(np.r_[pose[:2],0.]+body@mount)
    assert rotation==pytest.approx(body@mounting_rotation(camera['camera_pitch'],camera['camera_yaw']))
    base_contact=np.array([1.,.2,0.]);contact=np.r_[pose[:2],0.]+body@base_contact
    pixel=world.project([contact],translation,rotation,k)[0]
    model=FloorCamera(640,480,k,np.zeros(5),optical,mount)
    assert model.intersection(pixel)==pytest.approx(base_contact,abs=1e-6)
    old_translation,old_rotation=world.camera_pose(pose)
    assert not np.allclose(pixel,world.project([contact],old_translation,old_rotation)[0])


def test_range_limits_do_not_crop_the_physical_image():
    camera=default_camera_settings();restricted=dict(camera,camera_min_range=.8,camera_max_range=1.2)
    texture=np.full((80,80,3),(10,240,20),np.uint8);pose=[.95,.1,math.pi/2]
    assert np.array_equal(world.render_camera(pose,texture,True,camera=camera),
                          world.render_camera(pose,texture,True,camera=restricted))
    changed=dict(camera,camera_height=.31,camera_pitch=math.radians(28.6),horizontal_fov=math.radians(45))
    assert not np.array_equal(world.render_camera(pose,texture,True,camera=camera),
                              world.render_camera(pose,texture,True,camera=changed))


@pytest.mark.parametrize('changes',[dict(camera_height=0.),dict(camera_height=.01),dict(camera_height=.05),dict(camera_pitch=2.),
    dict(horizontal_fov=.1),dict(camera_yaw=float('nan')),dict(camera_x=True)])
def test_invalid_physical_mount_rejected(changes):
    with pytest.raises(ValueError):validate_camera_settings(dict(default_camera_settings(),**changes))


def transform():
    return NS(header=NS(),transform=NS(translation=NS(),rotation=NS()))


def robot_stub():
    source=Path(__file__).parents[1]/'scripts/simulation_robot.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='SimulatedRobot')
    scope=dict(Node=object,math=math,np=np,Rotation=Rotation,TransformStamped=transform,
        validate_camera_settings=validate_camera_settings,camera_intrinsics=camera_intrinsics,
        render_camera=world.render_camera,CameraInfo=lambda:NS())
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),scope)
    robot=scope['SimulatedRobot'].__new__(scope['SimulatedRobot'])
    robot.world_lock=threading.RLock();robot.accept_commands=False
    robot.camera=default_camera_settings();robot.camera_k=camera_intrinsics(robot.camera)
    robot.pose=[.95,.1,math.pi/2];robot.texture=np.full((80,80,3),120,np.uint8)
    robot.phase='mission';robot.walls=world.WALLS;robot.target=world.TARGET
    robot.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:NS(sec=1,nanosec=0)))
    transforms=[];images=[];infos=[]
    robot.static=NS(sendTransform=transforms.append)
    robot.image=NS(publish=images.append);robot.info=NS(publish=infos.append)
    robot.bridge=NS(cv2_to_imgmsg=lambda frame,encoding:NS(frame=frame,header=NS()))
    return robot,transforms,images,infos


def test_camera_update_republishes_laser_and_mount_tf_and_matching_camera_info():
    robot,transforms,images,infos=robot_stub();pose=robot.pose[:]
    camera=dict(default_camera_settings(),camera_height=.31,camera_pitch=math.radians(28.6),
                camera_yaw=.2,camera_x=.1,camera_y=.03,horizontal_fov=math.radians(60.))
    result=robot.apply_camera(camera)
    assert result==camera and robot.pose==pose
    camera['camera_height']=.9
    assert robot.camera_settings()['camera_height']==.31
    snapshot=robot.camera_settings();snapshot['camera_height']=.8
    assert robot.camera['camera_height']==.31 and not robot.camera_k.flags.writeable
    laser,mount=transforms[-1]
    assert laser.child_frame_id=='preview_laser' and laser.transform.translation.z==.12
    assert mount.child_frame_id=='camera_link'
    assert [mount.transform.translation.x,mount.transform.translation.y,mount.transform.translation.z]==[.1,.03,.31]
    q=mount.transform.rotation
    link=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    assert link@mounting_rotation(0.,0.)==pytest.approx(mounting_rotation(math.radians(28.6),.2))
    robot.camera_tick()
    assert infos[-1].k==camera_intrinsics(robot.camera).ravel().tolist()
    assert infos[-1].header is images[-1].header
    assert np.array_equal(images[-1].frame,world.render_camera(robot.pose,robot.texture,True,
                          robot.walls,robot.target,robot.camera))


def test_camera_update_requires_paused_chassis_and_validation_is_atomic():
    robot,transforms,_,_=robot_stub();before=robot.camera_settings()
    robot.accept_commands=True
    with pytest.raises(RuntimeError,match='Pause'):robot.apply_camera(before)
    robot.accept_commands=False
    with pytest.raises(ValueError):robot.apply_camera(dict(before,camera_height=0.))
    assert robot.camera_settings()==before and transforms==[]
