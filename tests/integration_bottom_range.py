"""Native ROS image -> empirical bottle range -> robust shared map posterior.

Neural inference is deterministic; ROS topics, capture-time TF, odometry,
attitude buffers, perception, association and filtering are production code.
Runs only in an isolated domain and never publishes motion commands.
"""
import json
import math
import os
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from ee3033_project.bottom_range import A, B, CALIBRATION_K, CALIBRATION_D
from ee3033_project.perception import Perception
from ee3033_project.target_filter import TargetFilter
from ee3033_project.ros_utils import LATCHED
from integration_ros import overrides, wait


class Tensor:
    def __init__(self, value): self.value=np.asarray(value,float)
    def cpu(self): return self
    def numpy(self): return self.value


class Detector:
    names={0:'bottle'}
    def __init__(self): self.box=[300.,100.,340.,A/(1.1-B)]
    def predict(self, frame, **kwargs):
        time.sleep(.15)
        box=SimpleNamespace(cls=SimpleNamespace(item=lambda:0),
            conf=SimpleNamespace(item=lambda:.9),xyxy=[Tensor(list(self.box))])
        return [SimpleNamespace(boxes=[box],names=self.names,orig_shape=frame.shape[:2],plot=lambda:frame.copy())]


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6')
    rclpy.init();fixture=Node('bottom_range_transport_fixture');detector=Detector()
    with patch.object(Perception,'load_detector',return_value=detector):
        perception=Perception(parameter_overrides=overrides(target_class='bottle',
            localization_method='hybrid',inference_period=.08))
    filt=TargetFilter(parameter_overrides=overrides(target_class='bottle'))
    dynamic=TransformBroadcaster(fixture);static=StaticTransformBroadcaster(fixture)
    tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='odom_combined'
    tr.header.stamp=fixture.get_clock().now().to_msg();tr.transform.rotation.w=1.;static.sendTransform(tr)
    pubs={name:fixture.create_publisher(kind,name,LATCHED if name in ('/map','/project/search_active','/project/mission_status') else 10)
        for name,kind in [('/map',OccupancyGrid),('/project/mission_status',String),
            ('/project/search_active',Bool),('/camera_info',CameraInfo),('/image_raw',Image),
            ('/odom',Odometry),('/amcl_pose',PoseWithCovarianceStamped),('/project/body_attitude',Vector3Stamped)]}
    frames=[];estimates=[];statuses=[]
    fixture.create_subscription(String,'/project/target_observations',lambda msg:frames.append(json.loads(msg.data)),10)
    fixture.create_subscription(String,'/project/target_estimate',lambda msg:estimates.append(json.loads(msg.data)),LATCHED)
    fixture.create_subscription(String,'/project/perception_status',lambda msg:statuses.append(json.loads(msg.data)),10)
    bridge=CvBridge();armed=[time.time()-.1];body_enabled=[True];moving=[False]
    def publish():
        stamp=fixture.get_clock().now().to_msg()
        transform=TransformStamped();transform.header.stamp=stamp;transform.header.frame_id='odom_combined'
        transform.child_frame_id='base_footprint';transform.transform.rotation.w=1.;dynamic.sendTransform(transform)
        image=bridge.cv2_to_imgmsg(np.zeros((480,640,3),np.uint8),'bgr8')
        image.header.stamp=stamp;image.header.frame_id='default_cam'
        info=CameraInfo();info.header=image.header;info.width=640;info.height=480
        info.k=CALIBRATION_K;info.d=CALIBRATION_D;info.distortion_model='plumb_bob'
        odom=Odometry();odom.header=transform.header;odom.child_frame_id='base_footprint';odom.pose.pose.orientation.w=1.
        if moving[0]:odom.twist.twist.linear.x=.08;odom.twist.twist.angular.z=.20
        pose=PoseWithCovarianceStamped();pose.header.stamp=stamp;pose.header.frame_id='map';pose.pose.pose.orientation.w=1.
        pose.pose.covariance[0]=pose.pose.covariance[7]=pose.pose.covariance[35]=.0001
        grid=OccupancyGrid();grid.header=pose.header;grid.info.resolution=.05;grid.info.width=grid.info.height=60
        grid.info.origin.position.x=grid.info.origin.position.y=-1.;grid.info.origin.orientation.w=1.;grid.data=[0]*3600
        body=Vector3Stamped();body.header.stamp=stamp;body.header.frame_id='base_footprint'
        body.vector.x=.1;body.vector.y=.2
        for name,msg in [('/map',grid),('/project/mission_status',String(data=json.dumps(dict(started_stamp=armed[0],target_detected=True)))),
                ('/project/search_active',Bool(data=True)),('/odom',odom),('/amcl_pose',pose),
                *([('/project/body_attitude',body)] if body_enabled[0] else []),('/camera_info',info),('/image_raw',image)]:
            pubs[name].publish(msg)
    fixture.create_timer(.1,publish)
    executor=MultiThreadedExecutor(num_threads=6)
    for node in (fixture,perception,filt):executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    def current(e):
        return (e.get('reportable') and e.get('mission_stamp')==armed[0]
            and e.get('stamp',-math.inf)>=armed[0]
            and all(stamp>=armed[0] for stamp in e.get('source_frames',[])))
    def normal(e):return current(e) and not e.get('near_guess')
    def motion_frame(e):return current(e) and 'capture_motion_uncertainty' in e.get('quality_flags',[])
    def near_frame(e):return current(e) and e.get('near_guess')
    def relaxed_frame(e):
        return (current(e) and not e.get('near_guess') and 'attitude_unverified' in e.get('quality_flags',[])
            and 'off_center_estimate' in e.get('quality_flags',[]))
    try:
        assert wait(lambda:statuses and statuses[-1].get('geometry_ready') and frames and frames[-1].get('bearings'),timeout=15),statuses[-1:]
        assert not frames[-1]['observations']
        assert frames[-1]['bearings'][0]['search_eligible'] is False
        assert filt.association.candidate is None
        assert not any(e.get('reportable') for e in estimates)
        detector.box=[300.,100.,340.,A/(.7-B)]
        assert wait(lambda:any(normal(e) for e in estimates),timeout=20),(statuses[-1:],frames[-1:],estimates[-1:])
        good=next(e for e in reversed(estimates) if normal(e))
        assert good['source']=='bottom_inverse_single' and len(good['source_frames'])==1 and good['stable'] is False
        assert math.dist((good['x'],good['y']),(.7,0))<.05,good
        assert good['sigma']>=.02 and len(filt.tracker.samples)==1
        assert not filt.tracker.bearings
        moving[0]=True;armed[0]=time.time();estimates.clear()
        assert wait(lambda:any(motion_frame(e) for e in estimates),timeout=15),(frames[-1:],estimates[-1:])
        motion=next(e for e in reversed(estimates) if motion_frame(e))
        assert len(motion['source_frames'])==1 and motion['capture_pose']['stamp']==motion['stamp']
        assert motion['stable'] is False
        moving[0]=False
        detector.box=[300.,150.,340.,480.];armed[0]=time.time();estimates.clear()
        assert wait(lambda:any(near_frame(e) for e in estimates),timeout=20),(frames[-1:],estimates[-1:])
        near=next(e for e in reversed(estimates) if near_frame(e))
        assert near['near_guess'] and near['sigma']>=.12
        assert near['observation_quality']=='approximate'
        assert .12<=math.hypot(near['x'],near['y'])<=.35
        body_enabled[0]=False;detector.box=[345.,100.,385.,A/(.7-B)]
        armed[0]=time.time();estimates.clear()
        assert wait(lambda:any(relaxed_frame(e) for e in estimates),timeout=25),(frames[-1:],estimates[-1:])
        relaxed=next(e for e in reversed(estimates) if relaxed_frame(e))
        assert relaxed['observation_quality']=='approximate' and 'off_center_estimate' in relaxed['quality_flags']
        print(json.dumps(dict(native_bottom_range_passed=True,far_excluded=True,normal_frames=1,moving_single_frame=True,
            normal_xy=[good['x'],good['y']],near_xy=[near['x'],near['y']],near_sigma=near['sigma'],
            relaxed_quality=relaxed['quality_flags'])),flush=True)
    finally:
        executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        for node in (fixture,perception,filt):node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':main()
