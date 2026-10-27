"""Native Image -> bearing -> MMSE transport; vertical clipping prevents floor depth.

Detector alone is deterministic. Timestamped TF, odometry windows, frame
association, intersection and the shared estimator are the production nodes.
"""
import json,math,os,threading,time
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import String,Bool
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from ee3033_project.perception import Perception
from ee3033_project.target_filter import TargetFilter
from ee3033_project.calibration import mounting_rotation
from ee3033_project.ros_utils import LATCHED,set_yaw
from integration_ros import overrides,wait
from integration_visual_perception import Detector


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6')
    rclpy.init();fixture=Node('bearing_transport_fixture');detector=Detector()
    detector.box=[308.,260.,332.,480.]
    with patch.object(Perception,'load_detector',return_value=detector):
        perception=Perception(parameter_overrides=overrides(target_class='bottle',localization_method='hybrid',inference_period=.08))
    filt=TargetFilter();tf=TransformBroadcaster(fixture);static=StaticTransformBroadcaster(fixture)
    transforms=[]
    for parent,child,xyz,rotation in [('map','odom_combined',[0.,0.,0.],np.eye(3)),
            ('base_footprint','project_camera_optical',[0.,0.,.22],mounting_rotation(math.radians(10),0.))]:
        tr=TransformStamped();tr.header.frame_id=parent;tr.child_frame_id=child
        tr.header.stamp=fixture.get_clock().now().to_msg()
        tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=xyz
        q=Rotation.from_matrix(rotation).as_quat()
        tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=map(float,q)
        transforms.append(tr)
    static.sendTransform(transforms)
    pubs={topic:fixture.create_publisher(cls,topic,LATCHED if topic in ('/map','/project/mission_status','/project/search_active') else 10)
          for topic,cls in [('/map',OccupancyGrid),('/project/mission_status',String),('/project/search_active',Bool),
                           ('/image_raw',Image),('/camera_info',CameraInfo),('/odom',Odometry),('/amcl_pose',PoseWithCovarianceStamped)]}
    origin=[0.,0.,math.pi/4];bridge=CvBridge();estimates=[];frames=[]
    fixture.create_subscription(String,'/project/target_estimate',lambda msg:estimates.append(json.loads(msg.data)),LATCHED)
    fixture.create_subscription(String,'/project/target_observations',lambda msg:frames.append(json.loads(msg.data)),10)
    armed=time.time()-.1
    def publish():
        stamp=fixture.get_clock().now().to_msg()
        tr=TransformStamped();tr.header.stamp=stamp;tr.header.frame_id='odom_combined';tr.child_frame_id='base_footprint'
        tr.transform.translation.x,tr.transform.translation.y=origin[:2];set_yaw(tr.transform.rotation,origin[2]);tf.sendTransform(tr)
        image=bridge.cv2_to_imgmsg(np.zeros((480,640,3),np.uint8),'bgr8');image.header.stamp=stamp;image.header.frame_id='project_camera_optical'
        info=CameraInfo();info.header=image.header;info.width=640;info.height=480;info.k=[450.,0.,320.,0.,450.,240.,0.,0.,1.];info.d=[0.]*5;info.distortion_model='plumb_bob'
        odom=Odometry();odom.header=tr.header;odom.child_frame_id='base_footprint';odom.pose.pose.position.x,odom.pose.pose.position.y=origin[:2];set_yaw(odom.pose.pose.orientation,origin[2])
        pose=PoseWithCovarianceStamped();pose.header.stamp=stamp;pose.header.frame_id='map';pose.pose.pose=odom.pose.pose
        pose.pose.covariance[0]=pose.pose.covariance[7]=pose.pose.covariance[35]=.0001
        grid=OccupancyGrid();grid.header=pose.header;grid.info.resolution=.05;grid.info.width=grid.info.height=60;grid.info.origin.position.x=grid.info.origin.position.y=-1.;grid.info.origin.orientation.w=1.;grid.data=[0]*3600
        for topic,msg in [('/map',grid),('/project/mission_status',String(data=json.dumps(dict(started_stamp=armed,target_detected=True)))),('/project/search_active',Bool(data=True)),('/camera_info',info),('/image_raw',image),('/odom',odom),('/amcl_pose',pose)]:pubs[topic].publish(msg)
    fixture.create_timer(.1,publish)
    executor=MultiThreadedExecutor(num_threads=6)
    for node in (fixture,perception,filt):executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert wait(lambda:filt.intersection.sites and len(filt.intersection.sites[0])>=4,timeout=15),frames[-1:]
        assert all(not frame['observations'] for frame in frames)
        origin[:]=[0.,1.,0.]
        assert wait(lambda:any(e.get('stable') and e.get('source')=='bearing_intersection' for e in estimates),timeout=15),estimates[-1:]
        result=next(e for e in reversed(estimates) if e.get('stable'))
        assert math.dist((result['x'],result['y']),(1.,1.))<.1,result
        assert result['sites']==2 and len(result['source_frames'])>=8,result
        assert len(filt.tracker.samples)==1
        print('PASS native bearing-only intersection -> shared MMSE',result['x'],result['y'],result['sigma'],flush=True)
    finally:
        executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        for node in (fixture,perception,filt):node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':main()
