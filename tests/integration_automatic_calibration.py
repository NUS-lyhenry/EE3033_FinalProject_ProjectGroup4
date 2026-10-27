"""ROS/HTTP automatic calibration on an isolated domain, with a virtual robot.

The actual node, HTTP API, TF, sensor subscriptions and final velocity gate run.
Physics is accelerated; no physical drivers, Nav2 actions or robot IO exist.
"""
import json
import copy
import math
import os
import multiprocessing
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen
from urllib.error import HTTPError

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan
from sensor_msgs.msg import Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ee3033_project.calibration import camera_model, predicted_points, load_profile
from ee3033_project.map_calibration import map_sample
from ee3033_project.calibration_node import CalibrationNode
from ee3033_project.console import Console
from ee3033_project.ros_utils import LATCHED, set_yaw
from ee3033_project.velocity_gate import VelocityGate

cv2.setNumThreads(1)

SETUP = dict(bottle_height=.25,bottle_diameter=.065,camera_height=.2655,camera_x=-.035,camera_y=0.)
CAMERA = camera_model(320,240,'test_optical',[260.,0.,160.,0.,260.,120.,0.,0.,1.],[0.]*5)


def wait(predicate, seconds=8):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if predicate():return True
        time.sleep(.03)
    return False


class Proposal:
    cls=np.array(0)
    def __init__(self, box):self.xyxy=[self];self.box=box
    def cpu(self):return self
    def numpy(self):return self.box


class Detector:
    names={0:'bottle'}
    visible=True
    duplicate=False
    def to(self, *args):return self
    def predict(self, image, **kwargs):
        pose=np.frombuffer(image[0,:24,0].copy().tobytes(),dtype=np.float64)
        sample=map_sample(SETUP,[.9,0.],pose,[])
        p=predicted_points([277.5,274.,math.radians(14.2),.035],SETUP,CAMERA,sample)
        box=np.array([p[2:,0].min(),p[1,1],p[2:,0].max(),p[0,1]])
        proposals=[Proposal(box)] if self.visible else []
        if self.visible and self.duplicate:proposals.append(Proposal(box+np.array([2.,0.,2.,0.])))
        return [SimpleNamespace(names=self.names,boxes=proposals)]


class Robot(Node):
    def __init__(self, telemetry, shift):
        super().__init__('automatic_virtual_robot')
        self.telemetry, self.map_shift = telemetry, shift
        self.pose=[0.,0.,0.];self.speed=[0.,0.];self.commands=[]
        self.frames=0;self.last=time.monotonic()
        self.create_subscription(Twist,'/cmd_vel',self.command,10)
        self.images=self.create_publisher(Image,'/image_raw',qos_profile_sensor_data)
        self.infos=self.create_publisher(CameraInfo,'/camera_info',qos_profile_sensor_data)
        self.odoms=self.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
        self.scans=self.create_publisher(LaserScan,'/scan_fixed',qos_profile_sensor_data)
        self.maps=self.create_publisher(OccupancyGrid,'/map',LATCHED)
        self.amcls=self.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',10)
        self.dynamic=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        # The lab image header uses an unrelated frame name without TF.
        # Physical mounting must come from camera_link, not that header.
        mount=TransformStamped();mount.header.frame_id='base_footprint';mount.child_frame_id='camera_link'
        mount.transform.translation.x=SETUP['camera_x'];mount.transform.translation.z=SETUP['camera_height'];mount.transform.rotation.w=1.
        self.static.sendTransform(mount)
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=grid.info.height=100;grid.info.resolution=.05
        grid.info.origin.position.x=grid.info.origin.position.y=-2.5;grid.info.origin.orientation.w=1.;grid.data=[0]*10000
        self.maps.publish(grid)
        self.create_timer(.05,self.tick)

    def command(self,msg):
        self.speed=[msg.linear.x,msg.angular.z];self.commands.append(self.speed)
        self.telemetry[3]=max(self.telemetry[3],abs(msg.linear.x))
        self.telemetry[4]=max(self.telemetry[4],abs(msg.angular.z))
        self.telemetry[5]=abs(msg.linear.x)+abs(msg.angular.z)

    def tick(self):
        now=time.monotonic();dt=min(.05,now-self.last)*5;self.last=now
        v,w=self.speed
        self.pose=[self.pose[0]+v*math.cos(self.pose[2])*dt,
                   self.pose[1]+v*math.sin(self.pose[2])*dt,self.pose[2]-w*dt]
        self.telemetry[:3]=self.pose
        stamp=self.get_clock().now().to_msg()
        odom=Odometry();odom.header.frame_id='odom';odom.header.stamp=stamp;odom.child_frame_id='base_footprint'
        odom.pose.pose.position.x,odom.pose.pose.position.y=self.pose[:2];set_yaw(odom.pose.pose.orientation,self.pose[2])
        odom.twist.twist.linear.x=v;odom.twist.twist.angular.z=-w;self.odoms.publish(odom)
        tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom';tf.header.stamp=copy.deepcopy(stamp)
        # AMCL normally publishes a transform slightly into the future.
        tf.header.stamp.nanosec+=500000000
        if tf.header.stamp.nanosec>=1000000000:tf.header.stamp.sec+=1;tf.header.stamp.nanosec-=1000000000
        tf.transform.rotation.w=1.;tf.transform.translation.x=self.map_shift.value;self.dynamic.sendTransform(tf)
        amcl=PoseWithCovarianceStamped();amcl.header.frame_id='map';amcl.header.stamp=stamp
        amcl.pose.pose.orientation.w=1.;amcl.pose.covariance[0]=amcl.pose.covariance[7]=.001;amcl.pose.covariance[35]=.002
        self.amcls.publish(amcl)
        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser';scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/360
        scan.range_min=.05;scan.range_max=5.;scan.ranges=[2.]*360;self.scans.publish(scan)
        self.frames+=1
        if self.frames%3==0:
            image=np.full((240,320,3),80,dtype=np.uint8)
            image[0,:24,0]=np.frombuffer(np.array(self.pose,dtype=np.float64).tobytes(),dtype=np.uint8)
            frame=CvBridge().cv2_to_imgmsg(image,'bgr8');frame.header.frame_id=CAMERA['frame_id'];frame.header.stamp=stamp;self.images.publish(frame)
            info=CameraInfo();info.header=frame.header;info.width=320;info.height=240;info.k=CAMERA['k'];info.d=CAMERA['d'];info.distortion_model='plumb_bob';self.infos.publish(info)



def virtual_process(kind, telemetry, shift, stop):
    rclpy.init()
    node = (Robot(telemetry, shift) if kind=='robot' else VelocityGate(parameter_overrides=[
        Parameter('allow_motion',value=True),Parameter('max_linear',value=.04),Parameter('max_angular',value=.15)]))
    try:
        while not stop.is_set():rclpy.spin_once(node,timeout_sec=.03)
    finally:
        node.destroy_node();rclpy.shutdown()


class AutomaticCalibrationHTTP(unittest.TestCase):
    def test_full_automatic_collection_save_and_fail_closed_stops(self):
        self.assertNotEqual(os.environ.get('ROS_DOMAIN_ID','0'),'6','Never use the physical robot domain')
        rclpy.init()
        with tempfile.TemporaryDirectory() as directory:
            detector=Detector()
            with patch.dict(sys.modules,{'torch':SimpleNamespace(set_num_threads=lambda _:None),'ultralytics':SimpleNamespace(YOLO=lambda _:detector)}):
                node=CalibrationNode(parameter_overrides=[Parameter('port',value=0),Parameter('map_file',value='/maps/test.yaml'),Parameter('allow_motion',value=True),Parameter('output_directory',value=directory)])
            console=Console(parameter_overrides=[Parameter('port',value=0),
                Parameter('phase',value='calibration'),Parameter('session_directory',value=directory+'/assessment'),
                Parameter('calibration_directory',value=directory),Parameter('calibration_port',value=node.p['port'])])
            console.stage=SimpleNamespace(process=True,loading=False,cancel=threading.Event(),
                calibration_file='',stop=lambda:None,
                snapshot=lambda:dict(managed=True,phase='calibration',running=True,transitioning=False,fault=None))
            console.selected_map=dict(id='test/map.yaml',path='/maps/test.yaml')
            node.p['parent_port']=console.p['port']
            processes=multiprocessing.get_context('spawn')
            telemetry=processes.Array('d',6);shift=processes.Value('d',0.);stop=processes.Event()
            children=[processes.Process(target=virtual_process,args=(kind,telemetry,shift,stop)) for kind in ('robot','gate')]
            for child in children:child.start()
            executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(console)
            thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
            base=f'http://127.0.0.1:{node.p["port"]}'
            def post(name,**body):
                request=Request(base+'/api/calibration/'+name,json.dumps({'generation':node.motion.generation,**body}).encode(),
                    {'Content-Type':'application/json','X-Project-Token':node.token},method='POST')
                deadline=time.monotonic()+8
                while True:
                    try:
                        return json.load(urlopen(request,timeout=5))
                    except HTTPError as exc:
                        error=exc.read().decode()
                        if (name=='start' and time.monotonic()<deadline and
                                any(text in error for text in ('bounded balancing', 'Fresh sensors', 'fresh bottle image'))):
                            time.sleep(.05)
                            continue
                        raise AssertionError(error) from exc
            def ready():
                with node.lock:
                    return node.capture_status()['ready'] and node.camera is not None and node.target() is not None and node.detections[-1].get('pose') is not None
            heartbeat_stop=threading.Event()
            heartbeat_requests=[]
            def pulse():
                # Like browser setInterval(fetch), the next pulse does not wait
                # for an earlier HTTP response to finish.
                while not heartbeat_stop.wait(.12):
                    if node.motion.mode=='auto':
                        worker=threading.Thread(target=lambda:post('heartbeat'),daemon=True)
                        worker.start();heartbeat_requests.append(worker)
            heartbeat_thread=threading.Thread(target=pulse,daemon=True)
            try:
                self.assertTrue(wait(ready,15),str(node.snapshot()))
                # Exercise the main console's authenticated forwarding path.
                self.assertTrue(wait(lambda:console.calibration_live and console.calibration_live.get('ready')))
                parent=f'http://127.0.0.1:{console.p["port"]}'
                request=Request(parent+'/api/calibration_target',json.dumps(dict(map_id='test/map.yaml',x=.9,y=0.)).encode(),
                    {'Content-Type':'application/json','X-Project-Token':console.token,
                     'X-Control-Generation':str(console.control_generation)},method='POST')
                self.assertTrue(json.load(urlopen(request,timeout=4))['success'])
                self.assertEqual(node.bottle_point['xy'],[.9,0.])
                with self.assertRaisesRegex(ValueError,'different map'):
                    console.set_calibration_target(dict(map_id='old/map.yaml',x=.9,y=0.))
                # Bad tokens/origins cannot even issue Stop.
                for headers in ({'X-Project-Token':'bad'}, {'X-Project-Token':node.token,'Origin':'https://untrusted.invalid'}):
                    with self.assertRaises(HTTPError) as denied:
                        urlopen(Request(base+'/api/calibration/stop',b'{}',headers,method='POST'),timeout=2)
                    self.assertEqual(denied.exception.code,403)
                with self.assertRaisesRegex(AssertionError,'currently loaded map'):
                    post('target',map_file='/maps/old.yaml',xy=[.9,0.])
                with self.assertRaisesRegex(AssertionError,'Unknown calibration command'):
                    post('automatic',distance=.9)
                with self.assertRaisesRegex(AssertionError,'context changed'):
                    post('start',generation=-1,bottle_height=.25,bottle_diameter=.065)
                post('start',bottle_height=.25,bottle_diameter=.065)
                with self.assertRaisesRegex(AssertionError,'Stop calibration'):
                    post('target',map_file='/maps/test.yaml',xy=[1.,0.])
                heartbeat_thread.start()
                deadline=time.monotonic()+160
                injected_miss=False
                injected_ambiguity=False
                while node.auto_status['state']=='collecting' and time.monotonic()<deadline:
                    if not injected_miss and len(node.auto_views)>=3 and telemetry[5]>0:
                        # Detection loss must not steer map-based motion. It may
                        # delay capture at the next stationary goal.
                        detector.visible=False
                        time.sleep(.35)
                        self.assertEqual(node.motion.mode,'auto')
                        detector.visible=True
                        injected_miss=True
                    elif injected_miss and not injected_ambiguity and len(node.auto_views)>=5:
                        # Duplicate overlapping proposals describe one bottle.
                        detector.duplicate=True
                        time.sleep(.6)
                        self.assertEqual(node.motion.mode,'auto')
                        detector.duplicate=False
                        injected_ambiguity=True
                    time.sleep(.1)
                self.assertTrue(injected_miss,'No moving detector-miss recovery was exercised')
                self.assertTrue(injected_ambiguity,'No ambiguous-box recovery was exercised')
                self.assertTrue(wait(lambda:node.auto_status['state']!='fitting',10))
                self.assertEqual(node.auto_status['state'],'complete',str(node.snapshot()))
                profile=load_profile(node.saved)
                self.assertEqual(len(profile['map_calibration']['observations']),17)
                self.assertAlmostEqual(math.degrees(profile['pitch']),14.2,delta=.3)
                self.assertEqual(len(profile['validation']),2)
                self.assertTrue(all(v['passed'] for v in profile['validation']))
                self.assertLessEqual(telemetry[3],.04);self.assertLessEqual(telemetry[4],.15)
                heartbeat_stop.set();heartbeat_thread.join(timeout=2)
                for worker in heartbeat_requests:worker.join(timeout=2)
                self.assertTrue(wait(ready))
                # A failed preflight must preserve the already saved draft.
                previous_setup=dict(node.setup)
                grid=node.map_grid
                col=int((telemetry[0]-grid['origin'][0])/grid['resolution'])
                row=int((telemetry[1]-grid['origin'][1])/grid['resolution'])
                cell=row*grid['width']+col
                grid['cells'][cell]=100
                try:
                    with self.assertRaisesRegex(AssertionError,'clear'):
                        post('start',bottle_height=.30,bottle_diameter=.065)
                    self.assertEqual(node.setup,previous_setup)
                    self.assertEqual(node.auto_status['state'],'complete')
                finally:
                    grid['cells'][cell]=0
                # The robot is farther from the fixed point now. A new plan is
                # computed, rather than reusing start coordinates or distance.
                post('start',bottle_height=.25,bottle_diameter=.065)
                self.assertTrue(wait(lambda:node.motion.mode=='idle',2))
                self.assertIn('heartbeat',node.auto_status['reason'])
                self.assertTrue(wait(ready))
                post('start',bottle_height=.25,bottle_diameter=.065)
                shift.value=.3
                self.assertTrue(wait(lambda:node.motion.mode=='idle',2))
                self.assertIn('Localization shifted',node.auto_status['reason'])
                self.assertTrue(wait(lambda:telemetry[5]==0,2))
                print('PASS automatic HTTP + timestamped ROS poses + future AMCL TF + 17 map views + inverted yaw + saved checks + heartbeat/localization stop')
            finally:
                heartbeat_stop.set()
                if heartbeat_thread.is_alive():heartbeat_thread.join(timeout=2)
                node.command('/api/calibration/stop',{})
                stop.set()
                for child in children:
                    child.join(timeout=5)
                    if child.is_alive():child.terminate();child.join(timeout=3)
                executor.shutdown(timeout_sec=3);thread.join(timeout=3)
                console.destroy_node();node.destroy_node();rclpy.shutdown()



class CalibrationPoseHandoff(unittest.TestCase):
    def fixture(self, age=0.):
        from ee3033_project.console_workflow import Workflow
        now=time.time()
        transform=TransformStamped()
        transform.header.stamp.sec=int(now-age)
        transform.header.stamp.nanosec=int(((now-age)%1)*1e9)
        transform.transform.translation.x=.6;transform.transform.translation.y=.4
        set_yaw(transform.transform.rotation,.25)
        class StopBeforeLaunch(Exception):pass
        stopped=[]
        def stop():
            stopped.append(True)
            raise StopBeforeLaunch()
        stage=SimpleNamespace(process=True,calibration_initial_pose=None,cancel=threading.Event(),loading=False,stop=stop)
        fake=SimpleNamespace(lock=threading.RLock(),stage=stage,p=dict(phase='mapping'),saved_map='/maps/current/map.yaml',selected_map=None,
            freshness={'Mission':time.monotonic()},tf=SimpleNamespace(lookup_transform=lambda *a:transform),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(now*1e9))),
            catalog=SimpleNamespace(entry=lambda _:dict(id='current/map.yaml',path='/maps/current/map.yaml',reviewed=True)),
            require_idle=lambda:None,require_camera_idle=lambda:None,require_zero_command=lambda:None,
            record=SimpleNamespace(snapshot=lambda:dict(finished=None,interrupted=False)),manual_pub=SimpleNamespace(publish=lambda _:None))
        return Workflow,fake,stopped,StopBeforeLaunch

    def test_fresh_slam_pose_is_captured_before_mapping_shutdown(self):
        workflow,fake,stopped,sentinel=self.fixture()
        with self.assertRaises(sentinel):workflow.workflow_command(fake,'/api/stage',dict(phase='calibration',map_id='current/map.yaml'))
        self.assertEqual(stopped,[True])
        np.testing.assert_allclose(fake.stage.calibration_initial_pose,[.6,.4,.25])

    def test_stale_slam_pose_preserves_the_mapping_stack(self):
        workflow,fake,stopped,_=self.fixture(age=2.)
        with self.assertRaisesRegex(ValueError,'fresh mapping pose'):
            workflow.workflow_command(fake,'/api/stage',dict(phase='calibration',map_id='current/map.yaml'))
        self.assertFalse(stopped)


if __name__=='__main__':unittest.main()
