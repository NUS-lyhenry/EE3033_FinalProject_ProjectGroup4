"""Local browser operator console; ROS callbacks and HTTP run on separate threads.

Bind only to loopback. For a remote Pi use SSH port forwarding, not an exposed
unauthenticated robot-control port. Manual control exists only in mapping mode.
"""
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import subprocess
import tempfile
import threading
import time
from urllib.parse import urlparse

import cv2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from .core import quaternion_yaw
from .map_io import save_new_map
from .ros_utils import LATCHED, parameters, set_yaw


class Console(Node):
    def __init__(self, **kwargs):
        super().__init__('project_console', **kwargs)
        self.p = parameters(self, {'port': 8765, 'phase': 'mission', 'allow_motion': False,
                                   'map_directory': '~/ee3033_maps'})
        if self.p['phase'] not in ('mapping','mission'):
            raise ValueError('phase must be mapping or mission')
        self.lock = threading.Lock()
        self.token = secrets.token_urlsafe(24)
        self.state = {'state':'WAITING','reason':'Waiting for mission status',
                      'target':None,'home':None}
        self.map_data = None
        self.map_revision = 0
        self.route = self.path = []
        self.events = deque(maxlen=20)
        self.freshness = {}
        self.jpeg = {}
        self.jpeg_time = {}
        self.last_encoding = {}
        self.amcl_covariance = None
        self.saved_map = None
        self.manual = (0.,0.,0.)
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.manual_pub = self.create_publisher(Twist,'/project/manual_cmd_vel',10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.command_clients = {k:self.create_client(Trigger,'/project/'+k) for k in ('start','stop')}
        self.create_subscription(String,'/project/mission_status',self.on_status,LATCHED)
        self.create_subscription(String,'/project/perception_status',
                                 lambda m:self.mark('YOLO'),10)
        self.create_subscription(LaserScan,'/scan_fixed',lambda m:self.mark('Lidar'),qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_amcl,10)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,LATCHED)
        self.create_subscription(NavPath,'/project/search_path',self.on_route,LATCHED)
        self.create_subscription(NavPath,'/plan',self.on_path,10)
        for source,topic in [('project','/project/detection_image'),('lab','/detector/result_image')]:
            self.create_subscription(Image,topic,lambda m,s=source:self.on_image(m,s),
                                     qos_profile_sensor_data)
        self.create_timer(0.05,self.teleop_tick)
        node = self
        asset = Path(get_package_share_directory('ee3033_project'))/'web'/'index.html'
        self.html = asset.read_bytes()

        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):
                pass

            def reply(self,code,data,kind='application/json'):
                body = json.dumps(data,allow_nan=False).encode() if kind=='application/json' else data
                self.send_response(code)
                self.send_header('Content-Type',kind)
                self.send_header('Content-Length',str(len(body)))
                self.send_header('Cache-Control','no-store')
                self.send_header('X-Content-Type-Options','nosniff')
                self.send_header('X-Frame-Options','DENY')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = urlparse(self.path).path
                if path=='/':
                    return self.reply(200,node.html,'text/html; charset=utf-8')
                if path=='/api/state':
                    return self.reply(200,node.snapshot())
                if path=='/api/map':
                    with node.lock:
                        data = node.map_data
                    return self.reply(200,data)
                if path=='/api/image.jpg':
                    with node.lock:
                        source = node.image_source()
                        data = node.jpeg.get(source)
                    return self.reply(200,data,'image/jpeg') if data else self.reply(404,{'error':'No live image'})
                return self.reply(404,{'error':'Unknown endpoint'})

            def do_POST(self):
                if not secrets.compare_digest(self.headers.get('X-Project-Token',''),node.token):
                    return self.reply(403,{'error':'Missing console session token'})
                origin = self.headers.get('Origin')
                allowed = {f'http://127.0.0.1:{node.p["port"]}',f'http://localhost:{node.p["port"]}'}
                if origin and origin not in allowed:
                    return self.reply(403,{'error':'Cross-origin control rejected'})
                try:
                    length = int(self.headers.get('Content-Length','0'))
                    if not 0<=length<=8192:
                        raise ValueError('Request too large')
                    body = json.loads(self.rfile.read(length) or b'{}')
                    result = node.command(urlparse(self.path).path,body)
                    return self.reply(200,result)
                except (ValueError,TypeError,KeyError) as exc:
                    return self.reply(400,{'error':str(exc)})
                except (RuntimeError,OSError,subprocess.SubprocessError) as exc:
                    return self.reply(503,{'error':str(exc)})

        self.server = ThreadingHTTPServer(('127.0.0.1',self.p['port']),Handler)
        self.p['port'] = self.server.server_address[1]
        self.server.daemon_threads = True
        self.http_thread = threading.Thread(target=self.server.serve_forever,daemon=True)
        self.http_thread.start()
        self.get_logger().info(f'Operator console: http://127.0.0.1:{self.p["port"]}')

    def mark(self,name):
        with self.lock:
            self.freshness[name] = time.monotonic()

    def on_status(self,msg):
        try:
            state = json.loads(msg.data)
            with self.lock:
                if state['state'] != self.state['state']:
                    self.events.appendleft({'time':time.strftime('%H:%M:%S'),
                                             'state':state['state'],'reason':state['reason']})
                self.state = state
                self.freshness['Mission'] = time.monotonic()
        except (ValueError,KeyError):
            pass

    def on_amcl(self,msg):
        with self.lock:
            self.amcl_covariance = [msg.pose.covariance[i] for i in (0,7,35)]
            self.freshness['AMCL'] = time.monotonic()

    def on_map(self,msg):
        with self.lock:
            self.map_revision += 1
            origin = msg.info.origin
            self.map_data = {'width':msg.info.width,'height':msg.info.height,
                             'resolution':msg.info.resolution,
                             'origin':[origin.position.x,origin.position.y,
                                       quaternion_yaw(origin.orientation)],
                             'cells':list(msg.data)}
            self.freshness['Map'] = time.monotonic()

    def on_route(self,msg):
        with self.lock:
            self.route = [[p.pose.position.x,p.pose.position.y] for p in msg.poses]

    def on_path(self,msg):
        with self.lock:
            self.path = [[p.pose.position.x,p.pose.position.y] for p in msg.poses]

    def on_image(self,msg,source):
        now = time.monotonic()
        if now-self.last_encoding.get(source,0)<0.35:
            return
        self.last_encoding[source] = now
        try:
            image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
            ok,jpeg = cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,78])
            if ok:
                with self.lock:
                    self.jpeg[source],self.jpeg_time[source] = jpeg.tobytes(),now
        except (ValueError,cv2.error):
            pass

    def image_source(self):
        now = time.monotonic()
        for source in ('project','lab'):
            if now-self.jpeg_time.get(source,-math.inf)<3:
                return source
        return None

    def snapshot(self):
        pose = None
        try:
            t = self.tf.lookup_transform('map','base_footprint',Time())
            pose = [t.transform.translation.x,t.transform.translation.y,
                    quaternion_yaw(t.transform.rotation)]
        except TransformException:
            pass
        with self.lock:
            now = time.monotonic()
            health = {name:{'age':round(now-t,1),'ok':now-t<limit}
                      for name,t in self.freshness.items()
                      for limit in [3 if name in ('YOLO','Mission') else
                                    30 if name=='AMCL' else 3 if name=='Lidar' else math.inf]}
            return {'mission':self.state,'phase':self.p['phase'],
                    'allow_motion':self.p['allow_motion'],'pose':pose,
                    'map_revision':self.map_revision,'route':self.route,'path':self.path,
                    'health':health,'amcl_covariance':self.amcl_covariance,
                    'camera_source':self.image_source(),'events':list(self.events),
                    'saved_map':self.saved_map,'token':self.token}

    def command(self,path,body):
        with self.lock:
            state = self.state['state']
            active = state in ('PLANNING','SEARCHING','OBSERVING','CANCELING','RETURNING','VERIFYING','MAPPING','SAVING')
        if path in ('/api/start','/api/stop'):
            key = path.rsplit('/',1)[-1]
            if key=='start' and not self.p['allow_motion']:
                raise ValueError('Motion disabled in this console')
            if key=='stop':
                self.manual = (0.,0.,0.)
                self.manual_pub.publish(Twist())
            client = self.command_clients[key]
            if not client.wait_for_service(timeout_sec=1):
                raise RuntimeError('Mission service is not available')
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic()+5
            while not future.done() and time.monotonic()<deadline:
                time.sleep(0.02)
            if not future.done():
                raise RuntimeError('Mission service timed out; inspect mission status')
            result = future.result()
            return {'success':result.success,'message':result.message}
        if path=='/api/initial_pose':
            if active or self.p['phase']!='mission':
                raise ValueError('Initial pose is only available before the autonomous mission')
            with self.lock:
                fresh = time.monotonic()-self.freshness.get('Mission',-math.inf)<2.0
            if not fresh:
                raise ValueError('Mission status is stale; initial pose changes are locked')
            x,y,yaw = [float(body[k]) for k in ('x','y','yaw')]
            if not all(math.isfinite(v) for v in (x,y,yaw)):
                raise ValueError('Initial pose must be finite')
            with self.lock:
                m = self.map_data
            if not m:
                raise ValueError('Wait for the saved map')
            dx,dy = x-m['origin'][0],y-m['origin'][1]
            a = m['origin'][2]
            c = math.floor((math.cos(a)*dx+math.sin(a)*dy)/m['resolution'])
            r = math.floor((-math.sin(a)*dx+math.cos(a)*dy)/m['resolution'])
            if not (0<=c<m['width'] and 0<=r<m['height']) or m['cells'][r*m['width']+c]!=0:
                raise ValueError('Select a free cell inside the saved map')
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id,msg.header.stamp = 'map',self.get_clock().now().to_msg()
            msg.pose.pose.position.x,msg.pose.pose.position.y = x,y
            set_yaw(msg.pose.pose.orientation,yaw)
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.04
            msg.pose.covariance[35] = 0.12
            self.initial_pub.publish(msg)
            return {'success':True,'message':'Assigned initial pose sent to AMCL'}
        if path=='/api/teleop':
            if self.p['phase']!='mapping' or active or not self.p['allow_motion']:
                raise ValueError('Manual driving is restricted to the mapping stage')
            x,z = float(body.get('linear',0)),float(body.get('angular',0))
            if not math.isfinite(x) or not math.isfinite(z):
                raise ValueError('Invalid velocity')
            self.manual = (max(-0.10,min(0.10,x)),max(-0.35,min(0.35,z)),time.monotonic())
            return {'success':True}
        if path=='/api/save_map':
            if self.p['phase']!='mapping' or active:
                raise ValueError('Map saving is only available during the mapping stage')
            self.manual = (0.,0.,0.)
            self.manual_pub.publish(Twist())
            saved = save_new_map(self.p['map_directory'])
            with self.lock:
                self.saved_map = saved
            return {'success':True,'message':'Map saved to '+self.saved_map,'path':self.saved_map}
        raise ValueError('Unknown command')

    def teleop_tick(self):
        if self.p['phase']=='mapping':
            x,z,stamp = self.manual
            cmd = Twist()
            if time.monotonic()-stamp<0.25:
                cmd.linear.x,cmd.angular.z = x,z
            self.manual_pub.publish(cmd)

    def destroy_node(self):
        if self.context.ok():
            self.manual_pub.publish(Twist())
        self.server.shutdown()
        self.server.server_close()
        self.http_thread.join(timeout=2)
        super().destroy_node()


def main():
    from .ros_utils import run
    run(Console)
