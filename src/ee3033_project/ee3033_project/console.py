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
import numpy as np
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from .core import quaternion_yaw, stamp_seconds
from .motion_safety import FOOTPRINT_RADIUS
from .robot_geometry import geometry_metadata
from .map_io import save_new_map
from .ros_utils import LATCHED, parameters, set_yaw
from .trial import LIMITS, PROFILES, parameter_plan, validate_trial
from .vision_geometry import validate_mount_settings
from .assessment import atomic_json
from . import gap
from .console_workflow import Workflow
from .model_labels import ModelLabels
from .waypoint import validate_waypoint
from .console_scan import ScanFilterSettings
from .console_attitude import AttitudeSettings


class Console(AttitudeSettings, ScanFilterSettings, Workflow, Node):
    def __init__(self, **kwargs):
        super().__init__('project_console', **kwargs)
        self.p = parameters(self, {'port': 8765, 'phase': 'mission', 'allow_motion': False,
                                   'map_directory': '~/ee3033_maps',
                                   'session_directory':'~/ee3033_results/assessment',
                                   'manage_stages':False,'stage_base':True,'stage_lidar':True,
                                   'stage_camera':True,'model':'/home/ros2_ws/weights/yolo26n.pt',
                                   'software_version':'development','calibration_file':'',
                                   'calibration_directory':'~/ee3033_calibrations','calibration_port':8780,
                                   'navigation_profile':'original','vision_config':'',
                                   'mapping_resolution':.05})
        if self.p['phase'] not in ('mapping','mission','calibration'):
            raise ValueError('phase must be mapping or mission')
        self.lock = threading.Lock()
        self.control_lock = threading.Lock()
        self.trial_fault = None
        self.gap_fault = None
        # Read-only discovery failures recover after a complete synchronized
        # read. Uncertain updates/rollback still require a verified reapply.
        self.gap_reapply_required = False
        self.control_generation=0
        self.token = secrets.token_urlsafe(24)
        self.model_labels=ModelLabels(self.p['model'])
        self.state = {'state':'WAITING','reason':'Waiting for mission status',
                      'target':None,'home':None}
        self.map_data = None
        self.map_revision = 0
        self.target_estimate = None
        self.route = self.path = []
        self.local_path=[]
        self.local_path_time=-math.inf
        self.travelled=deque(maxlen=1500)
        self.navigation_status={}
        self.simulation_motion={}
        self.events = deque(maxlen=20)
        self.freshness = {}
        self.jpeg = {}
        self.jpeg_time = {}
        self.last_encoding = {}
        self.amcl_covariance = None
        self.battery_voltage = None
        self.saved_map = None
        self.manual = (0.,0.,0.)
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.manual_pub = self.create_publisher(Twist,'/project/manual_cmd_vel',10)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.command_clients = {k:self.create_client(Trigger,'/project/'+k) for k in ('start','stop','start_waypoint')}
        node_names=set(parameter_plan(dict(max_linear=.12,max_angular=.4,goal_timeout=60.)))
        node_names.update(parameter_plan(dict(max_linear=.12,max_angular=.4,goal_timeout=60.), 'mission'))
        node_names.update(gap.parameter_plan(False,'mapping'))
        node_names.update(gap.parameter_plan(False,'mission'))
        node_names.add('project_scan_resampler')
        self.parameter_clients = {
            (name,operation):self.create_client(kind,f'/{name}/{operation}')
            for name in node_names for operation,kind in [
                ('get_parameters',GetParameters),('set_parameters_atomically',SetParametersAtomically)]}
        self.init_workflow()
        self.init_scan_filter()
        self.init_attitude()
        self.gate_status = None
        self.create_subscription(String, '/project/velocity_gate_status', self.on_gate_status, 10)
        self.create_subscription(String,'/project/mission_status',self.on_status,LATCHED)
        self.create_subscription(String,'/project/perception_status',self.on_perception_status,10)
        self.create_subscription(String,'/project/target_estimate',self.on_target_estimate,LATCHED)
        self.create_subscription(LaserScan,'/scan_fixed',lambda m:self.mark('Lidar'),qos_profile_sensor_data)
        self.create_subscription(Float32,'/power_voltage',self.on_voltage,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_amcl,10)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,LATCHED)
        self.create_subscription(NavPath,'/project/search_path',self.on_route,LATCHED)
        self.create_subscription(NavPath,'/plan',self.on_path,10)
        self.create_subscription(NavPath,'/local_plan',self.on_local_path,10)
        self.create_subscription(NavPath,'/lookahead_arc',self.on_local_path,10)
        self.create_subscription(NavPath,'/lookahead_collision_arc',self.on_local_path,10)
        self.create_subscription(String,'/project/navigation_status',self.on_navigation_status,10)
        self.create_subscription(String,'/project/simulation_motion',self.on_simulation_motion,10)
        for source,topic in [('project','/project/detection_image'),('lab','/detector/result_image')]:
            self.create_subscription(Image,topic,lambda m,s=source:self.on_image(m,s),
                                     qos_profile_sensor_data)
        self.create_timer(0.05,self.teleop_tick)
        node = self
        asset = Path(get_package_share_directory('ee3033_project'))/'web'/'index.html'
        self.html = asset.read_bytes()
        self.assets = {'/app.js':((asset.parent/'app.js').read_bytes(),'text/javascript; charset=utf-8'),
                       '/style.css':((asset.parent/'style.css').read_bytes(),'text/css; charset=utf-8')}

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
                if path in node.assets:
                    data,kind=node.assets[path]
                    return self.reply(200,data,kind)
                if path in ('/api/maps','/api/result'):
                    return self.reply(200,node.workflow_get(path))
                if path=='/api/state':
                    return self.reply(200,node.snapshot())
                if path=='/api/labels':
                    return self.reply(200,node.model_labels.read())
                if path in ('/api/trial','/api/gap','/api/scan_filter','/api/attitude'):
                    try:
                        with node.control_lock:
                            data = (node.attitude_settings() if path=='/api/attitude' else
                                    node.scan_filter_settings() if path=='/api/scan_filter' else
                                    node.trial_settings() if path=='/api/trial' else node.gap_settings())
                        return self.reply(200,data)
                    except (RuntimeError,ValueError) as exc:
                        return self.reply(503,{'error':str(exc)})
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
                    path = urlparse(self.path).path
                    # Stop never waits behind parameter service requests.
                    if path=='/api/stop':
                        result = node.command(path,body)
                    else:
                        with node.control_lock:
                            if node.stage and self.headers.get('X-Control-Generation')!=str(node.control_generation):
                                raise ValueError('Control context changed; refresh state before retrying')
                            result = node.command(path,body,expected_generation=int(self.headers.get('X-Control-Generation',node.control_generation)))
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

    def on_gate_status(self, msg):
        try:
            data = json.loads(msg.data)
            if 0 <= self.get_clock().now().nanoseconds*1e-9-float(data['stamp']) < 2.:
                with self.lock:
                    self.gate_status = data
        except (ValueError, TypeError, KeyError):
            pass

    def on_status(self,msg):
        try:
            state = json.loads(msg.data)
            if self.stage and (self.stage.process is None or float(state.get('stamp',-math.inf))<self.status_after):
                return
            if state.get('phase') and state['phase']!=self.p['phase']:
                return
            if (self.p['phase']=='mission' and self.retry_after_stamp>-math.inf
                    and float(state.get('started_stamp',-math.inf))<=self.retry_after_stamp):
                if not (self.retry_stop_generation is not None and state.get('state') in ('CANCELING','STOPPED')):
                    return
            with self.lock:
                if state['state'] != self.state['state']:
                    self.events.appendleft({'time':time.strftime('%H:%M:%S'),
                                             'state':state['state'],'reason':state['reason']})
                self.state = state
                self.freshness['Mission'] = time.monotonic()
            self.workflow_status(state)
        except (ValueError,TypeError,KeyError):
            pass

    def on_amcl(self,msg):
        with self.lock:
            self.amcl_covariance = [msg.pose.covariance[i] for i in (0,7,35)]
            self.freshness['AMCL'] = time.monotonic()

    def on_target_estimate(self, msg):
        try:
            data=json.loads(msg.data)
            if (self.retry_after_stamp>-math.inf
                    and float(data.get('mission_stamp',-math.inf))<=self.retry_after_stamp):return
            if data.get('available') is False:
                with self.lock:self.target_estimate=None
                return
            if (data.get('method')!='map-constrained MMSE' or data.get('frame_id')!='map'
                    or not all(math.isfinite(float(data[k])) for k in ('stamp','x','y','sigma'))
                    or data['sigma']<=0):return
            with self.lock:
                if self.p['phase']=='mission':self.target_estimate=data
        except (ValueError,TypeError,KeyError):pass

    def displayed_target_estimate(self):
        target=self.state.get('target')
        if target and target.get('estimator')=='map-constrained MMSE':
            age=max(0.,self.get_clock().now().nanoseconds*1e-9-target['observation_stamp'])
            return dict(target,label=target['class'],method=target['estimator'],
                        stamp=target['observation_stamp'],confirmed=True,
                        stable=target.get('measurement_stable',target.get('quality')=='preferred_precision'),
                        age=age,samples=target.get('confirmation_frames',4),stale=age>2.,frame_id='map')
        if not self.target_estimate:return None
        data=self.target_estimate
        age=self.get_clock().now().nanoseconds*1e-9-data['stamp']
        if not data.get('confirmed') and not 0<=age<=30:return None
        return dict(data,age=max(0.,age),stale=not data.get('confirmed') and age>2.)

    def on_voltage(self,msg):
        if math.isfinite(msg.data) and 0 <= msg.data <= 60:
            with self.lock:
                self.battery_voltage = float(msg.data)
                self.freshness['Battery'] = time.monotonic()

    def on_map(self,msg):
        with self.lock:
            self.target_estimate=None
            self.map_revision += 1
            origin = msg.info.origin
            self.map_data = {'width':msg.info.width,'height':msg.info.height,
                             'resolution':msg.info.resolution,
                             'origin':[origin.position.x,origin.position.y,
                                       quaternion_yaw(origin.orientation)],
                             'cells':list(msg.data)}
            self.freshness['Map'] = time.monotonic()

    def on_route(self,msg):
        if self.retry_after_stamp>-math.inf and stamp_seconds(msg.header.stamp)<=self.retry_after_stamp:return
        with self.lock:
            self.route = [[p.pose.position.x,p.pose.position.y] for p in msg.poses]

    def on_path(self,msg):
        if self.retry_after_stamp>-math.inf and stamp_seconds(msg.header.stamp)<=self.retry_after_stamp:return
        with self.lock:
            self.path = [[p.pose.position.x,p.pose.position.y] for p in msg.poses]

    def on_local_path(self,msg):
        if self.retry_after_stamp>-math.inf and stamp_seconds(msg.header.stamp)<=self.retry_after_stamp:return
        try:
            if msg.header.frame_id=='map':
                points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]
            else:
                t=self.tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
                a=quaternion_yaw(t.transform.rotation);c,ss=math.cos(a),math.sin(a)
                tr=t.transform.translation
                points=[[tr.x+c*p.pose.position.x-ss*p.pose.position.y,
                         tr.y+ss*p.pose.position.x+c*p.pose.position.y] for p in msg.poses]
            if not all(math.isfinite(v) for point in points for v in point):return
            with self.lock:
                self.local_path=points[:250]
                self.local_path_time=time.monotonic()
        except TransformException:
            pass

    def on_navigation_status(self,msg):
        try:
            status=json.loads(msg.data)
            if not isinstance(status,dict):return
            with self.lock:
                self.navigation_status=status
                self.freshness['Navigation']=time.monotonic()
        except (ValueError,TypeError):
            pass

    def on_simulation_motion(self,msg):
        try:
            status=json.loads(msg.data)
            if not isinstance(status,dict):return
            with self.lock:self.simulation_motion=status
        except (ValueError,TypeError):
            pass

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
            if pose and (not self.travelled or math.dist(pose[:2],self.travelled[-1])>=.02):
                self.travelled.append(list(pose[:2]))
            health = {name:{'age':round(now-t,1),'ok':now-t<limit}
                      for name,t in self.freshness.items()
                      for limit in [3 if name in ('YOLO','Mission') else
                                    30 if name=='AMCL' else 5 if name=='Battery' else
                                    1 if name=='Odometry' else 3 if name=='Lidar' else math.inf]}
            battery_age = now-self.freshness.get('Battery',-math.inf)
            return {'mission':self.state,'phase':self.p['phase'],
                    'allow_motion':self.p['allow_motion'],'pose':pose,
                    'calibration_file':self.p['calibration_file'] or None,
                    'velocity_gate': self.gate_status,
                    'target_estimate': self.displayed_target_estimate(),
                    'map_revision':self.map_revision,'route':self.route,'path':self.path,
                    'local_path':self.local_path if now-self.local_path_time<=1.5 else [],
                    'travelled_path':list(self.travelled),
                    'footprint_radius':FOOTPRINT_RADIUS,'robot_geometry':geometry_metadata(),
                    'navigation':self.navigation_status,
                    'scan_filter':self.scan_filter_snapshot(),
                    'attitude':self.attitude_snapshot(),
                    'simulation_motion':self.simulation_motion if hasattr(self,'simulation_robot') else None,
                    'health':health,'amcl_covariance':self.amcl_covariance,
                    'camera_source':self.image_source(),'events':list(self.events),
                    'control_fault':self.trial_fault or self.gap_fault or self.workflow_fault or self.scan_filter_fault,
                    **self.workflow_snapshot(),
                    'battery':{'voltage':self.battery_voltage,
                               'age':round(battery_age,1) if math.isfinite(battery_age) else None,
                               'fresh':battery_age<5},
                    'saved_map':self.saved_map,'token':self.token,'control_generation':self.control_generation}

    def parameter_request(self, name, operation, request):
        client = self.parameter_clients[(name,operation)]
        if not client.wait_for_service(timeout_sec=.4):
            raise RuntimeError(f'{name}: parameter service unavailable')
        future = client.call_async(request)
        deadline = time.monotonic()+2
        while not future.done() and time.monotonic()<deadline:
            time.sleep(.01)
        if not future.done():
            raise RuntimeError(f'{name}: parameter request timed out')
        result = future.result()
        if result is None:
            raise RuntimeError(f'{name}: parameter service failed')
        return result

    def read_trial_nodes(self):
        plan = parameter_plan(dict(max_linear=.12,max_angular=.4,goal_timeout=60.), self.p['phase'])
        result = {}
        for name, entries in plan.items():
            names = list(entries)
            if name=='project_velocity_gate':
                names += ['stop_distance','rotation_clearance']
            response = self.parameter_request(name,'get_parameters',GetParameters.Request(names=names))
            if len(response.values)!=len(names):
                raise RuntimeError(f'{name}: incomplete parameter readback')
            result[name] = {key:Parameter.from_parameter_msg(ParameterMessage(name=key,value=value)).value
                            for key,value in zip(names,response.values)}
            if any(type(value) is not float or not math.isfinite(value)
                   for value in result[name].values()):
                raise RuntimeError(f'{name}: invalid parameter readback')
        return result

    def write_trial_node(self, name, values):
        request = SetParametersAtomically.Request(parameters=[
            Parameter(key,value=value).to_parameter_msg() for key,value in values.items()])
        result = self.parameter_request(name,'set_parameters_atomically',request).result
        if not result.successful:
            raise RuntimeError(f'{name}: {result.reason}')

    @staticmethod
    def trial_matches(actual, expected):
        return all(math.isclose(actual[name][key],value,abs_tol=1e-8)
                   for name,entries in expected.items() for key,value in entries.items())

    def trial_settings(self, values=None):
        if self.p['phase'] not in ('mapping', 'mission'):
            raise ValueError('Navigation profiles require mapping or mission mode')
        if values is not None:
            values = validate_trial(values)
            with self.lock:
                if (self.state['state'] not in ('WAITING','STOPPED','FAILED','MAPPED','MAPPED_PARTIAL','WAYPOINT_REACHED')
                        or time.monotonic()-self.freshness.get('Mission',-math.inf)>2):
                    raise ValueError('Stop and wait for fresh status before tuning')
                if self.p['phase']=='mission' and self.state['state']!='WAITING':
                    raise ValueError('Configure navigation before mission Start')
            self.manual = (0.,0.,0.)
            self.manual_pub.publish(Twist())
        original = self.read_trial_nodes()
        if values is not None:
            expected = parameter_plan(values, self.p['phase'])
            attempted = []
            try:
                for name, entries in expected.items():
                    attempted.append(name)
                    self.write_trial_node(name,entries)
                actual = self.read_trial_nodes()
                if not self.trial_matches(actual,expected):
                    raise RuntimeError('Requested settings did not match live readback')
            except Exception as exc:
                restored = True
                for name in reversed(attempted):
                    try:
                        self.write_trial_node(name,{key:original[name][key] for key in expected[name]})
                    except Exception:
                        restored = False
                try:
                    restored = restored and self.trial_matches(self.read_trial_nodes(),original)
                except Exception:
                    restored = False
                self.trial_fault = None if restored else 'Trial update/rollback unverified; reapply settings before driving'
                raise RuntimeError(str(exc)+('; original values restored' if restored else '; driving locked')) from exc
            original = actual
            self.trial_fault = None
        gate = original['project_velocity_gate']
        current = {key:gate[key] for key in ('max_linear','max_angular')}
        current['goal_timeout'] = original['project_mapper' if self.p['phase']=='mapping' else 'project_mission']['goal_timeout']
        return {'success':True,'values':current,
                'limits':{key:dict(min=v[0],max=v[1],step=v[2],unit=v[3]) for key,v in LIMITS.items()},
                'protection':{key:gate[key] for key in ('stop_distance','rotation_clearance')},
                'synchronized':self.trial_matches(original,parameter_plan(current, self.p['phase'])),
                'fault':self.trial_fault,'temporary':True,
                'message':'Trial settings applied and verified; temporary until restart' if values is not None else ''}

    def read_gap_nodes(self):
        result = {}
        for name, entries in gap.parameter_plan(False,self.p['phase']).items():
            names = list(entries)
            if name in gap.COSTMAPS:
                names += ['footprint','footprint_padding','resolution',
                          'inflation_layer.enabled','inflation_layer.inflation_radius']
            elif name=='controller_server':
                names += ['progress_checker.plugin']
            response = self.parameter_request(name,'get_parameters',GetParameters.Request(names=names))
            if len(response.values)!=len(names):
                raise RuntimeError(f'{name}: incomplete gap readback')
            result[name] = {key:Parameter.from_parameter_msg(ParameterMessage(name=key,value=value)).value
                            for key,value in zip(names,response.values)}
            for key,value in result[name].items():
                expected = str if key=='footprint' or key.endswith('.plugin') else bool if key.endswith(('.enabled','.closed_loop')) else float
                if type(value) is not expected or (expected is float and not math.isfinite(value)):
                    raise RuntimeError(f'{name}: invalid {key} readback')
        return result

    def gap_settings(self, enabled=None, audacity=None):
        phase = self.p['phase']
        changing = enabled is not None or audacity is not None
        if audacity is not None:
            gap.validate_audacity(audacity)
        if changing:
            with self.lock:
                idle = self.state['state'] in ('WAITING','STOPPED','FAILED','MAPPED','MAPPED_PARTIAL','WAYPOINT_REACHED')
                if (not idle or time.monotonic()-self.freshness.get('Mission',-math.inf)>2
                        or (phase=='mission' and self.state['state']!='WAITING')):
                    raise ValueError('Stop mapping or configure clearance before starting stage 2')
            self.manual = (0.,0.,0.)
            self.manual_pub.publish(Twist())
        try:
            original = self.read_gap_nodes()
            current = gap.describe(original,phase)  # reject unsupported geometry before any write
        except (RuntimeError,ValueError) as exc:
            if not self.gap_reapply_required:
                self.gap_fault = 'Gap policy unavailable; driving locked: '+str(exc)
            raise RuntimeError(self.gap_fault) from exc
        if changing:
            if enabled is None:
                enabled = current['enabled']
                if enabled is None:
                    raise ValueError('Reapply both audacity and narrow-gap blocking to synchronize settings')
            expected = gap.parameter_plan(enabled,phase,audacity or current['audacity'] or gap.DEFAULT_AUDACITY)
            attempted = []
            try:
                for name,entries in expected.items():
                    attempted.append(name)
                    self.write_trial_node(name,entries)
                actual = self.read_gap_nodes()
                if not self.trial_matches(actual,expected):
                    raise RuntimeError('Gap settings differ from live readback')
                gap.describe(actual,phase)
            except Exception as exc:
                restored = True
                for name in reversed(attempted):
                    try:
                        self.write_trial_node(name,{key:original[name][key] for key in expected[name]})
                    except Exception:
                        restored = False
                try:
                    # Include geometry fields in verification, without treating bool/str as numbers.
                    restored = self.read_gap_nodes()==original and restored
                except Exception:
                    restored = False
                self.gap_fault = None if restored and gap.describe(original,phase)['synchronized'] else 'Gap update/rollback unverified; driving locked; reapply gap policy'
                self.gap_reapply_required = self.gap_fault is not None
                raise RuntimeError(str(exc)+('; original values restored' if restored else '; driving locked')) from exc
            original = actual
            self.gap_fault = None
            self.gap_reapply_required = False
        data = gap.describe(original,phase)
        if not data['synchronized']:
            self.gap_fault = 'Gap parameters differ across nodes; driving locked; reapply gap policy'
            self.gap_reapply_required = True
        elif not self.gap_reapply_required:
            self.gap_fault = None
        data.update(success=True,fault=self.gap_fault,
                    message='Navigation policy applied and verified; temporary until restart' if changing else '')
        return data

    def command(self,path,body,expected_generation=None):
        request_generation=self.control_generation if expected_generation is None else expected_generation
        if not isinstance(body,dict):
            raise ValueError('Request must be a JSON object')
        if path=='/api/attitude':
            return self.attitude_settings(body,request_generation)
        if path=='/api/scan_filter':
            return self.scan_filter_settings(body,request_generation)
        if path == '/api/calibration_target':
            return self.set_calibration_target(body)
        workflow=self.workflow_command(path,body)
        if workflow is not None:
            return workflow
        if self.p['phase']=='calibration' and path not in ('/api/stop','/api/initial_pose'):
            raise ValueError('Use the camera setup controls, or load a mapping/mission stage')
        if self.stage and self.stage.loading and path!='/api/stop':
            raise ValueError('Wait for the stage transition to finish')
        with self.lock:
            state = self.state['state']
            active = state in ('PLANNING','SEARCHING','OBSERVING','CANCELING','RETURNING','TURNING','VERIFYING','VERIFYING_HOME','MAPPING','SAVING','ACQUIRING','LOCALIZING','RECOVERING')
        if path in ('/api/start','/api/stop'):
            key = path.rsplit('/',1)[-1]
            start_generation=request_generation
            def stopped_during_start(reissue=False):
                if key!='start' or self.control_generation==start_generation:return None
                if reissue:
                    # One final Stop is ordered after the late Start response.
                    # Do not recurse into setup or retry a Start in the background.
                    stopper=self.command_clients['stop']
                    if stopper.wait_for_service(timeout_sec=.5):
                        stopped=stopper.call_async(Trigger.Request())
                        limit=time.monotonic()+2.
                        while not stopped.done() and time.monotonic()<limit:time.sleep(.02)
                        if not stopped.done() or not stopped.result().success:
                            self.workflow_fault='Stop after a delayed Start could not be confirmed; inspect robot status'
                    else:
                        self.workflow_fault='Stop service unavailable after a delayed Start; inspect robot status'
                return dict(success=False,message='Start canceled by Stop',canceled_by_stop=True)
            if key=='start' and not self.p['allow_motion']:
                raise ValueError('Motion disabled in this console')
            if key=='start' and (self.trial_fault or self.gap_fault or self.workflow_fault or self.scan_filter_fault):
                raise ValueError(self.trial_fault or self.gap_fault or self.workflow_fault or self.scan_filter_fault)
            if key=='start':
                record=self.record.snapshot()
                if record['interrupted'] or record['finished'] is not None:
                    raise ValueError('Begin a new trial before starting again')
                if self.stage and self.p['phase']=='mission' and not self.initial_pose_sent:
                    raise ValueError('Set the assigned initial pose before Start')
            if key=='stop':
                if self.stage:
                    self.stage.cancel.set()
                with self.lock:
                    self.control_generation+=1
                    if self.retry_after_stamp>-math.inf:self.retry_stop_generation=self.control_generation
                self.manual = (0.,0.,0.)
                self.manual_pub.publish(Twist())
            client = self.command_clients[key]
            if not client.wait_for_service(timeout_sec=1):
                if key=='stop' and self.stage:
                    return {'success':True,'message':'Stop recorded; no active stage service'}
                raise RuntimeError('Mission service is not available')
            canceled=stopped_during_start()
            if canceled:return canceled
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic()+5
            while not future.done() and time.monotonic()<deadline:
                time.sleep(0.02)
            if not future.done():
                canceled=stopped_during_start(reissue=True)
                if canceled:
                    self.workflow_fault=self.workflow_fault or 'Start response timed out after Stop; inspect robot status'
                    return canceled
                raise RuntimeError('Mission service timed out; inspect mission status')
            result = future.result()
            canceled=stopped_during_start(reissue=True)
            if canceled:return canceled
            if key=='start' and result.success:
                with self.lock:
                    changed=self.control_generation!=start_generation
                    if not changed:self.record.start(self.p['phase'])
                if changed:return stopped_during_start(reissue=True)
            return {'success':result.success,'message':result.message}
        if path=='/api/trial':
            return self.trial_settings(body)
        if path=='/api/navigation':
            profile = body.get('profile')
            if set(body) != {'profile'} or profile not in PROFILES:
                raise ValueError('Select original or fast navigation')
            consumer = 'project_mapper' if self.p['phase']=='mapping' else 'project_mission'
            timeout = self.read_trial_nodes()[consumer]['goal_timeout']
            result = self.trial_settings(dict(PROFILES[profile], goal_timeout=timeout))
            self.p['navigation_profile'] = profile
            if self.stage:
                self.stage.navigation_profile = profile
            return dict(result, profile=profile)
        if path=='/api/vision':
            self.require_idle()
            settings = validate_mount_settings(body)
            path = Path(self.p['session_directory']).expanduser()/'vision.yaml'
            atomic_json(path, {'project_perception': {'ros__parameters': settings}})
            self.p['vision_config'] = str(path.resolve())
            self.vision_settings = settings
            if self.stage:
                self.stage.vision_config = self.p['vision_config']
            return dict(success=True, message='Camera geometry saved. Load or reload the mission to apply it.')
        if path=='/api/map_waypoint':
            if self.p['phase']!='mapping' or active or not self.p['allow_motion']:
                raise ValueError('Mapping waypoints are available only while stage 1 is stopped')
            if self.trial_fault or self.gap_fault or self.workflow_fault or self.scan_filter_fault:
                raise ValueError(self.trial_fault or self.gap_fault or self.workflow_fault or self.scan_filter_fault)
            record=self.record.snapshot()
            if record['interrupted'] or record['finished'] is not None:
                raise ValueError('Begin a new trial before waypoint mapping')
            if set(body)!={'x','y','yaw'} or any(type(body[k]) not in (int,float) for k in body):
                raise ValueError('Provide waypoint x, y and heading as numbers')
            with self.lock:
                m=self.map_data
                if (time.monotonic()-self.freshness.get('Mission',-math.inf)>2
                        or time.monotonic()-self.freshness.get('Map',-math.inf)>2 or m is None):
                    raise ValueError('Wait for a fresh map and mapping status')
            target=validate_waypoint(np.array(m['cells']).reshape(m['height'],m['width']),
                m['resolution'],m['origin'],tuple(body[k] for k in ('x','y','yaw')))
            generation=self.control_generation
            self.manual=(0.,0.,0.);self.manual_pub.publish(Twist())
            values=dict(zip(('waypoint_x','waypoint_y','waypoint_yaw'),target))
            values['waypoint_request_time']=self.get_clock().now().nanoseconds*1e-9
            self.write_trial_node('project_mapper',values)
            response=self.parameter_request('project_mapper','get_parameters',GetParameters.Request(names=list(values)))
            actual=[Parameter.from_parameter_msg(ParameterMessage(name=k,value=v)).value for k,v in zip(values,response.values)]
            if len(actual)!=len(values) or any(type(a) is not float or not math.isclose(a,b,rel_tol=0,abs_tol=1e-8) for a,b in zip(actual,values.values())):
                raise RuntimeError('Waypoint parameters did not match readback; no goal was started')
            if generation!=self.control_generation:
                raise ValueError('Stop superseded the waypoint request')
            client=self.command_clients['start_waypoint']
            if not client.wait_for_service(timeout_sec=1):raise RuntimeError('Mapping waypoint service is unavailable')
            future=client.call_async(Trigger.Request());deadline=time.monotonic()+5
            while not future.done() and time.monotonic()<deadline:time.sleep(.01)
            if generation!=self.control_generation:
                self.command('/api/stop',{})
                raise ValueError('Stop superseded the waypoint request')
            if not future.done():
                self.command('/api/stop',{})
                raise RuntimeError('Waypoint start timed out; Stop requested')
            result=future.result()
            if result.success:self.record.start('mapping')
            return dict(success=result.success,message=result.message)
        if path=='/api/gap':
            if not body or set(body)-{'enabled','audacity'}:
                raise ValueError('Provide enabled: true/false and/or audacity: cautious/balanced/bold')
            if 'enabled' in body and type(body['enabled']) is not bool:
                raise ValueError('enabled must be true or false')
            if 'audacity' in body:
                gap.validate_audacity(body['audacity'])
            return self.gap_settings(body.get('enabled'),body.get('audacity'))
        if path=='/api/initial_pose':
            if active or self.p['phase'] not in ('mission','calibration'):
                raise ValueError('Initial pose is only available before the autonomous mission')
            with self.lock:
                fresh = time.monotonic()-self.freshness.get('Mission',-math.inf)<2.0
            if self.p['phase']=='calibration':
                self.require_camera_idle()
                self.require_zero_command()
            elif not fresh:
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
            if self.stage and self.state.get('home') is not None:
                raise ValueError('Initial pose is locked after the autonomous mission begins')
            self.initial_pub.publish(msg)
            with self.lock:
                self.target_estimate=None
            self.initial_pose_sent=True
            return {'success':True,'message':'Assigned initial pose sent to AMCL'}
        if path=='/api/teleop':
            if self.trial_fault or self.gap_fault or self.workflow_fault or self.scan_filter_fault:
                raise ValueError(self.trial_fault or self.gap_fault or self.workflow_fault or self.scan_filter_fault)
            if self.p['phase']!='mapping' or active or not self.p['allow_motion']:
                raise ValueError('Manual driving is restricted to the mapping stage')
            x,z = float(body.get('linear',0)),float(body.get('angular',0))
            if not math.isfinite(x) or not math.isfinite(z):
                raise ValueError('Invalid velocity')
            if x or z:
                with self.lock:
                    if time.monotonic()-self.freshness.get('Mission',-math.inf)>2:
                        raise ValueError('Mapping status is stale; manual motion is locked')
                self.record.start('mapping')
            self.manual = (max(-0.10,min(0.10,x)),max(-0.35,min(0.35,z)),time.monotonic())
            return {'success':True}
        if path=='/api/save_map':
            if self.p['phase']!='mapping' or active:
                raise ValueError('Map saving is only available during the mapping stage')
            self.manual = (0.,0.,0.)
            self.manual_pub.publish(Twist())
            saved = save_new_map(self.p['map_directory'])
            entry=self.catalog.register(saved,'snapshot')
            self.record.add_map(entry['id'])
            with self.lock:
                self.saved_map = saved
            return {'success':True,'message':'Map saved to '+self.saved_map,'path':self.saved_map,'map':entry}
        raise ValueError('Unknown command')

    def teleop_tick(self):
        if self.p['phase']=='mapping':
            x,z,stamp = self.manual
            cmd = Twist()
            if time.monotonic()-stamp<0.25:
                cmd.linear.x,cmd.angular.z = x,z
            self.manual_pub.publish(cmd)

    def destroy_node(self):
        self.model_labels.close()
        if self.context.ok():
            self.manual_pub.publish(Twist())
        if self.stage:
            self.stage.cancel.set()
            try:
                self.stage.stop()
            except RuntimeError as exc:
                self.get_logger().error(str(exc))
        self.server.shutdown()
        self.server.server_close()
        self.http_thread.join(timeout=2)
        super().destroy_node()


def main():
    from .ros_utils import run
    run(Console)
