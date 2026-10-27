"""Two-stage operator workflow layered onto the existing console controls."""
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from .assessment import ACTIVE, MapCatalog, StageProcess, TrialRecord, mission_options, require_current_mapping_map, calibration_pose_source
from .core import stamp_seconds, quaternion_yaw
from .ros_utils import LATCHED
from .console_calibration import CameraSetup
from .vision_geometry import validate_mount_settings
import yaml


class Workflow(CameraSetup):
    def init_workflow(self):
        self.catalog = MapCatalog(self.p['map_directory'])
        self.record = TrialRecord(self.p['session_directory'])
        self.status_after=-math.inf
        self.selected_map = None
        self.mission_options = mission_options({})
        self.initial_pose_sent = False
        self.perception_data = None
        self.result_data = None
        self.command_sample = None
        self.command_stamp = -math.inf
        self.workflow_fault = None
        self.last_registered_map = None
        self.stage = StageProcess(Path(self.p['session_directory']).expanduser()/'logs',
            allow_motion=self.p['allow_motion'], model=self.p['model'],
            calibration_file=self.p['calibration_file'],
            hardware={name:self.p['stage_'+name] for name in ('base','lidar','camera')}) if self.p['manage_stages'] else None
        self.vision_settings = dict(camera_height=0., camera_pitch=0., camera_yaw=0.,
            camera_x=0., camera_y=0., horizontal_fov=0., camera_min_range=.35, camera_max_range=2.)
        saved = Path(self.p['vision_config']).expanduser() if self.p['vision_config'] else Path(self.p['session_directory']).expanduser()/'vision.yaml'
        if saved.is_file():
            data = yaml.safe_load(saved.read_text())['project_perception']['ros__parameters']
            self.vision_settings = validate_mount_settings({k: data[k] for k in self.vision_settings})
            self.p['vision_config'] = str(saved.resolve())
        if self.stage:
            self.stage.navigation_profile = self.p['navigation_profile']
            self.stage.vision_config = self.p['vision_config']
        self.init_camera_setup()
        self.create_subscription(Twist,'/cmd_vel',self.on_final_command,10)
        self.create_subscription(Odometry,'/odom',self.on_workflow_odom,qos_profile_sensor_data)
        self.create_subscription(String,'/project/mission_result',self.on_result,LATCHED)
        self.create_timer(.5,self.workflow_tick)

    def on_final_command(self,msg):
        with self.lock:
            self.command_sample=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
            self.command_stamp=time.monotonic()

    def on_workflow_odom(self,msg):
        age=self.get_clock().now().nanoseconds*1e-9-stamp_seconds(msg.header.stamp)
        if 0<=age<1.:
            self.mark('Odometry')

    def on_perception_status(self,msg):
        try:
            data=json.loads(msg.data)
            age=self.get_clock().now().nanoseconds*1e-9-float(data['stamp'])
            if data.get('ready') and 0<=age<3:
                with self.lock:
                    self.perception_data=data
                    self.freshness['YOLO']=time.monotonic()
        except (ValueError,TypeError,KeyError): pass

    def on_result(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('phase')!='mission' or self.p['phase']!='mission': return
            if self.stage and float(data.get('stamp',-math.inf))<self.status_after: return
            with self.lock: self.result_data=data
        except (ValueError,TypeError): pass

    def workflow_tick(self):
        self.camera_setup_tick()
        if self.stage and self.stage.snapshot()['fault']:
            self.workflow_fault=self.stage.snapshot()['fault']
            self.manual=(0.,0.,0.)

    def workflow_status(self,status):
        try:
            self.record.status(self.p['phase'],status)
            saved=status.get('saved_map')
            if saved and saved!=self.last_registered_map:
                entry=self.catalog.register(saved,'partial' if status.get('state')=='MAPPED_PARTIAL' else 'mapped')
                self.record.add_map(entry['id'])
                self.saved_map=saved
                self.last_registered_map=saved
        except (OSError,ValueError,TypeError) as exc:
            self.workflow_fault='Assessment record unavailable: '+str(exc)

    def workflow_snapshot(self):
        return dict(assessment=self.record.snapshot(),
                    camera_calibration=self.calibration_status, calibration_live=self.calibration_live,
                    calibration_port=self.p['calibration_port'],
                    stage=self.stage.snapshot() if self.stage else dict(managed=False,transitioning=False),
                    selected_map=self.selected_map,mission_options=self.mission_options,
                    initial_pose_sent=self.initial_pose_sent,perception=self.perception_data,
                    vision_settings=self.vision_settings, navigation_profile=self.p['navigation_profile'],
                    mission_result=self.result_data,workflow_fault=self.workflow_fault)

    def workflow_get(self,path):
        if path=='/api/maps': return dict(maps=self.catalog.list())
        if path=='/api/result':
            return dict(assessment=self.record.snapshot(),mission=self.result_data or self.state,
                        selected_map=self.selected_map,settings=self.mission_options,
                        phase=self.p['phase'],software_version=self.p['software_version'])
        return None

    def require_idle(self):
        with self.lock:
            if self.state['state'] in ACTIVE:
                raise ValueError('Stop the current stage and wait for cancellation before setup')
            if self.stage and self.stage.loading:
                raise ValueError('Stage transition is in progress')

    def require_zero_command(self):
        with self.lock:
            if (self.command_sample is None or time.monotonic()-self.command_stamp>.8
                    or any(not math.isfinite(v) or abs(v)>1e-5 for v in self.command_sample)):
                raise ValueError('Wait for fresh zero velocity output before changing stages')

    def workflow_command(self,path,body):
        reset_trial=path=='/api/new_trial'
        if reset_trial:
            self.require_idle()
            if self.stage is None:
                raise ValueError('Use assessment.launch.py to begin a fresh mapping trial')
            path,body='/api/stage',{'phase':'mapping'}
        if path=='/api/review_map':
            self.require_idle()
            if self.p['phase']!='mapping': raise ValueError('Review the map before stage 2')
            candidate=self.catalog.entry(body['map_id'])
            require_current_mapping_map(self.p['phase'], bool(self.stage and self.stage.process),
                                        self.saved_map, candidate['path'])
            entry=self.catalog.review(body['map_id'])
            return dict(success=True,map=entry,message='Map review recorded; map pixels unchanged')
        if path=='/api/stage':
            if self.stage is None: raise ValueError('Launch assessment.launch.py for UI stage transitions')
            self.require_idle()
            phase=body.get('phase')
            if phase not in ('mapping','mission','calibration'): raise ValueError('Select camera setup, mapping or mission')
            self.require_camera_idle()
            map_path=None;selected=None;options=mission_options(body)
            self.stage.calibration_initial_pose = None
            if phase=='calibration':
                if not body.get('map_id'):
                    raise ValueError('Select a reviewed saved map for camera calibration')
                candidate=self.catalog.entry(body['map_id']) if body.get('map_id') else None
                require_current_mapping_map(self.p['phase'], bool(self.stage.process),
                                            self.saved_map, candidate['path'] if candidate else None)
            if phase=='calibration' and body.get('map_id'):
                selected=self.catalog.entry(body['map_id'])
                if not selected['reviewed']:
                    raise ValueError('Review the saved map before automatic camera calibration')
                map_path=selected['path']
                # Capture the current SLAM pose before stopping mapping, or
                # fresh AMCL on the same loaded map. Never use trial/home poses.
                source=calibration_pose_source(self.p['phase'],self.saved_map,map_path,
                    self.selected_map['id'] if self.selected_map else None,selected['id'])
                fresh_key='Mission' if source=='mapping' else 'AMCL'
                if source and time.monotonic()-self.freshness.get(fresh_key,-math.inf)<3:
                    try:
                        tf=self.tf.lookup_transform('map','base_footprint',Time())
                        age=self.get_clock().now().nanoseconds*1e-9-stamp_seconds(tf.header.stamp)
                        if -.6<=age<1.:
                            t=tf.transform
                            self.stage.calibration_initial_pose=[t.translation.x,t.translation.y,quaternion_yaw(t.rotation)]
                    except Exception:
                        pass
                if source=='mapping' and self.stage.calibration_initial_pose is None:
                    raise ValueError('Wait for a fresh mapping pose before loading camera setup; mapping remains running')
            if phase=='mission':
                self.model_labels.require(options['target_class'])
                selected=self.catalog.entry(body.get('map_id'))
                if not selected['reviewed']: raise ValueError('Inspect and review the saved map before stage 2')
                record=self.record.snapshot()
                if selected['id'] not in record['maps'] or not record['includes_mapping']:
                    raise ValueError('Construct and save this trial map in stage 1 before stage 2')
                map_path=selected['path']
            record=self.record.snapshot()
            if phase!='calibration' and not reset_trial and (record['finished'] is not None or record['interrupted']):
                raise ValueError('Begin a new trial to construct a fresh map')
            if self.stage.process:
                with self.lock:
                    if self.p['phase']!='calibration' and time.monotonic()-self.freshness.get('Mission',-math.inf)>2:
                        raise ValueError('Stage status is stale; inspect before manual intervention')
                self.require_zero_command()
            else:
                if self.count_publishers('/cmd_vel') or self.count_publishers('/project/mission_status'):
                    raise ValueError('Another robot stack is running; stop it before loading a stage')
                existing=set(self.get_node_names())
                conflicts=[name for flag,name in [('base','balance_robot_node'),('lidar','lslidar_driver_node'),
                           ('camera','usb_cam')] if self.p['stage_'+flag] and name in existing]
                if 'project_scan_resampler' in existing:
                    conflicts.append('project_scan_resampler')
                if conflicts:
                    raise ValueError('Existing drivers would be duplicated: '+', '.join(conflicts)+'; reuse them through launch flags or stop the earlier stack')
            self.manual=(0.,0.,0.);self.manual_pub.publish(Twist())
            self.stage.cancel.clear();self.stage.loading=True
            try:
                self.stage.stop()
                deadline=time.monotonic()+4
                while self.count_publishers('/cmd_vel') or self.count_publishers('/project/mission_status'):
                    if time.monotonic()>deadline: raise RuntimeError('Previous ROS publishers still exist; successor was not started')
                    time.sleep(.05)
                if self.stage.cancel.is_set():
                    return dict(success=True,message='Stage transition stopped; no new stage launched')
                if reset_trial:
                    self.record.reset()
                with self.lock:
                    self.control_generation+=1
                    self.status_after=self.get_clock().now().nanoseconds*1e-9
                    self.p['phase']=phase
                    self.state=dict(state='WAITING',reason='Starting '+phase+' services; waiting for fresh status',home=None,target=None)
                    self.map_data=None;self.map_revision+=1;self.route=[];self.path=[]
                    self.saved_map=None
                    self.amcl_covariance=None;self.initial_pose_sent=bool(phase=='calibration' and self.stage.calibration_initial_pose is not None)
                    self.perception_data=None;self.result_data=None
                    self.target_estimate=None
                    self.gate_status=None
                    for key in ('Mission','Map','AMCL','YOLO','Lidar','Odometry'): self.freshness.pop(key,None)
                    self.selected_map=selected;self.mission_options=options
                    self.trial_fault=self.gap_fault=self.workflow_fault=None
                    self.gap_reapply_required=False
                self.refresh_camera_profile()
                self.stage.calibration_directory=self.p['calibration_directory']
                self.stage.calibration_port=self.p['calibration_port']
                self.stage.console_port=self.p['port']
                if not self.stage.launch(phase,map_path,options,self.p['map_directory']):
                    return dict(success=True,message='Stage transition stopped; no stage remains active')
                if phase=='mapping':
                    self.record.start('mapping')
            except Exception as exc:
                self.workflow_fault='Stage load failed; no automatic restart: '+str(exc)
                raise
            finally: self.stage.loading=False
            return dict(success=True,message=('Camera setup loaded; complete it once and save the validated profile' if phase=='calibration'
                        else 'Mapping stack loaded; Start begins exploration' if phase=='mapping'
                        else 'Saved map loaded; establish the assigned initial pose before Start'))
        return None
