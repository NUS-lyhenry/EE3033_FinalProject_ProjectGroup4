"""Single final velocity writer with command and mission heartbeat deadlines."""
import math
import time
import json
import numpy as np
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from .ros_utils import LATCHED, parameters, run
from .trial import guarded_parameters
from .gap import DEFAULT_AUDACITY, TURNING, guard_rotation
from .core import quaternion_yaw, stamp_seconds
from .reverse_safety import reverse_clearance
from .robot_geometry import DEFAULT_STOP_DISTANCE, geometry_metadata
from .motion_safety import (FOOTPRINT_RADIUS, limit_twist, swept_arc,
                            swept_map_clearance, swept_scan_clearance)


class VelocityGate(Node):
    def __init__(self, **kwargs):
        super().__init__('project_velocity_gate', **kwargs)
        self.p = parameters(self, {
            'allow_motion': False, 'mapping_mode': False, 'output_topic': '/cmd_vel',
            'command_timeout': 0.5, 'heartbeat_timeout': 0.7, 'scan_timeout': 0.7,
            'max_linear': 0.12, 'max_angular': 0.40,
            'stop_distance': DEFAULT_STOP_DISTANCE, 'rotation_clearance': TURNING[DEFAULT_AUDACITY]['stop'],
            'navigation_profile': 'original',
            'reverse_speed': .04, 'reverse_distance': .20,
            'reverse_reserve': .04, 'reverse_radius': FOOTPRINT_RADIUS,
            'odom_frame': 'odom_combined', 'base_frame': 'base_footprint',
            'reverse_policy': 'recovery',
            'enforce_swept_map': True, 'swept_horizon': 1.0,
            'braking_deceleration': 0.30, 'footprint_radius': FOOTPRINT_RADIUS,
        })
        if (not math.isfinite(self.p['swept_horizon']) or self.p['swept_horizon']<=0
                or not math.isfinite(self.p['braking_deceleration']) or self.p['braking_deceleration']<=0
                or not math.isfinite(self.p['footprint_radius']) or self.p['footprint_radius']<FOOTPRINT_RADIUS):
            raise ValueError('Swept motion requires a positive horizon, braking limit and full footprint')
        if self.p['reverse_policy'] not in ('recovery','calibration'):
            raise ValueError('reverse_policy must be recovery or calibration')
        if self.p['navigation_profile'] not in ('original', 'fast'):
            raise ValueError('navigation_profile must be original or fast')
        if self.p['navigation_profile'] == 'fast':
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('max_linear', value=.16), Parameter('max_angular', value=.60)])
        self.enabled = False
        self.automatic = False
        guarded_parameters(self, ('max_linear','max_angular'),
                           lambda: not self.enabled and not self.automatic)
        guard_rotation(self, lambda: not self.enabled and not self.automatic)
        self.automatic_time = -math.inf
        self.enable_time = self.command_time = self.scan_time = -math.inf
        self.command = Twist()
        self.commands = {'manual': (Twist(), -math.inf), 'nav': (Twist(), -math.inf)}
        self.scan = None
        self.map = None
        self.localization = None
        self.output_arc = []
        self.output_arc_frame = None
        self.speed_scale = 1.
        self.safety_reason = None
        self.reverse_start = None
        self.reverse_idle = None
        self.reverse_reason = None
        self.reverse_envelope = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(Twist, self.p['output_topic'], 10)
        self.status_pub = self.create_publisher(String, '/project/velocity_gate_status', 10)
        self.last_status = 0.
        self.create_subscription(Bool, '/project/drive_enabled', self.on_enable, LATCHED)
        self.create_subscription(Bool, '/project/mapping_active', self.on_automatic, LATCHED)
        self.create_subscription(Twist, '/project/nav_cmd_vel',
                                 lambda msg:self.on_command(msg,'nav'), 10)
        if self.p['mapping_mode']:
            self.create_subscription(Twist, '/project/manual_cmd_vel',
                                     lambda msg:self.on_command(msg,'manual'), 10)
        self.create_subscription(LaserScan, '/scan_fixed', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        self.create_subscription(PoseWithCovarianceStamped,
            '/pose' if self.p['mapping_mode'] else '/amcl_pose', self.on_localization, 10)
        self.create_timer(0.05, self.tick)

    def on_map(self, msg):
        if msg.header.frame_id == 'map':
            self.map = msg

    def on_localization(self, msg):
        if msg.header.frame_id == 'map':
            self.localization = msg

    def approve_reverse(self, now):
        """Rear laser coverage is optional; known map and fresh pose are required."""
        self.reverse_envelope = None
        if self.map is None or self.localization is None:
            return False, 'reverse_map_or_localization_missing'
        clock = self.get_clock().now().nanoseconds*1e-9
        if not 0 <= clock-stamp_seconds(self.localization.header.stamp) <= 5.:
            return False, 'reverse_localization_stale'
        try:
            tr = self.tf.lookup_transform('map', self.p['base_frame'], Time())
            od = self.tf.lookup_transform(self.p['odom_frame'], self.p['base_frame'], Time())
            if any(not -.5 <= clock-stamp_seconds(t.header.stamp) <= .5 for t in (tr,od)):
                return False, 'reverse_transform_stale'
            p,q = tr.transform.translation,tr.transform.rotation
            op = od.transform.translation
            if self.reverse_start is None:
                self.reverse_start = (now, op.x, op.y)
            duration = now-self.reverse_start[0]
            traveled = math.hypot(op.x-self.reverse_start[1],op.y-self.reverse_start[2])
            if duration > 8. or traveled >= min(.20,self.p['reverse_distance']):
                return False, 'reverse_distance_or_time_limit'
            m=self.map;o=m.info.origin
            covariance=np.asarray(self.localization.pose.covariance).reshape(6,6)[np.ix_([0,1,5],[0,1,5])]
            allowed,reason,envelope = reverse_clearance(
                np.asarray(m.data).reshape(m.info.height,m.info.width),m.info.resolution,
                (o.position.x,o.position.y,quaternion_yaw(o.orientation)),
                (p.x,p.y,quaternion_yaw(q)),covariance,
                distance=min(.20,self.p['reverse_distance'])-traveled+.04,
                radius=max(FOOTPRINT_RADIUS,self.p['reverse_radius']),reserve=max(.04,self.p['reverse_reserve']))
            self.reverse_envelope = envelope
            return allowed, reason
        except (TransformException, ValueError, TypeError):
            return False, 'reverse_pose_unavailable'

    def on_enable(self, msg):
        if msg.data != self.enabled:
            # A new movement generation must receive a new command. A late
            # command from the canceled goal cannot survive Stop/re-enable.
            self.commands['nav'] = (Twist(), -math.inf)
        self.enabled, self.enable_time = msg.data, time.monotonic()

    def on_automatic(self,msg):
        if msg.data != self.automatic:
            self.commands = {'manual': (Twist(), -math.inf), 'nav': (Twist(), -math.inf)}
        self.automatic,self.automatic_time = msg.data,time.monotonic()

    def on_command(self, msg, source='nav'):
        self.commands[source] = msg,time.monotonic()

    def on_scan(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        age = self.get_clock().now().nanoseconds*1e-9 - stamp
        if 0 <= age < self.p['scan_timeout']:
            self.scan, self.scan_time = msg, time.monotonic()

    def laser_hits(self):
        """Actual scan endpoints in base coordinates, using capture-time mount TF."""
        scan = self.scan
        frame = scan.header.frame_id
        if not frame:
            raise ValueError('scan_frame_missing')
        tx = ty = yaw = 0.
        if frame != self.p['base_frame']:
            tr = self.tf.lookup_transform(self.p['base_frame'], frame, Time.from_msg(scan.header.stamp))
            tx, ty = tr.transform.translation.x, tr.transform.translation.y
            yaw = quaternion_yaw(tr.transform.rotation)
        points = []
        usable = 0
        for i, distance in enumerate(scan.ranges):
            if (math.isnan(distance) or distance < scan.range_min
                    or (math.isfinite(distance) and distance > scan.range_max)):
                continue
            usable += 1
            if not math.isfinite(distance) or distance >= scan.range_max:
                continue
            a = scan.angle_min + i*scan.angle_increment + yaw
            points.append((tx+distance*math.cos(a), ty+distance*math.sin(a)))
        return np.asarray(points,dtype=float).reshape(-1,2),usable

    def check_sweep(self, linear, angular, hits, require_map):
        """Check final, limited motion; never mutate its curvature for avoidance."""
        # Include command expiry and braking in the swept horizon. This is an
        # output check, not a second path planner or an authorization to move.
        horizon=max(self.p['swept_horizon'], self.p['command_timeout']+
                    abs(linear)/self.p['braking_deceleration'])
        radius=max(FOOTPRINT_RADIUS,self.p['footprint_radius'])
        laser=swept_scan_clearance(hits,linear,angular,horizon,radius)
        self.output_arc=swept_arc(linear,angular,horizon)[:,:2].tolist()
        self.output_arc_frame=self.p['base_frame']
        if not laser.allowed:
            return False,laser.reason
        if self.map is None:
            return (False,'swept_map_missing') if require_map else (True,laser.reason)
        try:
            tr=self.tf.lookup_transform('map',self.p['base_frame'],Time())
            clock=self.get_clock().now().nanoseconds*1e-9
            if not -.5<=clock-stamp_seconds(tr.header.stamp)<=.5:
                return False,'swept_transform_stale'
            p,q=tr.transform.translation,tr.transform.rotation
            pose=(p.x,p.y,quaternion_yaw(q))
            self.output_arc=swept_arc(linear,angular,horizon,pose=pose)[:,:2].tolist()
            self.output_arc_frame='map'
            m=self.map;o=m.info.origin
            check=swept_map_clearance(np.asarray(m.data).reshape(m.info.height,m.info.width),
                m.info.resolution,(o.position.x,o.position.y,quaternion_yaw(o.orientation)),
                pose,linear,angular,horizon,radius)
            return check.allowed,check.reason
        except (TransformException,ValueError,TypeError,ZeroDivisionError):
            return False,'swept_pose_unavailable'

    def tick(self):
        for name in ('max_linear','max_angular','rotation_clearance'):
            self.p[name] = self.get_parameter(name).value
        now = time.monotonic()
        out = Twist()
        reason = 'motion_disabled'
        nearest = bearing = None
        self.output_arc=[];self.output_arc_frame=None;self.speed_scale=1.;self.safety_reason=None
        manual = self.p['mapping_mode'] and not self.automatic
        self.command,self.command_time = self.commands['manual' if manual else 'nav']
        active = manual or (self.enabled and now-self.enable_time < self.p['heartbeat_timeout'])
        if self.p['mapping_mode'] and self.automatic:
            active = active and now-self.automatic_time < self.p['heartbeat_timeout']
        if self.p['allow_motion']:
            reason = ('heartbeat_missing' if not active else 'command_stale'
                      if now-self.command_time >= self.p['command_timeout'] else 'scan_stale')
        if (self.p['allow_motion'] and active
                and now-self.command_time < self.p['command_timeout']
                and now-self.scan_time < self.p['scan_timeout'] and self.scan is not None):
            vx, wz = self.command.linear.x, self.command.angular.z
            if math.isfinite(vx) and math.isfinite(wz):
                reason = 'passing'
                out.linear.x,out.angular.z,self.speed_scale=limit_twist(
                    vx,wz,self.p['max_linear'],self.p['max_angular'])
                if vx < -1e-5 and self.p['reverse_policy']=='recovery':
                    self.reverse_idle = None
                    approved,self.reverse_reason = self.approve_reverse(now)
                    if approved and abs(wz) <= .01:
                        out.linear.x = max(-min(.04,self.p['reverse_speed']),out.linear.x)
                        # Bounded recovery is straight by contract, not a
                        # clipped curve which the controller expected to turn.
                        out.angular.z = 0.
                        reason = 'bounded_map_checked_reverse'
                    else:
                        out = Twist()
                        reason = self.reverse_reason if not approved else 'reverse_must_be_straight'
                else:
                    self.reverse_idle = self.reverse_idle or now
                    if now-self.reverse_idle >= .5:
                        self.reverse_start = None
                if abs(out.linear.x)>1e-5 or abs(out.angular.z)>1e-5:
                    try:
                        hits,usable=self.laser_hits()
                        if not usable:
                            out=Twist();reason='scan_unusable'
                        else:
                            if len(hits):
                                distances=np.linalg.norm(hits,axis=1)
                                idx=int(np.argmin(distances))
                                nearest=float(distances[idx]);bearing=math.atan2(hits[idx,1],hits[idx,0])
                            # Preserve the configured front/rear emergency
                            # stop and rotation floor in base coordinates.
                            angles=np.arctan2(hits[:,1],hits[:,0]) if len(hits) else np.array([])
                            ranges=np.linalg.norm(hits,axis=1) if len(hits) else np.array([])
                            front=ranges[np.cos(angles)>math.cos(.50)]
                            back=ranges[np.cos(angles)<-math.cos(.50)]
                            blocked=(out.linear.x>0 and len(front) and min(front)<self.p['stop_distance']) or (out.linear.x<0 and len(back) and min(back)<self.p['stop_distance'])
                            if blocked:
                                reason='forward_obstacle' if out.linear.x>0 else 'rear_obstacle'
                                out=Twist()
                            elif (nearest is not None and nearest<self.p['rotation_clearance'] and abs(out.angular.z)>1e-5):
                                reason='rotation_obstacle';out=Twist()
                            else:
                                allowed,self.safety_reason=self.check_sweep(out.linear.x,out.angular.z,hits,
                                    self.p['enforce_swept_map'] and not manual)
                                if not allowed:
                                    reason=self.safety_reason;out=Twist()
                    except (TransformException,ValueError,TypeError):
                        reason='scan_transform_unavailable';out=Twist()
            else:
                reason = 'invalid_command'
        self.pub.publish(out)
        if now-self.last_status >= .5:
            self.last_status = now
            def finite(v): return float(v) if math.isfinite(v) else None
            self.status_pub.publish(String(data=json.dumps(dict(
                stamp=self.get_clock().now().nanoseconds*1e-9, reason=reason,
                requested=[finite(self.command.linear.x), finite(self.command.angular.z)],
                output=[out.linear.x, out.angular.z], speed_scale=self.speed_scale,
                safety_reason=self.safety_reason, output_arc=self.output_arc,
                output_arc_frame=self.output_arc_frame, footprint_radius=FOOTPRINT_RADIUS,
                robot_geometry=geometry_metadata(),
                nearest=nearest, bearing=bearing,
                command_age=finite(now-self.command_time), scan_age=finite(now-self.scan_time),
                heartbeat_age=finite(now-self.enable_time)))))

    def destroy_node(self):
        if self.context.ok():
            self.pub.publish(Twist())
        super().destroy_node()


def main():
    run(VelocityGate)
