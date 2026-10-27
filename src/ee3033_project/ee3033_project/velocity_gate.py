"""Single final velocity writer with command and mission heartbeat deadlines."""
import math
import time
import json
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from .ros_utils import LATCHED, parameters, run
from .trial import guarded_parameters
from .gap import DEFAULT_AUDACITY, TURNING, guard_rotation


class VelocityGate(Node):
    def __init__(self, **kwargs):
        super().__init__('project_velocity_gate', **kwargs)
        self.p = parameters(self, {
            'allow_motion': False, 'mapping_mode': False, 'output_topic': '/cmd_vel',
            'command_timeout': 0.5, 'heartbeat_timeout': 0.7, 'scan_timeout': 0.7,
            'max_linear': 0.15, 'max_angular': 0.45,
            'stop_distance': 0.25, 'rotation_clearance': TURNING[DEFAULT_AUDACITY]['stop'],
            'navigation_profile': 'original',
        })
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
        self.create_timer(0.05, self.tick)

    def on_enable(self, msg):
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

    def tick(self):
        for name in ('max_linear','max_angular','rotation_clearance'):
            self.p[name] = self.get_parameter(name).value
        now = time.monotonic()
        out = Twist()
        reason = 'motion_disabled'
        nearest = bearing = None
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
                out.linear.x = max(-self.p['max_linear'], min(self.p['max_linear'], vx))
                out.angular.z = max(-self.p['max_angular'], min(self.p['max_angular'], wz))
                front, back, clearance = math.inf, math.inf, math.inf
                usable = 0
                for i, distance in enumerate(self.scan.ranges):
                    if math.isnan(distance) or distance < self.scan.range_min:
                        continue
                    usable += 1
                    if distance > self.scan.range_max:
                        continue
                    a = self.scan.angle_min+i*self.scan.angle_increment
                    if nearest is None or distance < nearest:
                        nearest, bearing = distance, a
                    clearance = min(clearance, distance)
                    if math.cos(a) > math.cos(0.50):
                        front = min(front, distance)
                    if math.cos(a) < -math.cos(0.50):
                        back = min(back, distance)
                if usable == 0:
                    out = Twist()
                    reason = 'scan_unusable'
                elif ((out.linear.x > 0 and front < self.p['stop_distance'])
                      or (out.linear.x < 0 and back < self.p['stop_distance'])):
                    out.linear.x = 0.0
                    reason = 'forward_obstacle' if vx > 0 else 'rear_obstacle'
                if clearance < self.p['rotation_clearance']:
                    out.angular.z = 0.0
                    if abs(wz) > 1e-5:
                        reason = 'rotation_obstacle'
            else:
                reason = 'invalid_command'
        self.pub.publish(out)
        if now-self.last_status >= .5:
            self.last_status = now
            def finite(v): return float(v) if math.isfinite(v) else None
            self.status_pub.publish(String(data=json.dumps(dict(
                stamp=self.get_clock().now().nanoseconds*1e-9, reason=reason,
                requested=[finite(self.command.linear.x), finite(self.command.angular.z)],
                output=[out.linear.x, out.angular.z],
                nearest=nearest, bearing=bearing,
                command_age=finite(now-self.command_time), scan_age=finite(now-self.scan_time),
                heartbeat_age=finite(now-self.enable_time)))))

    def destroy_node(self):
        if self.context.ok():
            self.pub.publish(Twist())
        super().destroy_node()


def main():
    run(VelocityGate)
