"""Own fixed-bin adapter; leaves the lab resampler unchanged."""
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from .ros_utils import parameters, run
from .scan_core import resample


class ScanResampler(Node):
    def __init__(self):
        super().__init__('project_scan_resampler')
        self.p = parameters(self, {'input_topic': '/scan', 'output_topic': '/scan_fixed',
                                   'bins': 450,'self_mask_radius':0.14})
        if self.p['bins'] < 10:
            raise ValueError('bins must be at least 10')
        self.pub = self.create_publisher(LaserScan, self.p['output_topic'],
                                         qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p['input_topic'], self.on_scan,
                                 qos_profile_sensor_data)

    def on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.range_min, out.range_max = max(msg.range_min,self.p['self_mask_radius']), msg.range_max
        out.scan_time = msg.scan_time
        n = self.p['bins']
        out.angle_min = -math.pi
        out.angle_increment = 2*math.pi/n
        out.angle_max = out.angle_min+(n-1)*out.angle_increment
        # Bins are reordered, so do not invent a per-bin acquisition time.
        out.time_increment = 0.0
        out.ranges = resample(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,
                              msg.range_max,n,self.p['self_mask_radius'])
        self.pub.publish(out)


def main():
    run(ScanResampler)
