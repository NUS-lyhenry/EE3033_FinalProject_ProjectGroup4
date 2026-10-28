"""Own fixed-bin adapter; leaves the lab resampler unchanged."""
import math
import struct
import time
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from .ros_utils import LATCHED, parameters, run
from .scan_core import resample


class ScanResampler(Node):
    def __init__(self, **kwargs):
        super().__init__('project_scan_resampler', **kwargs)
        self.p = parameters(self, {'input_topic': '/scan', 'output_topic': '/scan_fixed',
                                   'bins': 450,'self_mask_radius':0.08})
        # This explicitly configured sensor-distance exclusion is independent
        # of the physical chassis footprint and the sensor's true range_min.
        if (type(self.p['self_mask_radius']) is not float
                or not math.isfinite(self.p['self_mask_radius'])
                or not 0. <= self.p['self_mask_radius'] <= .20):
            raise ValueError('self_mask_radius must be a finite float in 0.0–0.20 m')
        if self.p['bins'] < 10:
            raise ValueError('bins must be at least 10')
        self.pub = self.create_publisher(LaserScan, self.p['output_topic'],
                                         qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.p['input_topic'], self.on_scan,
                                 qos_profile_sensor_data)
        self.motion_heartbeats={'drive':(False,-math.inf),'automatic':(False,-math.inf)}
        self.create_subscription(Bool,'/project/drive_enabled',
            lambda msg:self.on_motion(msg,'drive'),LATCHED)
        # This is the mapper's existing automatic-mode heartbeat. Mission and
        # calibration use drive_enabled; no parallel motion switch is invented.
        self.create_subscription(Bool,'/project/mapping_active',
            lambda msg:self.on_motion(msg,'automatic'),LATCHED)
        self.add_on_set_parameters_callback(self.validate_mask)

    def on_motion(self, msg, source):
        self.motion_heartbeats[source]=(bool(msg.data),time.monotonic())

    def validate_mask(self, changes):
        for value in changes:
            if value.name!='self_mask_radius':
                continue
            if (value.type_!=Parameter.Type.DOUBLE or type(value.value) is not float
                    or not math.isfinite(value.value) or not 0.<=value.value<=.20):
                return SetParametersResult(successful=False,
                    reason='self_mask_radius must be a finite float in 0.0–0.20 m')
            now=time.monotonic()
            if any(active and 0.<=now-stamp<.7
                    for active,stamp in self.motion_heartbeats.values()):
                return SetParametersResult(successful=False,
                    reason='Stop before changing the near-return exclusion distance')
        # Do not cache proposed values: later atomic validators may reject the
        # update. on_scan reads the actually committed parameter each frame.
        return SetParametersResult(successful=True)

    def on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.range_min, out.range_max = msg.range_min, msg.range_max
        out.scan_time = msg.scan_time
        n = self.p['bins']
        out.angle_min = -math.pi
        out.angle_increment = 2*math.pi/n
        out.angle_max = out.angle_min+(n-1)*out.angle_increment
        # Bins are reordered, so do not invent a per-bin acquisition time.
        out.time_increment = 0.0
        radius=self.get_parameter('self_mask_radius').value
        # LaserScan ranges are float32. Quantize the configured boundary to the
        # same representation so an exact 8 cm wire value is retained.
        radius=struct.unpack('f',struct.pack('f',radius))[0]
        out.ranges = resample(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,
                              msg.range_max,n,radius)
        self.pub.publish(out)


def main():
    run(ScanResampler)
