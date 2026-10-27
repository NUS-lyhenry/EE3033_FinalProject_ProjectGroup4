import json
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String
from rclpy.node import Node
from .core import Observation, TargetTracker
from .ros_utils import LATCHED, parameters, run


class TargetFilter(Node):
    def __init__(self, **kwargs):
        super().__init__('project_target_filter', **kwargs)
        self.p = parameters(self, {
            'confirmation_frames': 4, 'confidence': 0.60, 'association_radius': 0.30,
            'max_frame_gap': 1.2, 'filter_window': 9, 'max_position_spread': 0.12,
            'max_observation_age': 2.0, 'map_frame': 'map',
        })
        self.tracker = TargetTracker(self.p['confirmation_frames'], self.p['confidence'],
                                     self.p['association_radius'], self.p['max_frame_gap'],
                                     self.p['filter_window'], self.p['max_position_spread'])
        self.active = False
        self.armed_at = 0.0
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                               '/project/confirmed_target', 10)
        self.details_pub = self.create_publisher(String, '/project/target_details', 10)
        self.create_subscription(Bool, '/project/search_active', self.on_active, LATCHED)
        self.create_subscription(String, '/project/target_observations', self.on_frame, 10)

    def on_active(self, msg):
        if msg.data and not self.active:
            self.tracker.reset()
            self.armed_at = self.get_clock().now().nanoseconds*1e-9
        if not msg.data:
            self.tracker.reset()
        self.active = msg.data

    def on_frame(self, msg):
        if not self.active:
            return
        try:
            data = json.loads(msg.data)
            stamp = float(data['stamp'])
            now = self.get_clock().now().nanoseconds*1e-9
            if stamp < self.armed_at or not 0 <= now-stamp <= self.p['max_observation_age']:
                self.tracker.reset()
                return
            candidates = [Observation(**{k: o[k] for k in
                          ('stamp','x','y','confidence','label','sigma')})
                          for o in data['observations']]
            if self.tracker.samples:
                previous = self.tracker.samples[-1]
                candidates = [c for c in candidates if c.label == previous.label]
                candidates.sort(key=lambda c: math.hypot(c.x-previous.x, c.y-previous.y))
            else:
                candidates.sort(key=lambda c: c.confidence, reverse=True)
            if not candidates and data.get('deferred') is True:
                self.tracker.defer(stamp)
                return
            estimate = self.tracker.update(candidates[0] if candidates else None)
            if estimate is None:
                return
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = self.p['map_frame']
            pose.header.stamp.sec = int(stamp)
            pose.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
            pose.pose.pose.position.x = estimate['x']
            pose.pose.pose.position.y = estimate['y']
            pose.pose.pose.orientation.w = 1.0
            pose.pose.covariance[0] = estimate['variance']
            pose.pose.covariance[7] = estimate['variance']
            self.pose_pub.publish(pose)
            self.details_pub.publish(String(data=json.dumps(estimate)))
        except (ValueError, TypeError, KeyError, OverflowError):
            self.tracker.reset()


def main():
    run(TargetFilter)
