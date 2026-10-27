"""One authoritative map posterior for both the UI and mission acceptance."""
import json
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from rclpy.node import Node
from .core import quaternion_yaw
from .ros_utils import LATCHED, parameters, run
from .target_estimate import MapMMSE


class TargetFilter(Node):
    def __init__(self, **kwargs):
        super().__init__('project_target_filter', **kwargs)
        self.p=parameters(self,dict(confirmation_frames=4,confidence=.60,max_frame_gap=1.2,
            max_observation_age=2.,map_frame='map',target_class='',posterior_window=8.,
            posterior_samples=30,common_error_fraction=.8,unknown_prior_weight=.25))
        self.tracker=MapMMSE(frames=self.p['confirmation_frames'],seconds=self.p['posterior_window'],
            max_samples=self.p['posterior_samples'],max_gap=self.p['max_frame_gap'],
            max_age=self.p['max_observation_age'],confidence=self.p['confidence'],
            common_fraction=self.p['common_error_fraction'],unknown_weight=self.p['unknown_prior_weight'])
        self.active=False
        self.armed_at=0.
        self.seed=None
        self.mission_stamp=None
        self.last_frame_stamp=0.
        self.pose_pub=self.create_publisher(PoseWithCovarianceStamped,'/project/confirmed_target',10)
        self.details_pub=self.create_publisher(String,'/project/target_details',10)
        self.estimate_pub=self.create_publisher(String,'/project/target_estimate',LATCHED)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,LATCHED)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,10)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.on_initial_pose,10)
        self.create_subscription(Bool,'/project/search_active',self.on_active,LATCHED)
        self.create_subscription(String,'/project/target_observations',self.on_frame,10)
        self.create_subscription(String,'/project/mission_status',self.on_status,LATCHED)
        self.create_subscription(String,'/project/mission_result',self.on_result,LATCHED)
        self.create_timer(.5,self.publish)

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def on_map(self,msg):
        if msg.header.frame_id!=self.p['map_frame']:return
        try:
            grid=np.array(msg.data).reshape(msg.info.height,msg.info.width)
            pose=msg.info.origin
            self.tracker.set_map(grid,msg.info.resolution,[pose.position.x,pose.position.y,quaternion_yaw(pose.orientation)])
            if self.seed:self.tracker.set_seed(*self.seed)
            self.publish()
        except (ValueError,TypeError) as exc:self.get_logger().warning('MMSE map unavailable: '+str(exc))

    def on_pose(self,msg):
        if msg.header.frame_id!=self.p['map_frame']:return
        p=msg.pose.pose.position
        if math.isfinite(p.x) and math.isfinite(p.y):
            self.seed=(p.x,p.y)
            self.tracker.set_seed(*self.seed)

    def on_initial_pose(self,msg):
        self.tracker.reset()
        self.on_pose(msg)
        self.publish()

    def on_active(self,msg):
        self.active=msg.data
        self.publish()

    def on_frame(self,msg):
        try:
            data=json.loads(msg.data);stamp=float(data['stamp']);now=self.now()
            if not 0<=now-stamp<=self.p['max_observation_age'] or stamp<self.armed_at or stamp<=self.last_frame_stamp:return
            self.last_frame_stamp=stamp
            candidates=[o for o in data['observations'] if o.get('stamp')==stamp and (not self.p['target_class'] or o.get('label')==self.p['target_class'])]
            candidates.sort(key=lambda o:o.get('confidence',0),reverse=True)
            updated=bool(candidates and self.tracker.update(candidates[0],now))
            if not updated:self.tracker.miss(stamp,data.get('deferred') is True)
            self.publish()
            estimate=self.tracker.snapshot(now)
            if self.active and updated and estimate and estimate['stable']:
                pose=PoseWithCovarianceStamped()
                pose.header.frame_id=self.p['map_frame']
                pose.header.stamp.sec=int(estimate['stamp'])
                pose.header.stamp.nanosec=int((estimate['stamp']-int(estimate['stamp']))*1e9)
                pose.pose.pose.position.x,pose.pose.pose.position.y=estimate['x'],estimate['y']
                pose.pose.pose.orientation.w=1.
                c=estimate['covariance']
                pose.pose.covariance[0],pose.pose.covariance[1]=c[0]
                pose.pose.covariance[6],pose.pose.covariance[7]=c[1]
                self.pose_pub.publish(pose)
                self.details_pub.publish(String(data=json.dumps(estimate)))
        except (ValueError,TypeError,KeyError,OverflowError):pass

    def on_status(self,msg):
        try:
            data=json.loads(msg.data);stamp=data.get('started_stamp')
            if stamp and stamp!=self.mission_stamp:
                self.mission_stamp=stamp
                self.armed_at=float(stamp)
                self.last_frame_stamp=0.
                self.tracker.reset()
                self.publish()
        except (ValueError,TypeError):pass

    def on_result(self,msg):
        try:
            data=json.loads(msg.data);target=data.get('target')
            if target and target.get('estimator')=='map-constrained MMSE':
                frozen=dict(target,label=target['class'],stamp=target['observation_stamp'],
                            method=target['estimator'],samples=target.get('confirmation_frames',4),
                            frames=target.get('confirmation_frames',4),prior_conflict=False)
                self.tracker.freeze(frozen)
                self.publish()
        except (ValueError,TypeError,KeyError):pass

    def publish(self):
        result=self.tracker.snapshot(self.now())
        if result:
            result['frame_id']=self.p['map_frame']
            result['mission_stamp']=self.mission_stamp
        self.estimate_pub.publish(String(data=json.dumps(result or dict(available=False,map_signature=self.tracker.signature))))


def main():run(TargetFilter)
