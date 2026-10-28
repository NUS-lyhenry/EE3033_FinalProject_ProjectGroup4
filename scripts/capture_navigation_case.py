#!/usr/bin/env python3
"""Read-only simulation snapshot and timestamped ROS diagnostic recording.

Run inside the same isolated DDS domain as a test. No goals, parameters,
heartbeat, maps, TF or robot command messages are published.
"""
import argparse
import gzip
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlparse
from urllib.request import urlopen


def simulation_snapshot(url):
    parsed=urlparse(url)
    if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost'):
        raise ValueError('Only a localhost simulation console is allowed')
    base=url.rstrip('/')+'/api/'
    with urlopen(base+'state',timeout=5) as response:s=json.load(response)
    if s.get('simulated') is not True or s.get('preview_backend')!='main-ros':
        raise ValueError('Refusing a non-simulation endpoint')
    with urlopen(base+'map',timeout=5) as response:m=json.load(response)
    # Session tokens are deliberately excluded from persisted evidence.
    return dict(scenario=s['simulation_scenario'],map=m,
        state={k:s.get(k) for k in ('mission','pose','path','route','velocity_gate','navigation','simulation_motion')},
        captured_utc=__import__('datetime').datetime.now(__import__('datetime').timezone.utc).isoformat())


class NavigationRecorder:
    """Independent subscriber; truth is recorded exclusively for evaluation."""
    def __init__(self,node,directory):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Path as NavPath,Odometry,OccupancyGrid
        from nav2_msgs.msg import Costmap
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import qos_profile_sensor_data
        from ee3033_project.ros_utils import LATCHED
        self.node=node;self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        self.stream=gzip.open(self.directory/'ros_diagnostics.jsonl.gz','wt')
        self.lock=threading.Lock();self.counts={};self.latest={};self.subscriptions=[];self.closed=False
        specs=[('/plan',NavPath,10),('/project/native_plan',NavPath,10),('/local_plan',NavPath,10),('/received_global_plan',NavPath,10),
            ('/transformed_global_plan',NavPath,10),('/lookahead_arc',NavPath,10),
            ('/map',OccupancyGrid,LATCHED),('/local_costmap/costmap_raw',Costmap,10),
            ('/global_costmap/costmap_raw',Costmap,10),('/odom',Odometry,qos_profile_sensor_data),
            ('/scan_fixed',LaserScan,qos_profile_sensor_data),('/tf',TFMessage,10),('/tf_static',TFMessage,LATCHED),
            ('/project/nav_cmd_vel',Twist,10),('/cmd_vel',Twist,10),
            ('/project/velocity_gate_status',String,10),('/project/navigation_status',String,10),('/project/planning_status',String,10),
            ('/project/simulation_motion',String,10),('/project/mission_status',String,LATCHED)]
        try:
            from dwb_msgs.msg import LocalPlanEvaluation
            specs.append(('/evaluation',LocalPlanEvaluation,10))
        except ImportError:pass
        for topic,kind,qos in specs:
            self.subscriptions.append(node.create_subscription(kind,topic,
                lambda msg,t=topic:self.record(t,msg),qos))

    def record(self,topic,msg):
        from rosidl_runtime_py.convert import message_to_ordereddict
        d=message_to_ordereddict(msg)
        if topic in ('/project/velocity_gate_status','/project/navigation_status','/project/planning_status','/project/simulation_motion','/project/mission_status'):
            try:d=json.loads(d['data'])
            except (ValueError,KeyError):pass
        with self.lock:
            if self.closed:return
            self.latest[topic]=d;self.counts[topic]=self.counts.get(topic,0)+1
            self.stream.write(json.dumps(dict(monotonic=time.monotonic(),topic=topic,message=d),separators=(',',':'))+'\n')

    def close(self):
        with self.lock:
            self.closed=True
            self.stream.close()
        (self.directory/'topic_counts.json').write_text(json.dumps(self.counts,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:8796')
    parser.add_argument('--output',required=True)
    parser.add_argument('--duration',type=float,default=20.)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6') or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        parser.error('Use the explicitly isolated simulation DDS domain')
    import rclpy
    from rclpy.node import Node
    rclpy.init();node=Node('navigation_evidence_recorder');directory=Path(args.output);directory.mkdir(parents=True,exist_ok=True)
    (directory/'snapshot.json').write_text(json.dumps(simulation_snapshot(args.url),indent=2)+'\n')
    recorder=NavigationRecorder(node,directory)
    deadline=time.monotonic()+args.duration
    try:
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
    finally:recorder.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
