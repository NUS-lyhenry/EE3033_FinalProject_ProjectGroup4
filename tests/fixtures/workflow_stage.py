"""Isolated ROS stage fixture: real project nodes, synthetic sensors/actions.

Never use domain 6. This fixture cannot start chassis, lidar, camera or motors.
"""
import os
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.executors import MultiThreadedExecutor,ExternalShutdownException
from integration_ros import FakeRobot,overrides
from ee3033_project.mapper import Mapper
from ee3033_project.mission import Mission
from ee3033_project.target_filter import TargetFilter
from ee3033_project.velocity_gate import VelocityGate

if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
    raise SystemExit('Fixture requires an isolated ROS domain')
phase,root=sys.argv[1:3]
rclpy.init()
robot=FakeRobot('found' if phase=='mission' else 'absent')
consumer=(Mapper(parameter_overrides=overrides(allow_motion=True,map_directory=root,
    settle_seconds=.1,frontier_settle_seconds=.2)) if phase=='mapping' else
    Mission(parameter_overrides=overrides(allow_motion=True,target_class='bottle',
    viewpoint_spacing=1.,max_goals=4,max_candidates=12,observe_seconds=.2,
    verify_seconds=.25,verify_timeout=2.,results_directory=str(Path(root)/'results'))))
gate=VelocityGate(parameter_overrides=overrides(allow_motion=False,mapping_mode=phase=='mapping'))
nodes=[robot,consumer,gate]
if phase=='mission':nodes.append(TargetFilter())
executor=MultiThreadedExecutor(num_threads=6)
for node in nodes:executor.add_node(node)
try:executor.spin()
except (KeyboardInterrupt,ExternalShutdownException):pass
finally:
    executor.shutdown(timeout_sec=3)
    for node in reversed(nodes):node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
