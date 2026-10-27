"""Real Humble navigator/planner/controller recovery with a synthetic chassis.

Exercises the installed default tree through NavigateToPose, rather than
calling its child actions directly. Uses an isolated domain and a test-only
gate output, without a /cmd_vel publisher or physical drivers.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, BackUp
from nav2_msgs.srv import GetCostmap, ManageLifecycleNodes
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import yaml

from ee3033_project.ros_utils import set_yaw
from ee3033_project.velocity_gate import VelocityGate
from integration_turning_nav2 import Chassis, wait


def until(predicate, seconds=10):
    deadline=time.monotonic()+seconds
    while not predicate():
        assert time.monotonic()<deadline, 'Condition timed out'
        time.sleep(.02)


class RecoveryChassis(Chassis):
    def __init__(self):
        super().__init__()
        self.localization_valid=True
        self.rear_blind=True
        self.localization=self.create_publisher(PoseWithCovarianceStamped,'/amcl_pose',10)
        self.behaviors={name:set() for name in ('spin','backup','wait')}
        for name in self.behaviors:
            self.create_subscription(GoalStatusArray, '/'+name+'/_action/status',
                lambda msg,key=name: self.behaviors[key].update(
                    bytes(s.goal_info.goal_id.uuid) for s in msg.status_list), 10)

    def tick(self):
        super().tick()
        pose=PoseWithCovarianceStamped();pose.header.frame_id='map'
        pose.header.stamp=self.get_clock().now().to_msg()
        pose.pose.pose.position.x,pose.pose.pose.position.y=self.xyz[:2]
        set_yaw(pose.pose.pose.orientation,self.xyz[2])
        for i in (0,7,35):pose.pose.covariance[i]=.0004 if self.localization_valid else .25
        self.localization.publish(pose)

    def map_wall(self, blocked, unknown_rear=False):
        msg=OccupancyGrid();msg.header.frame_id='map'
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.info.width=msg.info.height=100;msg.info.resolution=.05
        msg.info.origin.position.x=msg.info.origin.position.y=-2.5
        msg.info.origin.orientation.w=1.
        grid=np.zeros((100,100),np.int8);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
        if blocked:grid[:,57]=100 # Sealed divider at x=0.35 m.
        if unknown_rear:grid[45:56,40:47]=-1
        msg.data=grid.ravel().tolist();self.maps.publish(msg)


def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
        raise SystemExit('Use an isolated ROS_DOMAIN_ID, e.g. 101')
    root=Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix='group4_recovery_') as tmp:
        config=yaml.safe_load((root/'src/ee3033_project/config/navigation.yaml').read_text())
        # Only shorten the synthetic frozen-chassis case. Production keeps 18 s.
        config['controller_server']['ros__parameters']['progress_checker']['movement_time_allowance']=1.5
        for name in ('local_costmap','global_costmap'):
            p=config[name][name]['ros__parameters']
            p['plugins']=['static_layer','inflation_layer']
            p['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True}
        path=Path(tmp)/'nav.yaml';path.write_text(yaml.safe_dump(config))
        with open('/tmp/group4_recovery_nav2.log','w') as log:
            proc=subprocess.Popen(['ros2','launch','ee3033_project','navigation.launch.py',
                'nav_config:='+str(path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            rclpy.init()
            chassis=RecoveryChassis()
            gate=VelocityGate(parameter_overrides=[Parameter('allow_motion',value=True),
                Parameter('output_topic',value='/test/turn_output')])
            executor=MultiThreadedExecutor(num_threads=4)
            for node in (chassis,gate):executor.add_node(node)
            thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
            nav=ActionClient(chassis,NavigateToPose,'/navigate_to_pose')
            backup=ActionClient(chassis,BackUp,'/backup')
            cost=chassis.create_client(GetCostmap,'/global_costmap/get_costmap')
            handle=None

            def wall(blocked,unknown_rear=False):
                chassis.map_wall(blocked,unknown_rear)
                def updated():
                    m=wait(cost.call_async(GetCostmap.Request())).map
                    col=math.floor((.375-m.metadata.origin.position.x)/m.metadata.resolution)
                    row=math.floor((.025-m.metadata.origin.position.y)/m.metadata.resolution)
                    return (m.data[row*m.metadata.size_x+col]>=253)==blocked
                until(updated)

            def begin(yaw=0., frozen=False):
                chassis.running=False;time.sleep(.15)
                chassis.xyz=[0.,0.,yaw];chassis.command=(0.,0.);chassis.commands=[]
                chassis.frozen=frozen
                time.sleep(.3)
                goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
                goal.pose.header.stamp=chassis.get_clock().now().to_msg()
                goal.pose.pose.position.x=.75
                set_yaw(goal.pose.pose.orientation,0.)
                chassis.running=True
                h=wait(nav.send_goal_async(goal));assert h.accepted
                return h

            def reverse(seconds=8):
                chassis.xyz=[0.,0.,0.];chassis.command=(0.,0.);chassis.commands=[]
                chassis.frozen=False;chassis.forward_blocked=False
                chassis.running=False;time.sleep(.7);chassis.running=True
                goal=BackUp.Goal();goal.target.x=.20;goal.speed=.04
                goal.time_allowance.sec=seconds
                h=wait(backup.send_goal_async(goal));assert h.accepted
                return h

            def counts():return {name:len(ids) for name,ids in chassis.behaviors.items()}

            def finish(h, seconds=10):
                result=wait(h.get_result_async(),seconds)
                until(lambda: all(abs(v)<1e-6 for v in chassis.command),2)
                chassis.running=False
                return result

            try:
                chassis.map_wall(True)
                for name in ('controller_server','planner_server','smoother_server','behavior_server','bt_navigator'):
                    client=chassis.create_client(GetState,'/'+name+'/get_state')
                    assert client.wait_for_service(timeout_sec=25)
                    until(lambda:wait(client.call_async(GetState.Request())).current_state.id==3,30)
                assert nav.wait_for_server(timeout_sec=10) and cost.wait_for_service(timeout_sec=5)
                assert backup.wait_for_server(timeout_sec=5)
                getter=chassis.create_client(GetParameters,'/bt_navigator/get_parameters')
                assert getter.wait_for_service(timeout_sec=5)
                loaded=wait(getter.call_async(GetParameters.Request(names=['default_nav_to_pose_bt_xml'])))
                installed=Path(loaded.values[0].string_value)
                assert installed.name=='navigate_with_bounded_recovery.xml'
                assert installed.read_text()==(root/'src/ee3033_project/behavior_trees'/installed.name).read_text()
                assert chassis.count_publishers('/cmd_vel')==0

                # Prove the live deployment procedure can reload only Nav2,
                # keeping the map publisher and chassis/TF fixture running.
                manager=chassis.create_client(ManageLifecycleNodes,'/project_navigation_lifecycle/manage_nodes')
                setter=chassis.create_client(SetParametersAtomically,'/bt_navigator/set_parameters_atomically')
                assert manager.wait_for_service(timeout_sec=5) and setter.wait_for_service(timeout_sec=5)
                assert wait(manager.call_async(ManageLifecycleNodes.Request(command=3))).success # RESET
                reloaded=Path(tmp)/installed.name;reloaded.write_text(installed.read_text())
                update=wait(setter.call_async(SetParametersAtomically.Request(parameters=[
                    Parameter('default_nav_to_pose_bt_xml',value=str(reloaded)).to_parameter_msg()])))
                assert update.result.successful,update.result.reason
                assert wait(manager.call_async(ManageLifecycleNodes.Request(command=0)),30).success # STARTUP
                print('PASS Nav2-only lifecycle reload: map and simulated chassis preserved',flush=True)

                wall(False);chassis.localization_valid=True
                handle=reverse();result=finish(handle,10);handle=None
                assert result.status==4 and -.23<chassis.xyz[0]<-.18,(result.status,chassis.xyz)
                assert any(v<-.01 for v,w in chassis.commands)
                assert all(v>=-.040001 for v,w in chassis.commands)
                print(f'PASS no rear laser returns: map/AMCL-checked reverse {chassis.xyz}',flush=True)

                wall(False,True);handle=reverse(2)
                result=finish(handle,4);handle=None
                assert result.status==6 and abs(chassis.xyz[0])<.01,(result.status,chassis.xyz)
                assert not any(v<-.001 for v,w in chassis.commands)
                print('PASS unknown rear map prevents reversing',flush=True)

                wall(False);chassis.localization_valid=False;handle=reverse(2)
                result=finish(handle,4);handle=None
                assert result.status==6 and abs(chassis.xyz[0])<.01,(result.status,chassis.xyz)
                assert not any(v<-.001 for v,w in chassis.commands)
                print('PASS uncertain localization prevents reversing',flush=True)

                wall(True);before=counts();started=time.monotonic();handle=begin()
                result=finish(handle,15);handle=None;elapsed=time.monotonic()-started
                assert result.status==6 and elapsed<13,(result.status,elapsed)
                assert counts()['wait']==before['wait']+1,counts()
                assert counts()['backup']==before['backup']+1,counts()
                assert chassis.commands and all(abs(v)<1e-6 and abs(w)<1e-6 for v,w in chassis.commands)
                print(f'PASS persistent no-path: abort {elapsed:.2f}s; one gated backup fallback, no motion',flush=True)

                chassis.localization_valid=True
                wall(True);before=counts();handle=begin(yaw=2.)
                until(lambda:counts()['wait']==before['wait']+1)
                wall(False)
                result=finish(handle,25);handle=None
                assert result.status==4 and chassis.xyz[0]>.60,(result.status,chassis.xyz)
                turns=[w for v,w in chassis.commands if abs(w)>.10]
                assert turns and turns[0]<0,turns[:10]
                assert counts()['backup']==before['backup'],counts()
                print(f'PASS transient blockage: path recovers; clockwise path alignment and arrival {chassis.xyz}',flush=True)

                # A chassis caught against a corner cannot move forward until a
                # bounded retreat creates space. The map remains free and Nav2
                # owns every command; reverse is reached only after replan fails.
                wall(False);before=counts();handle=begin()
                chassis.forward_blocked=True
                result=finish(handle,40);handle=None
                assert result.status==4 and chassis.xyz[0]>.60,(result.status,chassis.xyz)
                assert counts()['backup']==before['backup']+1,counts()
                assert any(v<-.01 for v,w in chassis.commands)
                print(f'PASS constrained corner: one fallback retreat then forward arrival {chassis.xyz}',flush=True)

                wall(False);before=counts();started=time.monotonic();handle=begin(frozen=True)
                result=finish(handle,18);handle=None;elapsed=time.monotonic()-started
                assert result.status==6 and elapsed<17,(result.status,elapsed)
                assert counts()['wait']==before['wait']+1,counts()
                assert counts()['backup']==before['backup']+1,counts()
                assert chassis.xyz==[0.,0.,0.]
                print(f'PASS frozen chassis: controller retries once then aborts in {elapsed:.2f}s',flush=True)

                wall(True);before=counts();handle=begin()
                until(lambda:counts()['wait']==before['wait']+1)
                started=time.monotonic();wait(handle.cancel_goal_async())
                result=finish(handle,3);handle=None
                assert result.status==5 and time.monotonic()-started<2,result.status
                old=counts();time.sleep(1.2)
                assert counts()==old and all(abs(v)<1e-6 for v in chassis.command)
                assert counts()['spin']==before['spin'] and counts()['backup']==before['backup'],counts()
                print('PASS cancel during recovery: terminal stop; no successor or fallback after cancel',flush=True)
                assert chassis.count_publishers('/cmd_vel')==0
            finally:
                chassis.running=False
                if handle:handle.cancel_goal_async()
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
                executor.shutdown(timeout_sec=3);thread.join(timeout=3)
                for node in (gate,chassis):node.destroy_node()
                rclpy.shutdown()


if __name__=='__main__':main()
