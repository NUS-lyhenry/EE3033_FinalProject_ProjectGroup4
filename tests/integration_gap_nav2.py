"""Real Humble costmaps + NavFn passage tests. No drivers or motion goals.

Run on an isolated domain. Both costmaps use a synthetic static layer so the
same passage exercises hard inflation locally and globally, independently of
lidar sampling. Nav2's other settings come from the project configuration.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
from urllib.request import Request, urlopen

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from lifecycle_msgs.srv import GetState
from tf2_ros import StaticTransformBroadcaster
from ee3033_project.console import Console
from ee3033_project.mapper import Mapper
from ee3033_project.ros_utils import LATCHED


def wait(future,seconds=10):
    deadline=time.monotonic()+seconds
    while not future.done() and time.monotonic()<deadline: time.sleep(.02)
    assert future.done(), 'ROS request timed out'
    return future.result()


def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
        raise SystemExit('Use an isolated ROS_DOMAIN_ID, e.g. 91')
    resolution=float(os.environ.get('GROUP4_GAP_RESOLUTION','.01'))
    narrow=.30 if resolution==.05 else .34
    with tempfile.TemporaryDirectory(prefix='group4_gap_') as tmp:
        config=yaml.safe_load((Path(__file__).resolve().parents[1]/'src/ee3033_project/config/navigation.yaml').read_text())
        # Test fixtures only: the live lidar obstacle layer is not bypassed in the project.
        for name in ('local_costmap','global_costmap'):
            p=config[name][name]['ros__parameters']
            p['plugins']=['static_layer','inflation_layer']
            p['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_subscribe_transient_local':True}
            p['resolution']=resolution
            p['update_frequency']=5.
        path=Path(tmp)/'nav.yaml';path.write_text(yaml.safe_dump(config))
        with open('/tmp/group4_gap_nav2.log','w') as log:
            proc=subprocess.Popen(['ros2','launch','ee3033_project','navigation.launch.py','nav_config:='+str(path)],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            rclpy.init()
            world=Node('gap_test_world')
            console=Console(parameter_overrides=[Parameter('port',value=0),Parameter('phase',value='mapping')])
            mapper=Mapper()
            tf=StaticTransformBroadcaster(world)
            transforms=[]
            for parent,child,x in [('map','odom_combined',0.),('odom_combined','base_footprint',-.8)]:
                t=TransformStamped();t.header.stamp=world.get_clock().now().to_msg()
                t.header.frame_id,t.child_frame_id=parent,child;t.transform.translation.x=x
                t.transform.rotation.w=1.;transforms.append(t)
            tf.sendTransform(transforms)
            maps=world.create_publisher(OccupancyGrid,'/map',LATCHED)
            executor=MultiThreadedExecutor(num_threads=4)
            for node in (world,console,mapper): executor.add_node(node)
            thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
            base=f'http://127.0.0.1:{console.p["port"]}'
            planner=ActionClient(world,ComputePathToPose,'/compute_path_to_pose')
            getters={name:world.create_client(GetCostmap,'/'+name+'/get_costmap') for name in ('global_costmap','local_costmap')}
            def publish_passage(width):
                # 3 m sealed room split by a wall with an integer-cell opening.
                n=round(3/resolution);mid=n//2;grid=np.zeros((n,n),np.int8);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
                grid[:,mid]=100
                cells=round(width/resolution);low=mid-cells//2;grid[low:low+cells,mid]=0
                msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=world.get_clock().now().to_msg()
                msg.info.width=msg.info.height=n;msg.info.resolution=resolution
                msg.info.origin.position.x=msg.info.origin.position.y=-1.5;msg.info.origin.orientation.w=1.
                msg.data=grid.ravel().tolist();maps.publish(msg)
            def post(enabled):
                req=Request(base+'/api/gap',json.dumps({'enabled':enabled}).encode(),
                    {'Content-Type':'application/json','X-Project-Token':console.token},method='POST')
                result=json.load(urlopen(req,timeout=20));assert result['synchronized'];return result
            def center_cost(name):
                grid=wait(getters[name].call_async(GetCostmap.Request())).map
                m=grid.metadata
                c=math.floor((.005-m.origin.position.x)/m.resolution)
                row=math.floor((.005-m.origin.position.y)/m.resolution)
                return grid.data[row*m.size_x+c]
            def plan():
                goal=ComputePathToPose.Goal();goal.planner_id='GridBased';goal.use_start=True
                for pose,x in ((goal.start,-.8),(goal.goal,.8)):
                    pose.header.frame_id='map';pose.header.stamp=world.get_clock().now().to_msg()
                    pose.pose.position.x=x;pose.pose.position.y=.005;pose.pose.orientation.w=1.
                handle=wait(planner.send_goal_async(goal));assert handle.accepted
                result=wait(handle.get_result_async())
                return result.status==4 and len(result.result.path.poses)>0
            try:
                publish_passage(narrow)
                for name in ('planner_server','controller_server'):
                    client=world.create_client(GetState,'/'+name+'/get_state')
                    assert client.wait_for_service(timeout_sec=25)
                    deadline=time.monotonic()+30
                    while wait(client.call_async(GetState.Request())).current_state.id!=3:
                        assert time.monotonic()<deadline,'Inactive '+name;time.sleep(.2)
                assert planner.wait_for_server(timeout_sec=10)
                for client in getters.values(): assert client.wait_for_service(timeout_sec=5)
                for width,enabled,passable in [(narrow,False,True),(narrow,True,False),(.70,True,True),(.20,False,False),(narrow,False,True)]:
                    publish_passage(width);post(enabled)
                    # Wait for inflation to reflect changed footprint and a fresh map.
                    deadline=time.monotonic()+8
                    while True:
                        costs={name:center_cost(name) for name in getters}
                        if all((cost<253)==passable for cost in costs.values()): break
                        assert time.monotonic()<deadline,(width,enabled,costs);time.sleep(.15)
                    assert plan()==passable,(width,enabled,costs)
                    print(f'PASS native NavFn: grid={resolution} opening={width:.2f} m reserve={enabled} passable={passable} costs={costs}',flush=True)
                assert world.count_publishers('/cmd_vel')==0,'Fixture must not connect to chassis output'
                print('PASS: real global/local inflation and native NavFn ON/OFF; no chassis output',flush=True)
            finally:
                os.killpg(proc.pid,signal.SIGINT)
                try: proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
                executor.shutdown(timeout_sec=3);thread.join(timeout=3)
                for node in (console,mapper,world):node.destroy_node()
                rclpy.shutdown()


if __name__=='__main__': main()
