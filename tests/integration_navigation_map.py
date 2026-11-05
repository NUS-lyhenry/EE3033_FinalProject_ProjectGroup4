"""Real Nav2 static-layer/inflation resolution check; no drivers or motion."""
import os
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from nav2_msgs.srv import GetCostmap
from lifecycle_msgs.srv import ChangeState
from ee3033_project.navigation_map import NavigationMap, refine_map
from ee3033_project.ros_utils import LATCHED
from ee3033_project.path_validation import guarded_plan, costmap_mask, path_search_grid
from ee3033_project.robot_geometry import NAVIGATION_RADIUS, FOOTPRINT_PADDING


def wait(future):
    until=time.monotonic()+15
    while not future.done() and time.monotonic()<until:time.sleep(.02)
    assert future.done(),'Service timed out'
    return future.result()


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6')
    rclpy.init();world=Node('navigation_map_test_world');relay=NavigationMap()
    executor=MultiThreadedExecutor(num_threads=3)
    for node in (world,relay):executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    publisher=world.create_publisher(OccupancyGrid,'/map',LATCHED)
    tf=StaticTransformBroadcaster(world)
    transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='base_footprint'
    transform.header.stamp=world.get_clock().now().to_msg();transform.transform.rotation.w=1.
    transform.transform.translation.x=.8;transform.transform.translation.y=1.5
    tf.sendTransform(transform)
    with tempfile.TemporaryDirectory() as tmp:
        config=yaml.safe_load((Path(__file__).parents[1]/'src/ee3033_project/config/navigation.yaml').read_text())
        params=config['global_costmap']['global_costmap']['ros__parameters']
        # Static synthetic room: no laser, base driver, controller or velocity topic.
        params['plugins']=['static_layer','inflation_layer']
        params['robot_radius']=NAVIGATION_RADIUS;params['footprint_padding']=FOOTPRINT_PADDING
        file=Path(tmp)/'params.yaml';file.write_text(yaml.safe_dump(config))
        log=open('/tmp/navigation_map_native_test.log','w')
        process=subprocess.Popen(['ros2','run','nav2_planner','planner_server','--ros-args','--params-file',str(file)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            def publish(width):
                grid=np.zeros((60,60),dtype=np.int8);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
                grid[:,30]=100;n=round(width/.05);lo=30-n//2;grid[lo:lo+n,30]=0
                message=OccupancyGrid();message.header.frame_id='map';message.header.stamp=world.get_clock().now().to_msg()
                message.info.width=message.info.height=60;message.info.resolution=.05;message.info.origin.orientation.w=1.
                message.data=grid.ravel().tolist()
                refined=refine_map(message)
                assert refined.header==message.header and refined.info.origin==message.info.origin
                assert np.array_equal(np.array(refined.data).reshape(120,120),np.repeat(np.repeat(grid,2,0),2,1))
                publisher.publish(message)
                return grid
            grid=publish(.35)
            client=world.create_client(ChangeState,'/planner_server/change_state')
            assert client.wait_for_service(timeout_sec=20)
            for transition in (1,3):
                request=ChangeState.Request();request.transition.id=transition
                assert wait(client.call_async(request)).success
            getter=world.create_client(GetCostmap,'/global_costmap/get_costmap');assert getter.wait_for_service(timeout_sec=10)
            for width,expected in ((.35,True),(.25,False),(.20,False),(.35,True)):
                grid=publish(width);deadline=time.monotonic()+10
                while True:
                    cost=wait(getter.call_async(GetCostmap.Request())).map
                    cm=cost.metadata;origin=(cm.origin.position.x,cm.origin.position.y,0.)
                    if cm.size_x==120 and cm.size_y==120 and abs(cm.resolution-.025)<1e-8:
                        costs=np.array(cost.data).reshape(cm.size_y,cm.size_x)
                        fine,res,_=path_search_grid(grid,.05)
                        mask=costmap_mask(fine.shape,res,(0.,0.,0.),costs,cm.resolution,origin)
                        plan=guarded_plan(fine,res,(0.,0.,0.),(.8,1.5),(2.2,1.5),mask=mask)
                        if plan.allowed==expected:break
                    assert time.monotonic()<deadline,(width,cm,plan.reason if 'plan' in locals() else None)
                    time.sleep(.2)
                print('PASS actual Nav2: source=5cm costmap=2.5cm opening=',width,'route=',plan.allowed,flush=True)
            assert world.count_publishers('/cmd_vel')==0
            print('PASS footprint/padding/inflation unchanged; no chassis output',flush=True)
        except Exception:
            log.flush();print(Path(log.name).read_text()[-6000:],flush=True)
            raise
        finally:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            log.close()
    executor.shutdown(timeout_sec=3);thread.join(timeout=3)
    relay.destroy_node();world.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
