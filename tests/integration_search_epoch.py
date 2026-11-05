"""Real ROS transport epoch handshake and calibrated batch isolation.

An isolated domain hosts the production TargetFilter. Synthetic *observations*
exercise its production association, SettledBatch, map posterior and status
reset callbacks. No robot, velocity, navigation or hardware topics are used.
"""
import json
import os
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Bool

from ee3033_project.target_filter import TargetFilter
from ee3033_project.ros_utils import LATCHED
from integration_ros import overrides, wait


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6')
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    fixture=Node('search_epoch_transport_fixture')
    filt=TargetFilter(parameter_overrides=overrides(target_class='bottle'))
    executor=MultiThreadedExecutor(num_threads=3)
    for node in (fixture,filt):executor.add_node(node)
    estimates=[];diagnostics=[]
    fixture.create_subscription(String,'/project/target_estimate',lambda m:estimates.append(json.loads(m.data)),LATCHED)
    fixture.create_subscription(String,'/project/localization_diagnostics',lambda m:diagnostics.append(json.loads(m.data)),20)
    status=fixture.create_publisher(String,'/project/mission_status',LATCHED)
    maps=fixture.create_publisher(OccupancyGrid,'/map',LATCHED)
    observations=fixture.create_publisher(String,'/project/target_observations',10)
    results=fixture.create_publisher(String,'/project/mission_result',LATCHED)
    active=fixture.create_publisher(Bool,'/project/search_active',LATCHED)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert wait(lambda:status.get_subscription_count()>0 and observations.get_subscription_count()>0,timeout=5)
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.02
        grid.info.width=grid.info.height=150
        grid.info.origin.position.x=grid.info.origin.position.y=-1.
        grid.info.origin.orientation.w=1.;grid.data=[0]*(150*150)
        maps.publish(grid)
        assert wait(lambda:filt.tracker.signature is not None,timeout=5)
        now=lambda:fixture.get_clock().now().nanoseconds*1e-9
        mission_stamp=now()-.2
        def arm(epoch,cutoff,locked=True):
            status.publish(String(data=json.dumps(dict(started_stamp=mission_stamp,
                search_epoch=epoch,epoch_started_stamp=cutoff,target_detected=locked))))
        def frame(stamp=None):
            stamp=now() if stamp is None else stamp
            box=[300.,100.,340.,258.]
            d=dict(stamp=stamp,label='bottle',confidence=.95,box=box,image_width=640,
                bearing=0.,bearing_valid=True,origin=[0.,0.],range_model='bottom_inverse_17pt_20261031',
                settled=True,centered=True,projection_valid=True,capture_heading=0.,
                ranging_mode='calibrated',attitude=dict(valid=True),search_eligible=True)
            o=dict(stamp=stamp,label='bottle',confidence=.95,box=box,x=.7,y=0.,
                sigma=.03,method='bottom_inverse',source_frame=stamp,quality_flags=[])
            return dict(stamp=stamp,observations=[o],bearings=[d],diagnostics=dict(localization_method='bottom_inverse'))
        def send(data):
            before=len(diagnostics);observations.publish(String(data=json.dumps(data)))
            assert wait(lambda:len(diagnostics)>before,timeout=3),diagnostics[-1:]
            time.sleep(.04)
        active.publish(Bool(data=True))
        arm(0,mission_stamp)
        assert wait(lambda:filt.mission_stamp==mission_stamp and filt.target_locked
            and filt.active and any(e.get('search_epoch')==0
                and e.get('mission_stamp')==mission_stamp for e in estimates),timeout=3),estimates[-3:]
        send(frame())
        assert wait(lambda:any(e.get('reportable') for e in estimates),timeout=3),diagnostics[-2:]
        initial=next(e for e in reversed(estimates) if e.get('reportable'))
        assert initial['frames']==1 and initial['stable'] is False
        old_frame=frame()
        cutoff=now();arm(1,cutoff,False)
        assert wait(lambda:any(e.get('search_epoch')==1 and e.get('available') is False for e in estimates),timeout=3)
        assert filt.association.candidate is None
        send(old_frame);arm(0,mission_stamp)
        time.sleep(.1)
        assert filt.search_epoch==1 and filt.tracker.snapshot(now()) is None
        # A single new exposure is sufficient without a second lock frame.
        send(frame())
        assert wait(lambda:any(e.get('search_epoch')==1 and e.get('reportable') for e in estimates),timeout=3),estimates[-1:]
        good=next(e for e in reversed(estimates) if e.get('search_epoch')==1 and e.get('reportable'))
        assert len(good['source_frames'])==1 and good['source_frames'][0]>=cutoff
        assert good['mission_stamp']==mission_stamp and good['stable'] is False
        target=dict(good,estimator=good['method'],observation_stamp=good['stamp'],**{'class':'bottle'})
        results.publish(String(data=json.dumps(dict(event='target_accepted',target_reported=False,
            started_stamp=mission_stamp,search_epoch=1,target=target))))
        time.sleep(.1);assert filt.tracker.frozen is None
        results.publish(String(data=json.dumps(dict(event='target_reported',target_reported=True,
            started_stamp=mission_stamp,search_epoch=1,target=target))))
        assert wait(lambda:filt.tracker.frozen is not None,timeout=3)
        frozen=filt.tracker.snapshot(now()+3.)
        assert frozen['frames']==1 and frozen['stable'] is False and frozen['stale'] is True
        print(json.dumps(dict(native_search_epoch_passed=True,epoch_ack=True,
            old_exposure_rejected=True,reordered_status_rejected=True,
            first_new_exposure_accepted=True,accepted_does_not_freeze=True,
            final_freeze_preserves_single_frame_and_age=True,source_frames=1)),flush=True)
    finally:
        executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        for node in (fixture,filt):node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
