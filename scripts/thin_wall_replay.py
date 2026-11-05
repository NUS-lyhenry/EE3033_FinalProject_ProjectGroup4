#!/usr/bin/env python3
"""Replay identical deterministic scans through baseline and revised native SLAM.

Run after deployment in a hardware-free Humble container, on an unused private
DDS domain. This does not start a robot, Nav2, Console, or velocity publisher.
Maps are unmodified slam_toolbox outputs. Run full navigation separately.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from thin_wall_validation import cases, evaluate_map, fixture_scans


def save(path, value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def replay(config_path, case, scans, threshold, output, hold_seconds):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.executors import SingleThreadedExecutor
    from ee3033_project.core import quaternion_yaw
    from ee3033_project.ros_utils import LATCHED

    context=rclpy.context.Context();rclpy.init(context=context)
    node=rclpy.create_node('thin_wall_replay',context=context)
    executor=SingleThreadedExecutor(context=context);executor.add_node(node)
    maps=[];poses=[]
    topic='/thin_wall_replay/map'
    node.create_subscription(OccupancyGrid,topic,maps.append,LATCHED)
    node.create_subscription(PoseWithCovarianceStamped,'/thin_wall_replay/pose',poses.append,10)
    publisher=node.create_publisher(LaserScan,'/scan_fixed',qos_profile_sensor_data)
    dynamic=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
    mount=TransformStamped();mount.header.frame_id='base_footprint';mount.child_frame_id='thin_wall_laser'
    mount.transform.rotation.w=1.;static.sendTransform(mount)
    process=None
    try:
        with (output/'slam.log').open('w') as log:
            command=['/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node','--ros-args',
                '--params-file',str(config_path),'-p','resolution:=0.05',
                '-p',f'occupancy_threshold:={threshold}', '-p','min_pass_through:=2',
                '-r',f'/map:={topic}','-r','/pose:=/thin_wall_replay/pose']
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+15
            while publisher.get_subscription_count()==0:
                if process.poll() is not None:raise RuntimeError('SLAM exited during startup; inspect slam.log')
                if time.monotonic()>deadline:raise RuntimeError('SLAM did not subscribe to the replay scans')
                executor.spin_once(timeout_sec=.05)
            sequence=list(zip(case['poses'],scans))
            # Continue publishing the last observation while the periodic map
            # exporter catches up. Never fill scans or maps from ground truth.
            sequence.append((case['poses'][-1],scans[-1]))
            sample_number=0
            source_epoch_ns=node.get_clock().now().nanoseconds-100_000_000
            for index,(pose,ranges) in enumerate(sequence):
                repetitions=25 if index==len(sequence)-1 else math.ceil(hold_seconds/.1)
                for _ in range(repetitions):
                    next_sample=time.monotonic()+.1
                    if process.poll() is not None:raise RuntimeError('SLAM exited during replay; inspect slam.log')
                    # Identical count and relative timestamps in both runs,
                    # including the 0.5 s production SLAM acceptance interval.
                    stamp=Time(nanoseconds=source_epoch_ns+sample_number*100_000_000).to_msg()
                    sample_number+=1
                    transform=TransformStamped();transform.header.stamp=stamp
                    transform.header.frame_id='odom_combined';transform.child_frame_id='base_footprint'
                    transform.transform.translation.x=float(pose[0]);transform.transform.translation.y=float(pose[1])
                    transform.transform.rotation.z=math.sin(pose[2]/2);transform.transform.rotation.w=math.cos(pose[2]/2)
                    dynamic.sendTransform(transform)
                    scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='thin_wall_laser'
                    scan.angle_min=-math.pi;scan.angle_increment=math.tau/len(ranges)
                    scan.angle_max=scan.angle_min+(len(ranges)-1)*scan.angle_increment
                    scan.range_min=.08;scan.range_max=8.;scan.scan_time=.1
                    scan.ranges=ranges
                    # Give the transform its own delivery opportunity before
                    # publishing the scan with the exact same acquisition time.
                    executor.spin_once(timeout_sec=.01);publisher.publish(scan)
                    while time.monotonic()<next_sample:executor.spin_once(timeout_sec=.01)
            if not maps or not poses:raise RuntimeError('SLAM produced no raw map or accepted pose')
            latest=poses[-1].header.stamp
            accepted_ns=latest.sec*1_000_000_000+latest.nanosec
            lag_ns=source_epoch_ns+(sample_number-1)*100_000_000-accepted_ns
            if lag_ns>700_000_000:raise RuntimeError('SLAM did not process the end of the frozen replay')
            msg=maps[-1];origin=msg.info.origin
            data=dict(width=msg.info.width,height=msg.info.height,resolution=msg.info.resolution,
                origin=[origin.position.x,origin.position.y,quaternion_yaw(origin.orientation)],cells=list(msg.data))
            save(output/'map.json',data)
            return dict(evaluate_map(data,case),scans_published=sample_number,accepted_pose_samples=len(poses),
                accepted_pose_lag_seconds=lag_ns/1e9)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
        executor.shutdown(timeout_sec=2)
        node.destroy_node();rclpy.shutdown(context=context)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    root=Path(__file__).resolve().parents[1]
    parser.add_argument('--config',type=Path,default=root/'src/ee3033_project/config/slam.yaml')
    parser.add_argument('--fixtures',type=Path,default=root/'tests/fixtures/thin_wall_cases.json')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',action='append',help='Case ID; repeat to select several, otherwise run all eight')
    parser.add_argument('--opening-width',type=float,choices=[.30,.35],
        help='Metres: mandatory 0.30 by default; 0.35 comparison uses the same fixtures')
    parser.add_argument('--hold-seconds',type=float,default=1.1)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6','121') or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        parser.error('Requires an unused isolated ROS_DOMAIN_ID (not 0, 6 or 121) and ROS_LOCALHOST_ONLY=1')
    if not Path('/.dockerenv').exists():parser.error('Run inside the hardware-free ROS Humble container')
    if not math.isfinite(args.hold_seconds) or args.hold_seconds<1.1:parser.error('--hold-seconds must be at least 1.1')
    settings,fixtures=cases(args.fixtures,args.opening_width)
    if settings['resolution']!=.05 or settings['beams']!=450:parser.error('This parity runner requires 5 cm maps and 450 beams')
    if args.case:
        selected=set(args.case);fixtures=[f for f in fixtures if f['case_id'] in selected]
        if selected!={f['case_id'] for f in fixtures}:parser.error('Unknown fixture case ID')
    args.output.mkdir(parents=True,exist_ok=False)
    rows=[]
    report=dict(simulated=True,kind='Native SLAM replay; not physical or navigation acceptance',
        fixture_sha256=hashlib.sha256(args.fixtures.read_bytes()).hexdigest(),
        slam_config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
        effective_overrides=dict(resolution=.05,min_pass_through=2),sensor=settings,
        replay=dict(sample_interval_seconds=.1,pose_repetitions=math.ceil(args.hold_seconds/.1),final_repetitions=25),
        ros_domain=os.environ['ROS_DOMAIN_ID'],rows=rows,passed=False)
    for case in fixtures:
        scans,digest=fixture_scans(case,settings['beams'],settings['mask_radius'])
        save(args.output/(case['case_id']+'-fixture.json'),case)
        # One cached sequence shared by both processes; only threshold changes.
        for label,threshold in [('baseline',.25),('revised',.10)]:
            output=args.output/(case['case_id']+'-'+label);output.mkdir()
            row=dict(case_id=case['case_id'],configuration=label,occupancy_threshold=threshold,scan_sha256=digest)
            try:row.update(replay(args.config,case,scans,threshold,output,args.hold_seconds))
            except Exception as exc:row.update(passed=False,error=str(exc))
            rows.append(row);save(args.output/'report.json',report)
            print(json.dumps(row,allow_nan=False),flush=True)
    revised=[row for row in rows if row['configuration']=='revised']
    report['complete']=len(rows)==2*len(fixtures) and all('error' not in row for row in rows)
    report['passed']=bool(report['complete'] and revised and all(row['passed'] for row in revised))
    save(args.output/'report.json',report)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
