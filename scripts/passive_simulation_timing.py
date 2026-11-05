#!/usr/bin/env python3
"""Passive simulation timing evidence. No controls, inference or data publishers.

Records source/receipt times for TF, sensors, mission and detector events.
Readiness requires actual fresh packets; Humble per-sample publisher IDs are unavailable.
"""
import argparse
import gzip
import json
import math
import os
from pathlib import Path
import signal
import time


class StreamStats:
    def __init__(self):
        self.count=0;self.last_receipt=None;self.last_stamp=None
        self.maximum_receive_gap=0.;self.minimum_age=None;self.maximum_age=None
        self.backward_stamps=0

    def add(self,receipt,stamp,monotonic):
        if self.last_receipt is not None:
            self.maximum_receive_gap=max(self.maximum_receive_gap,monotonic-self.last_receipt)
        self.count+=1;self.last_receipt=monotonic
        if stamp is not None:
            age=receipt-stamp
            if self.last_stamp is not None and stamp<self.last_stamp:self.backward_stamps+=1
            self.last_stamp=stamp
            self.minimum_age=age if self.minimum_age is None else min(self.minimum_age,age)
            self.maximum_age=age if self.maximum_age is None else max(self.maximum_age,age)

    def snapshot(self):
        return dict(samples=self.count,maximum_receive_gap_seconds=self.maximum_receive_gap,
            minimum_header_age_seconds=self.minimum_age,maximum_header_age_seconds=self.maximum_age,
            last_header_stamp_seconds=self.last_stamp,backward_header_stamps=self.backward_stamps)


def write_json(path,data):
    temporary=path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    temporary.replace(path)


TARGET_KEYS=('x','y','sigma','quality','source','stamp','observation_stamp','accepted_at',
    'source_frames','first_source_stamp','last_source_stamp','confidence','frames','samples',
    'centered','measurement_valid','single_frame','reportable','class')
DETECTION_KEYS=('stamp','label','confidence','box','image_width','image_height','search_eligible',
    'projection_valid','bearing_valid','range_model','ranging_mode','range_m','x','y','sigma',
    'origin','reason','reason_codes','center_error_px','settled')
MISSION_KEYS=('phase','state','reason','stamp','started_stamp','search_epoch','epoch_started_stamp',
    'elapsed_seconds','target_detected','detection_stamp','target_ever_detected','target_accepted_at',
    'target_reported','return_verified','return_distance','return_reason','stationary_check',
    'stationary_linear_speed','stationary_angular_speed','viewpoints_completed','viewpoints_total',
    'viewpoints_attempted','available_viewpoints','observed_fraction','coverable_fraction',
    'termination_reason','internal_error','home')


def compact_payload(stream,data):
    if not isinstance(data,dict):raise ValueError('Expected a JSON object')
    if stream=='target_detections':
        return dict(stamp=data.get('stamp'),detections=[{key:d[key] for key in DETECTION_KEYS if key in d}
            for d in data.get('detections',[]) if isinstance(d,dict)])
    keys=MISSION_KEYS if stream=='mission_status' else ('event','stamp','started_stamp','search_epoch','target_reported','phase','state')
    result={key:data[key] for key in keys if key in data}
    if isinstance(data.get('target'),dict):result['target']={key:data['target'][key] for key in TARGET_KEYS if key in data['target']}
    return result


def json_safe(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {key:json_safe(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):return [json_safe(item) for item in value]
    return value


def message_callback(handler,*suffix):
    # Installed Humble invokes callbacks with the message only.
    def callback(message):return handler(message,None,*suffix)
    return callback


def string_callback(stream,handler):
    def callback(message):return handler(stream,message,None)
    return callback


def stream_readiness(stats,lookups,ros_now,monotonic_now,navigation_required=False):
    required=('odom','tf:odom_combined->base_footprint','scan_fixed','mission_status','target_detections','cmd_vel')
    if navigation_required:required+=('scan_navigation',)
    streams={}
    for key in required:
        item=stats.get(key)
        age=ros_now-item.last_stamp if item is not None and item.last_stamp is not None else None
        receipt_age=monotonic_now-item.last_receipt if item is not None and item.last_receipt is not None else None
        fresh=bool(item and item.count and receipt_age is not None and 0<=receipt_age<=1.
                   and (key=='cmd_vel' or age is not None and -.5<=age<=1.))
        streams[key]=dict(samples=item.count if item else 0,source_age_seconds=age,
                          receipt_age_seconds=receipt_age,fresh=fresh)
    tf_age=lookups.get('latest_age_seconds')
    tf_receipt_age=monotonic_now-lookups.get('latest_receipt_monotonic',-math.inf)
    composed=bool(lookups.get('latest_available') and tf_age is not None and -.5<=tf_age<=.5 and tf_receipt_age<=1.)
    return dict(ready=all(item['fresh'] for item in streams.values()) and composed,
                updated_wall_seconds=time.time(),streams=streams,composed_tf_fresh=composed)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='New .jsonl.gz evidence file')
    parser.add_argument('--duration',type=float,default=1800.,help='Maximum recording seconds; SIGINT flushes sooner')
    parser.add_argument('--navigation-geometry',action='store_true',help='Diagnostic-only full scan/AMCL/plan/arc/upstream command capture')
    parser.add_argument('--require-navigation-scan',action='store_true',help='Require fresh independent navigation scan packets before readiness')
    parser.add_argument('--ready-file',type=Path,help='Defaults to OUTPUT.ready.json')
    args=parser.parse_args()
    if not math.isfinite(args.duration) or not 1<=args.duration<=43200:
        parser.error('--duration must be between 1 and 43200 seconds')
    domain=os.environ.get('ROS_DOMAIN_ID')
    if domain in (None,'0','6') or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        parser.error('Requires a private non-robot ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    if not args.output.name.endswith('.jsonl.gz'):parser.error('--output must end in .jsonl.gz')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    ready=args.ready_file or args.output.with_name(args.output.name+'.ready.json')
    summary=args.output.with_name(args.output.name+'.summary.json')
    ready.parent.mkdir(parents=True,exist_ok=True)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry, Path as NavPath
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from std_msgs.msg import String

    stopping=[None]
    def stop(signum,frame):stopping[0]='signal_'+str(signum)
    rclpy.init()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    node=Node('passive_simulation_timing_'+str(os.getpid()),enable_rosout=False,start_parameter_services=False)
    buffer=Buffer();listener=TransformListener(buffer,node)
    stats={};lookups=dict(samples=0,failures=0,maximum_age_seconds=None)
    started=time.monotonic();error=None
    def stamp(value):return value.sec+value.nanosec*1e-9
    def gid(info):return bytes(info.publisher_gid).hex() if info is not None else None
    with gzip.open(args.output,'xt',encoding='utf-8',compresslevel=1) as output:
        def emit(event):output.write(json.dumps(json_safe(event),separators=(',',':'),allow_nan=False)+'\n')
        def record(key,source_stamp,info,**values):
            receipt=node.get_clock().now().nanoseconds*1e-9;monotonic=time.monotonic()
            stats.setdefault(key,StreamStats()).add(receipt,source_stamp,monotonic)
            emit(dict(event='sample',stream=key,receipt_ros_seconds=receipt,
                receipt_wall_seconds=time.time(),receipt_monotonic_seconds=monotonic,
                header_stamp_seconds=source_stamp,header_age_seconds=receipt-source_stamp if source_stamp is not None else None,
                publisher_gid=gid(info),**values))
        def on_tf(message,info,static=False):
            for transform in message.transforms:
                record(('tf_static:' if static else 'tf:')+transform.header.frame_id+'->'+transform.child_frame_id,
                    stamp(transform.header.stamp),info,parent=transform.header.frame_id,child=transform.child_frame_id,
                    translation=[transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z],
                    rotation_xyzw=[transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w])
        def on_scan(message,info,stream='scan_fixed'):
            geometry=dict(ranges=list(message.ranges),angle_min=message.angle_min,angle_max=message.angle_max,
                angle_increment=message.angle_increment,range_min=message.range_min,range_max=message.range_max) if args.navigation_geometry else {}
            record(stream,stamp(message.header.stamp),info,frame=message.header.frame_id,
                samples=len(message.ranges),scan_time=message.scan_time,time_increment=message.time_increment,**geometry)
        def on_odom(message,info):
            record('odom',stamp(message.header.stamp),info,frame=message.header.frame_id,child=message.child_frame_id,
                linear_x=message.twist.twist.linear.x,angular_z=message.twist.twist.angular.z)
        def pose_values(pose):
            return [pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        def on_amcl(message,info):
            geometry=dict(pose=pose_values(message.pose.pose),covariance=list(message.pose.covariance)) if args.navigation_geometry else {}
            record('amcl_pose',stamp(message.header.stamp),info,frame=message.header.frame_id,**geometry)
        def on_path(stream,message,info):
            record(stream,stamp(message.header.stamp),info,frame=message.header.frame_id,
                poses=[pose_values(item.pose) for item in message.poses])
        def on_command(message,info,stream='cmd_vel'):
            # Twist has no header. Preserve receipt time without inventing a
            # command-generation timestamp or zero delivery latency.
            record(stream,None,info,linear_x=message.linear.x,angular_z=message.angular.z)
        def on_string(stream,message,info):
            try:
                decoded=json.loads(message.data)
                payload=decoded if stream in ('velocity_gate_status','navigation_status','simulation_motion','navigation_scan_status') else compact_payload(stream,decoded)
                source=payload.get('stamp')
                source=float(source) if isinstance(source,(int,float)) and not isinstance(source,bool) and math.isfinite(source) else None
                # Detection stamp=exposure; status stamp=publication. Result
                # events lack a publication stamp, so their receipt is a bound.
                record(stream,source,info,payload=payload)
            except (ValueError,TypeError,KeyError) as exc:
                emit(dict(event='decode_error',stream=stream,receipt_wall_seconds=time.time(),error=str(exc)[:200]))
        def lookup():
            receipt=node.get_clock().now().nanoseconds*1e-9;lookups['samples']+=1
            try:
                transform=buffer.lookup_transform('map','base_footprint',Time());age=receipt-stamp(transform.header.stamp)
                previous=lookups['maximum_age_seconds'];lookups['maximum_age_seconds']=age if previous is None else max(previous,age)
                lookups.update(latest_available=True,latest_age_seconds=age,latest_receipt_monotonic=time.monotonic())
                emit(dict(event='composed_tf',receipt_ros_seconds=receipt,receipt_monotonic_seconds=time.monotonic(),
                    available=True,header_stamp_seconds=stamp(transform.header.stamp),age_seconds=age))
            except TransformException as exc:
                lookups['failures']+=1
                lookups.update(latest_available=False,latest_age_seconds=None,latest_receipt_monotonic=time.monotonic())
                emit(dict(event='composed_tf',receipt_ros_seconds=receipt,receipt_monotonic_seconds=time.monotonic(),available=False,error=str(exc)[:400]))
        dynamic=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT)
        static=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(TFMessage,'/tf',message_callback(on_tf),dynamic)
        node.create_subscription(TFMessage,'/tf_static',message_callback(on_tf,True),static)
        node.create_subscription(LaserScan,'/scan_fixed',message_callback(on_scan),qos_profile_sensor_data)
        if args.require_navigation_scan or args.navigation_geometry:
            node.create_subscription(LaserScan,'/scan_navigation',message_callback(on_scan,'scan_navigation'),qos_profile_sensor_data)
        node.create_subscription(Odometry,'/odom',message_callback(on_odom),qos_profile_sensor_data)
        node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',message_callback(on_amcl),10)
        node.create_subscription(Twist,'/cmd_vel',message_callback(on_command),dynamic)
        node.create_subscription(String,'/project/target_detections',string_callback('target_detections',on_string),dynamic)
        node.create_subscription(String,'/project/mission_status',string_callback('mission_status',on_string),dynamic)
        node.create_subscription(String,'/project/mission_result',string_callback('mission_result',on_string),dynamic)
        if args.navigation_geometry:
            node.create_subscription(Twist,'/project/nav_cmd_vel',message_callback(on_command,'nav_cmd_vel'),dynamic)
            node.create_subscription(LaserScan,'/scan',message_callback(on_scan,'scan_raw'),qos_profile_sensor_data)
            for topic in ('/plan','/local_plan','/lookahead_arc','/lookahead_collision_arc'):
                node.create_subscription(NavPath,topic,string_callback(topic.lstrip('/'),on_path),dynamic)
            for stream,topic in (('velocity_gate_status','/project/velocity_gate_status'),('navigation_status','/project/navigation_status'),('simulation_motion','/project/simulation_motion'),('navigation_scan_status','/project/navigation_scan_status')):
                node.create_subscription(String,topic,string_callback(stream,on_string),dynamic)
        node.create_timer(.2,lookup)
        metadata=dict(event='metadata',read_only=True,ros_domain=domain,pid=os.getpid(),
            started_wall_seconds=time.time(),ready_kind='Actual odom, TF, scan, mission, detector and command packets plus fresh composed TF observed',
            output=str(args.output),maximum_duration_seconds=args.duration,navigation_geometry=args.navigation_geometry,required_navigation_scan=args.require_navigation_scan,
            result_timestamp_note='mission_result and cmd_vel have no source publication header; receipt timestamps are recorded, not fabricated source times.',
            qos_note='Passive best-effort TF/sensor logging avoids backpressure; logger gaps alone do not prove publisher gaps.')
        metadata.update(per_sample_publisher_gid_available=False,publisher_gid_note='Installed Humble supplies message-only callbacks; sample GIDs are null, graph GIDs are separate inventories.')
        emit(metadata);output.flush()
        flush_at=time.monotonic()+1.;graph_at=time.monotonic()+1.;last_graph=None
        readiness_at=time.monotonic();announced=False
        try:
            while rclpy.ok() and stopping[0] is None and time.monotonic()-started<args.duration:
                rclpy.spin_once(node,timeout_sec=.05)
                now=time.monotonic()
                if now>=flush_at:output.flush();flush_at=now+1.
                if now>=readiness_at:
                    status=stream_readiness(stats,lookups,node.get_clock().now().nanoseconds*1e-9,now,args.require_navigation_scan)
                    write_json(ready,status)
                    if status['ready'] and not announced:
                        print('TIMING_LOGGER_READY '+json.dumps(dict(output=str(args.output),ready_file=str(ready),pid=os.getpid())),flush=True)
                        announced=True
                    readiness_at=now+.5
                if now>=graph_at:
                    graph={topic:[dict(node=entry.node_namespace.rstrip('/')+'/'+entry.node_name,
                        publisher_gid=bytes(entry.endpoint_gid).hex()) for entry in node.get_publishers_info_by_topic(topic)]
                        for topic in ('/tf','/scan_fixed','/scan_navigation','/odom','/amcl_pose','/cmd_vel',
                            '/project/target_detections','/project/mission_status','/project/mission_result')}
                    if graph!=last_graph:emit(dict(event='publisher_graph',receipt_wall_seconds=time.time(),topics=graph));last_graph=graph
                    graph_at=now+5.
        except KeyboardInterrupt:stopping[0]='keyboard_interrupt'
        except Exception as exc:error=str(exc)
        finally:
            final=dict(read_only=True,ros_domain=domain,duration_seconds=time.monotonic()-started,
                stop_reason=stopping[0] or ('error' if error else 'duration'),error=error,
                streams={key:value.snapshot() for key,value in stats.items()},composed_tf=lookups,
                initial_discovery_lookup_failures_included=True,
                final_readiness=stream_readiness(stats,lookups,node.get_clock().now().nanoseconds*1e-9,time.monotonic(),args.require_navigation_scan))
            emit(dict(event='summary',**final));output.flush();write_json(summary,final)
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
    print('TIMING_LOGGER_SAVED '+str(summary),flush=True)
    return 1 if error else 0


if __name__=='__main__':raise SystemExit(main())
