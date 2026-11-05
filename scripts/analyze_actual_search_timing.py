#!/usr/bin/env python3
"""Post-run measurement only; no ROS, inference or control requests."""
import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path


def number(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


def analyze(events,state):
    mission=state['mission']
    start=mission['started_stamp']
    label=mission.get('target_class','bottle')
    raw=eligible=accepted=reported=None
    raw_received=eligible_received=None
    report_source=None
    terminal_receipt=None
    rows=[]
    for row in events:
        receipt=row.get('receipt_ros_seconds')
        if row.get('event')!='sample' or not number(receipt) or receipt<start:continue
        rows.append(row)
        stream=row.get('stream');data=row.get('payload') or {}
        if stream=='target_detections':
            stamp=data.get('stamp')
            if not number(stamp) or stamp<start:continue
            for detection in data.get('detections',[]):
                if detection.get('label')!=label:continue
                raw=stamp if raw is None else min(raw,stamp)
                raw_received=receipt if raw_received is None else min(raw_received,receipt)
                if detection.get('search_eligible') is True:
                    eligible=stamp if eligible is None else min(eligible,stamp)
                    eligible_received=receipt if eligible_received is None else min(eligible_received,receipt)
        if stream in ('mission_status','mission_result') and data.get('started_stamp')==start:
            candidates=[data.get('target_accepted_at'),(data.get('target') or {}).get('accepted_at')]
            for stamp in candidates:
                if number(stamp) and stamp>=start:accepted=stamp if accepted is None else min(accepted,stamp)
            if stream=='mission_result' and data.get('event')=='target_reported':
                if reported is None or receipt<reported:
                    reported=receipt;report_source='target_reported event receipt (publication stamp unavailable)'
            if stream=='mission_status' and data.get('state')==mission.get('state'):
                if terminal_receipt is None:terminal_receipt=receipt
    if accepted is None and number(mission.get('target_accepted_at')):
        accepted=mission['target_accepted_at']
    if reported is None and mission.get('target_reported'):
        candidates=[row['receipt_ros_seconds'] for row in rows
            if row.get('stream')=='mission_status' and (row.get('payload') or {}).get('started_stamp')==start
            and (row.get('payload') or {}).get('target_reported')]
        if candidates:
            reported=min(candidates);report_source='first reported Mission status receipt; report event not observed'
    finish=terminal_receipt if terminal_receipt is not None else start+mission.get('elapsed_seconds',0.)
    stop_rows=[row for row in rows if finish-.5<=row['receipt_ros_seconds']<=finish]
    commands=[row for row in stop_rows if row.get('stream')=='cmd_vel']
    odom=[row for row in stop_rows if row.get('stream')=='odom']
    def maximum(items,key):
        values=[abs(row[key]) for row in items if number(row.get(key))]
        return max(values) if values else None
    def mean(items,key):
        values=[abs(row[key]) for row in items if number(row.get(key))]
        return sum(values)/len(values) if values else None
    zeros=bool(commands) and all(abs(row.get('linear_x',math.inf))<=1e-6 and
                                abs(row.get('angular_z',math.inf))<=1e-6 for row in commands)
    transforms=[row for row in rows if row.get('stream')=='tf:odom_combined->base_footprint'
                and row['receipt_ros_seconds']<=finish]
    publishers=Counter(row.get('publisher_gid') for row in transforms)
    distance=turns=None
    if len(publishers)==1:
        previous=None;distance=turns=0.
        for row in sorted(transforms,key=lambda row:row.get('header_stamp_seconds',row['receipt_ros_seconds'])):
            xy=row.get('translation',[])[:2];q=row.get('rotation_xyzw',[])
            if len(xy)!=2 or len(q)!=4:continue
            x,y,z,w=q;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            if previous is not None:
                distance+=math.dist(xy,previous[:2])
                turns+=abs((yaw-previous[2]+math.pi)%(2*math.pi)-math.pi)
            previous=[*xy,yaw]
    stations=[];distinct=[];revisits=0;same_station=0
    for xy in state.get('route',[]):
        if len(xy)<2:continue
        xy=xy[:2]
        if stations and math.dist(xy,stations[-1])<.15:
            same_station+=1;continue
        if any(math.dist(xy,old)<.15 for old in distinct):revisits+=1
        else:distinct.append(xy)
        stations.append(xy)
    def elapsed(stamp):return stamp-start if stamp is not None else None
    return dict(measurement_only=True,started_stamp=start,
        first_raw_detection_exposure_seconds=elapsed(raw),first_raw_detection_received_seconds=elapsed(raw_received),
        first_explicitly_eligible_detection_exposure_seconds=elapsed(eligible),
        first_explicitly_eligible_detection_received_seconds=elapsed(eligible_received),
        first_accepted_decision_seconds=elapsed(accepted),
        first_report_event_received_seconds=elapsed(reported),report_timing_source=report_source,
        report_receipt_is_transport_upper_bound=True,
        full_task_seconds=mission.get('elapsed_seconds'),final_state=mission.get('state'),
        target_reported=mission.get('target_reported'),return_verified=mission.get('return_verified'),
        reported_target=mission.get('target'),reported_coordinate_source='Mission target; never simulator scene truth',
        source_stationary_check=mission.get('stationary_check'),
        final_half_second=dict(anchor='First received terminal Mission status, or recorded task elapsed time',
            command_samples=len(commands),all_recorded_commands_zero=zeros,
            command_observed_span_seconds=commands[-1]['receipt_ros_seconds']-commands[0]['receipt_ros_seconds'] if commands else None,
            maximum_abs_linear_command=maximum(commands,'linear_x'),maximum_abs_angular_command=maximum(commands,'angular_z'),
            odometry_samples=len(odom),mean_abs_linear_twist=mean(odom,'linear_x'),mean_abs_angular_twist=mean(odom,'angular_z')),
        sampled_odom_frame_path_metres=distance,sampled_absolute_turn_radians=turns,
        negative_linear_command_samples=sum(row.get('stream')=='cmd_vel' and row.get('linear_x',0.) < -1e-6 for row in rows),
        odometry_transform_publishers=dict(publishers),
        route_diagnostics=dict(planned_search_stations=stations,planned_station_revisits=revisits,
            same_station_heading_attempts=same_station,viewpoints_attempted=mission.get('viewpoints_attempted'),
            viewpoints_completed=mission.get('viewpoints_completed'),failed_viewpoints=mission.get('failed_viewpoints'),
            navigation_recovery_events=len(mission.get('navigation_recoveries') or [])),
        limitation='One real run, not the 3-scene median/+15% statistical acceptance; sampled passive logs may drop packets')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    state=json.loads((args.directory/'final-state.json').read_text())
    with gzip.open(args.directory/'raw-timing.jsonl.gz','rt') as stream:
        result=analyze((json.loads(line) for line in stream),state)
    result['analysis_source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    path=args.directory/'timing-analysis.json'
    path.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__':main()
