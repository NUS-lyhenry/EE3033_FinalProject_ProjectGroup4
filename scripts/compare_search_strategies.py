#!/usr/bin/env python3
"""Compare search forecasts or record actual isolated ROS simulation timing.

This tool never starts a mission, changes parameters or publishes robot commands.
Start the recorder before starting the selected simulation mission. Matrix lists
the requested paired cases; it does not claim those cases have been executed.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time

SCENES = ('open', 'divider', 'alcove')
TARGETS = ('near', 'far', 'occluded', 'absent')
STRATEGIES = ('greedy', 'adaptive')
TERMINAL = {'COMPLETE', 'FAILED', 'STOPPED', 'NOT_FOUND', 'SEARCH_INCOMPLETE', 'FOUND_UNLOCALIZED'}


def matrix():
    return [dict(scene=scene, target=target, strategy=strategy, status='not_run')
            for scene in SCENES for target in TARGETS for strategy in STRATEGIES]


def summarize_events(events):
    """Use exposure timestamps for detections and ROS receive time for reports.

    A target lock is distinct from the first YOLO detection. Missing events stay
    null; an analytic field-of-view prediction never becomes a detection.
    """
    initial = next((e['message'] for e in events if e['topic'] == '/project/mission_status'
                    and e['message'].get('phase') == 'mission' and e['message'].get('started_stamp')), {})
    started, label = initial.get('started_stamp'), initial.get('target_class')
    first_detection = first_lock = first_report = None
    last = {}
    for event in events:
        data, topic, received = event['message'], event['topic'], event['stamp']
        if topic == '/project/mission_status' and data.get('phase') == 'mission':
            value = data.get('started_stamp', 0.)
            if started and value and value != started:
                break  # A second mission belongs to a separate comparison run.
            if not value:
                continue
            last = data
            if started and data.get('target_detected') and first_lock is None:
                first_lock = data.get('detection_stamp') or received
            if started and data.get('target_reported') and first_report is None:
                first_report = received
        elif topic == '/project/target_detections' and started:
            for detection in data.get('detections', ()):
                stamp = detection.get('stamp', 0.)
                if (detection.get('label') == label and detection.get('search_eligible', True)
                        and started <= stamp <= received and received-stamp <= 1.5):
                    first_detection = stamp if first_detection is None else min(first_detection, stamp)
        elif topic == '/project/mission_result' and started and data.get('started_stamp') == started:
            if data.get('event') == 'target_reported' and first_report is None:
                first_report = received
    def elapsed(stamp):
        return max(0., stamp-started) if started is not None and stamp is not None else None
    return dict(started_stamp=started, first_target_detection_seconds=elapsed(first_detection),
                first_target_lock_seconds=elapsed(first_lock), first_target_report_seconds=elapsed(first_report),
                task_seconds=last.get('elapsed_seconds') if last.get('state') in TERMINAL else None,
                task_complete=bool(last.get('state') == 'COMPLETE' and last.get('return_verified')
                                   and last.get('target_reported') and last.get('target')),
                task_timing='Mission Start through localization/report, return and verified stop; mapping excluded',
                final_state=last.get('state'), return_verified=last.get('return_verified'),
                observed_fraction=last.get('observed_fraction'),
                search_stop_reason=last.get('search_stop_reason'),
                timing='Detection: source exposure timestamp; lock: source detection timestamp; report: ROS receive time')


def record_native(args):
    if os.environ.get('ROS_DOMAIN_ID') in (None, '0', '6') or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise ValueError('Use an explicitly isolated simulation DDS domain and ROS_LOCALHOST_ONLY=1')
    from capture_navigation_case import simulation_snapshot
    snapshot = simulation_snapshot(args.url)  # Rejects physical/non-native endpoints.
    if snapshot['state'].get('mission', {}).get('started_stamp'):
        raise ValueError('Start this recorder before the simulation mission')
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String
    from ee3033_project.ros_utils import LATCHED
    directory = Path(args.output)
    directory.mkdir(parents=True, exist_ok=True)
    (directory/'snapshot.json').write_text(json.dumps(snapshot, indent=2)+'\n')
    events = []
    rclpy.init()
    node = Node('search_comparison_recorder')
    try:
        client = node.create_client(GetParameters, '/project_mission/get_parameters')
        if not client.wait_for_service(timeout_sec=5.):
            raise ValueError('Mission parameter service is unavailable; load the simulation mission first')
        future = client.call_async(GetParameters.Request(names=['search_strategy']))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done() or not future.result():
            raise ValueError('Could not verify the effective search strategy')
        effective_strategy = future.result().values[0].string_value
        if effective_strategy != args.strategy:
            raise ValueError(f'Expected {args.strategy}, but the mission uses {effective_strategy}')
    except Exception:
        node.destroy_node()
        rclpy.shutdown()
        raise
    stream = (directory/'events.jsonl').open('w')
    def capture(topic, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        event = dict(topic=topic, stamp=node.get_clock().now().nanoseconds*1e-9, message=data)
        events.append(event)
        stream.write(json.dumps(event, separators=(',', ':'))+'\n')
        stream.flush()
    subscriptions = [node.create_subscription(String, topic, lambda m, t=topic: capture(t, m), qos)
                     for topic, qos in (('/project/mission_status', LATCHED),
                                        ('/project/target_detections', 10), ('/project/mission_result', LATCHED))]
    deadline = time.monotonic()+args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            latest = next((e['message'] for e in reversed(events) if e['topic'] == '/project/mission_status'), {})
            if latest.get('started_stamp') and latest.get('state') in TERMINAL:
                break
    finally:
        stream.close()
        node.destroy_node()
        rclpy.shutdown()
    result = dict(evidence='Recorded native ROS simulation; real perception/mission event timing, no physical validation',
                  declared_scene=args.scene, declared_target=args.target, strategy=effective_strategy,
                  scenario=snapshot.get('scenario'), metrics=summarize_events(events))
    (directory/'metrics.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def planner_forecast(scene, strategy):
    import numpy as np
    from ee3033_project.coverage import CoveragePlanner
    from ee3033_project.core import angle_difference
    grid = np.zeros((25, 35), dtype=int)
    grid[[0, -1], :] = 100
    grid[:, [0, -1]] = 100
    if scene != 'open':
        grid[8:, 17:19] = 100
    if scene == 'alcove':
        grid[8:10, 22:31] = 100
        grid[8:19, 29:31] = 100
    p = CoveragePlanner(grid, .1, (0., 0., 0.), clearance=.15,
                        spacing=.65, headings=6, view_range=.95, fov=1.35)
    p.prepare(.75, .75)
    pose = (.75, .75, 0.)
    seen, attempted, previous, first_seen, latencies = set(), set(), [], {}, []
    elapsed = distance_total = turns = dwell = 0.
    station = None
    attempts = 0
    for _ in range(120):
        begin = time.monotonic()
        excluded = set(attempted)
        for i, v in enumerate(p.options):
            if math.dist(pose[:2], (v.x, v.y)) < .15:
                turn = abs(angle_difference(v.yaw, pose[2]))/.4
                if attempts >= 6 or dwell+turn+1.8 > 25.:
                    excluded.add(i)
        if strategy == 'greedy':
            ranked = p.rank_viewpoints(*pose[:2], seen, excluded, yaw=pose[2],
                                       observation_seconds=.8, previous_stations=previous)
            choice = ranked[0][1] if ranked else None
        else:
            choice = p.select_station_view(*pose[:2], seen, excluded, yaw=pose[2],
                deadline=begin+1.5, station_views=attempts, station_seconds=dwell,
                previous_stations=previous, strategy=strategy)
        latencies.append(time.monotonic()-begin)
        if choice is None:
            break
        v = p.options[choice]
        key = p.station_key(v.x, v.y)
        local = math.dist(pose[:2], (v.x, v.y)) < .15
        if not local:
            if station is not None:
                previous.append(station)
            attempts = 0
            dwell = 0.
        field, _ = p.travel_field(*pose[:2])
        distance = 0. if local else float(field[key])
        heading = pose[2] if local else math.atan2(v.y-pose[1], v.x-pose[0])
        turn = abs(angle_difference(heading, pose[2]))+abs(angle_difference(v.yaw, heading))
        work = turn/.4+1.8
        elapsed += distance/.12+work
        dwell += work
        attempts += 1
        station = key
        distance_total += distance
        turns += turn
        for cell in v.visible-seen:
            first_seen[cell] = elapsed
        seen.update(v.visible)
        attempted.add(choice)
        pose = (v.x, v.y, v.yaw)
        if p.completeness(seen)['complete']:
            break
    targets = {'near': (.85, .75), 'far': (2.65, 2.05), 'occluded': (2.65, 1.45)}
    probes = {name: min(p.target_cells, key=lambda cell: math.dist(p.cell_to_world(*cell), xy))
              for name, xy in targets.items()}
    visibility = {name: first_seen.get(cell) for name, cell in probes.items()}
    visibility['absent'] = None
    return dict(route_seconds=elapsed, path_metres=distance_total, turn_radians=turns,
                observations=len(attempted), observed_fraction=len(seen)/max(1, len(p.target_cells)),
                mean_analytic_first_visibility_seconds=float(np.mean(list(first_seen.values()))) if first_seen else None,
                analytic_coverage_at_60_seconds=sum(stamp <= 60. for stamp in first_seen.values())/max(1,len(p.target_cells)),
                planning_p95_seconds=float(np.percentile(latencies, 95)),
                analytic_probe_positions={name: p.cell_to_world(*cell) for name, cell in probes.items()},
                analytic_first_visibility_seconds=visibility,
                actual_detection_seconds=None, actual_report_seconds=None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('matrix', help='List the 24 paired case specifications without running them')
    sub.add_parser('planner', help='Analytic forecast only; never launches ROS or perception')
    record = sub.add_parser('record', help='Read-only ROS event recording for one preconfigured native simulation')
    record.add_argument('--url', default='http://127.0.0.1:8792')
    record.add_argument('--scene', choices=SCENES, required=True)
    record.add_argument('--target', choices=TARGETS, required=True)
    record.add_argument('--strategy', choices=STRATEGIES, required=True)
    record.add_argument('--output', required=True)
    record.add_argument('--duration', type=float, default=1000.)
    args = parser.parse_args()
    if args.command == 'matrix':
        result = matrix()
    elif args.command == 'planner':
        result = dict(evidence='Analytic planner-only, ideal endpoint exposures at 0.12 m/s and 0.4 rad/s; no detection/report timing',
                      target_note='Occluded is only a probe location in open space; no occlusion is implied for the open scene',
                      scenes={scene: {s: planner_forecast(scene, s) for s in STRATEGIES} for scene in SCENES})
    else:
        if args.duration <= 0 or not math.isfinite(args.duration):
            parser.error('duration must be finite and positive')
        result = record_native(args)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
