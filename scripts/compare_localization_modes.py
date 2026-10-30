#!/usr/bin/env python3
"""Compare explicit A/B off/on using real localhost simulation workflows.

Each case gets a committed source snapshot, its own physical sensor world,
fresh SLAM map, normal map review/start-pose handoff and a clean mission. Only
enable_ab_maneuver differs between modes. This runner never publishes a target
observation, navigation outcome, map pixel or mission state. Simulator truth
and read-only map->odom_combined TF are used only after observing the result.
"""
import argparse
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import re
import subprocess
import tarfile
import time
import traceback
import urllib.error
import urllib.parse
import urllib.request


TERMINAL = {'COMPLETE', 'FOUND_UNLOCALIZED', 'SEARCH_INCOMPLETE', 'NOT_FOUND', 'STOPPED', 'FAILED'}
POST_PATHS = {'stage', 'review_map', 'initial_pose', 'start', 'stop'}
EXPECTED_CAMERA = dict(camera_height=.22, camera_pitch=math.radians(10.), camera_yaw=0.,
                       camera_x=.05, camera_y=0., horizontal_fov=0.,
                       camera_min_range=.35, camera_max_range=2.)


def scenarios():
    bounds = [-.7, -.9, 1.3, .9]
    corners = [[bounds[0], bounds[1]], [bounds[2], bounds[1]],
               [bounds[2], bounds[3]], [bounds[0], bounds[3]]]
    room = dict(version=1, name='Small open comparison room', seed=None,
                bounds=bounds, walls=[[corners[i], corners[(i+1) % 4]] for i in range(4)],
                start=[0., 0., 0.])
    return {name: dict(copy.deepcopy(room), target=target)
            for name, target in [('normal', [.65, 0., 0.]), ('far_edge', [1., .35, 0.])]}


def local_base(url):
    parsed = urllib.parse.urlparse(url)
    if (parsed.scheme != 'http' or parsed.hostname not in ('127.0.0.1', 'localhost')
            or parsed.username or parsed.password or parsed.path not in ('', '/')
            or parsed.query or parsed.fragment):
        raise ValueError('Only an uncredentialed localhost HTTP simulation origin is allowed')
    return url.rstrip('/') + '/api/'


def require_simulated(state):
    if state.get('simulated') is not True or state.get('preview_backend') != 'main-ros':
        raise RuntimeError('Refusing to control a non-simulation console')


def sanitized(value):
    if isinstance(value, dict):
        return {k: sanitized(v) for k, v in value.items() if k != 'token'}
    if isinstance(value, list):
        return [sanitized(v) for v in value]
    return value


def save(path, value):
    Path(path).write_text(json.dumps(sanitized(value), indent=2, allow_nan=False) + '\n')


def sha_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def switch_ab(text, enabled):
    result, count = re.subn(r'(?m)^(\s*enable_ab_maneuver:\s*)(true|false)(\s*(?:#.*)?)$',
                           lambda m: m[1] + str(enabled).lower() + m[3], text)
    if count != 1:
        raise ValueError('Expected exactly one explicit enable_ab_maneuver configuration entry')
    return result


def map_point(transform, point):
    x, y, yaw = transform
    return [x + math.cos(yaw)*point[0] - math.sin(yaw)*point[1],
            y + math.sin(yaw)*point[0] + math.cos(yaw)*point[1]]


class Api:
    def __init__(self, url, output):
        self.base = local_base(url)
        self.output = Path(output)
        self.last_key = None
        self.first_report_wall = None
        self.first_report = None

    def get(self, path='state'):
        with urllib.request.urlopen(self.base + path, timeout=35) as response:
            return json.load(response)

    def state(self):
        state = self.get()
        require_simulated(state)
        now = time.time()
        with (self.output/'states.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(captured_wall=now, state=sanitized(state)), allow_nan=False)+'\n')
        mission = state.get('mission', {})
        if state.get('phase') == 'mission' and mission.get('target') and self.first_report is None:
            self.first_report_wall, self.first_report = now, sanitized(state)
            save(self.output/'first-report-state.json', dict(captured_wall=now, state=state))
        key = (state.get('phase'), mission.get('state'), mission.get('reason'))
        if key != self.last_key:
            print('STATE', *key, flush=True)
            self.last_key = key
        return state

    def post(self, path, body=None):
        if path not in POST_PATHS:
            raise ValueError('Unapproved control path: '+path)
        state = self.state()
        request = urllib.request.Request(self.base+path, data=json.dumps(body or {}).encode(),
            headers={'Content-Type': 'application/json', 'X-Project-Token': state['token'],
                     'X-Control-Generation': str(state['control_generation'])})
        try:
            with urllib.request.urlopen(request, timeout=65) as response:
                result = json.load(response)
        except urllib.error.HTTPError as error:
            raise RuntimeError(error.read().decode()) from error
        if result.get('success') is False:
            raise RuntimeError(str(result))
        with (self.output/'controls.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(captured_wall=time.time(), path=path, body=body or {},
                                         response=sanitized(result)))+'\n')
        return result

    def wait(self, predicate, timeout, label):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            state = self.state()
            if predicate(state):
                return state
            if state.get('mission', {}).get('state') in ('FAILED', 'STOPPED'):
                raise RuntimeError(label+': '+str(state.get('mission')))
            if state.get('workflow_fault') or state.get('stage', {}).get('fault'):
                raise RuntimeError(label+': stage/workflow fault')
            time.sleep(.25)
        raise TimeoutError(label+' exceeded runner deadline; outcome is censored, not an algorithm failure')


TF_OBSERVER = r'''
import json,math,time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener,TransformException
rclpy.init()
node=Node('localization_comparison_truth_observer')
buffer=Buffer();listener=TransformListener(buffer,node)
deadline=time.monotonic()+900.
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.05)
    try:
        tr=buffer.lookup_transform('map','odom_combined',Time())
        p,q=tr.transform.translation,tr.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        print(json.dumps(dict(captured_wall=time.time(),ros_time=node.get_clock().now().nanoseconds*1e-9,
            transform_stamp=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9,
            parent=tr.header.frame_id,child=tr.child_frame_id,transform=[p.x,p.y,yaw])),flush=True)
    except TransformException:
        pass
node.destroy_node();rclpy.shutdown()
'''


def docker(args, *command, **kwargs):
    return subprocess.run(['docker', '--context', args.context, *command],
                          check=True, text=True, **kwargs)


def parameter(args, container, node, name):
    result = docker(args, 'exec', container, 'bash', '-lc',
        'source /opt/ros/humble/setup.bash; ros2 param get '+node+' '+name,
        capture_output=True, timeout=15)
    return result.stdout.strip()


def prepare(args, root):
    source = Path(args.source).resolve()
    if not re.fullmatch('[0-9a-f]{40}', args.sha):
        raise ValueError('Provide the exact 40-character committed source SHA')
    actual = subprocess.check_output(['git', '-C', str(source), 'rev-parse', args.sha+'^{commit}'], text=True).strip()
    if actual != args.sha:
        raise ValueError('Source commit mismatch')
    archive = subprocess.check_output(['git', '-C', str(source), 'archive', '--format=tar', args.sha])
    modes = {}
    for enabled in (False, True):
        mode = 'ab_on' if enabled else 'ab_off'
        snapshot = root/'sources'/mode
        snapshot.mkdir(parents=True)
        with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
            tar.extractall(snapshot, filter='data')
        config = snapshot/'src/ee3033_project/config/project.yaml'
        original = config.read_text()
        config.write_text(switch_ab(original, enabled))
        modes[mode] = dict(enabled=enabled, source=str(snapshot), source_sha=args.sha,
            only_override='project_mission.enable_ab_maneuver',
            original_config_sha256=hashlib.sha256(original.encode()).hexdigest(),
            config_sha256=sha_file(config),
            files={str(p.relative_to(snapshot)): sha_file(p) for p in sorted(snapshot.rglob('*')) if p.is_file()})
    save(root/'source-manifest.json', modes)
    for name, scene in scenarios().items():
        for mode in modes:
            output = root/(name+'_'+mode)
            (output/'session'/'scenarios').mkdir(parents=True)
            save(output/'session'/'scenarios'/'current.json', scene)
            save(output/'case.json', dict(scene=scene, mode=mode, source=modes[mode],
                scene_sha256=sha_file(output/'session'/'scenarios'/'current.json'),
                expected_camera=EXPECTED_CAMERA, mapping_resolution=.05,
                evidence='full two-stage real algorithms with simulated physical sensors; not hardware'))
    return modes


def observe_tf(args, container, output):
    stream = (output/'map-to-odom-tf.jsonl').open('w')
    error = (output/'tf-observer.stderr').open('w')
    proc = subprocess.Popen(['docker', '--context', args.context, 'exec', '-i', container,
        'bash', '-lc', 'source /opt/ros/humble/setup.bash; python3 -u -'],
        stdin=subprocess.PIPE, stdout=stream, stderr=error, text=True)
    proc.stdin.write(TF_OBSERVER)
    proc.stdin.close()
    return proc, stream, error


def truth_metrics(output, scene, api, result):
    target = (api.first_report or {}).get('mission', {}).get('target') or result.get('mission', {}).get('target')
    metrics = dict(reference='Simulator world equals odom_combined; target transformed by observed map->odom_combined TF',
                   target_world=scene['target'], reported_target=target)
    samples = []
    path = output/'map-to-odom-tf.jsonl'
    for line in path.read_text().splitlines() if path.exists() else []:
        try:
            samples.append(json.loads(line))
        except ValueError:
            continue
    if target and samples and api.first_report_wall is not None:
        tf = min(samples, key=lambda row: abs(row['captured_wall']-api.first_report_wall))
        delta = abs(tf['captured_wall']-api.first_report_wall)
        age = tf['ros_time']-tf['transform_stamp']
        metrics.update(transform_sample=tf, sampling_delta_seconds=delta, transform_age_seconds=age)
        if delta <= .5 and -.5 <= age <= 1.:
            point = map_point(tf['transform'], scene['target'])
            metrics.update(map_truth=point,
                target_error_metres=math.hypot(target['x']-point[0], target['y']-point[1]))
        else:
            metrics.update(target_error_metres=None,
                truth_unavailable_reason='TF or first-report sampling was too stale for an accuracy claim')
    else:
        metrics['target_error_metres'] = None
        metrics['truth_unavailable_reason'] = 'No report or no read-only map/odom TF sample'
    return metrics


def run_case(args, root, name, scene, mode, source, index):
    output = root/(name+'_'+mode)
    run_id = hashlib.sha256(str(root).encode()).hexdigest()[:6]
    container = 'ee3033-loc-'+args.sha[:8]+'-'+run_id+'-'+name.replace('_', '-')+'-'+mode.replace('_', '-')
    url = 'http://127.0.0.1:'+str(args.port)
    api = Api(url, output)
    observer = None
    created = owned_endpoint = False
    started = time.monotonic()
    summary = dict(case=name, mode=mode, container=container, source_sha=args.sha,
                   full_two_stage=True, automatic_retries=0, started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
    launch = '''set -eo pipefail
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/comparison-colcon-logs build --symlink-install --base-paths /work/src --packages-up-to ee3033_project --build-base /tmp/comparison-build --install-base /tmp/comparison-install
source /tmp/comparison-install/setup.bash
echo '9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef  /tmp/ee3033_sim/assets/yolo26n.pt' | sha256sum --check
exec python3 scripts/run_algorithm_preview.py --model /tmp/ee3033_sim/assets/yolo26n.pt --texture /tmp/ee3033_sim/assets/cup.png --public-port '''+str(args.port)+''' --output-directory /tmp/ee3033_comparison_session'''
    result = {}
    try:
        docker(args, 'run', '-d', '--name', container, '-p', '127.0.0.1:'+str(args.port)+':8790',
            '--mount', 'type=bind,source='+source['source']+',target=/work,readonly',
            '--mount', 'type=bind,source='+str(output/'session')+',target=/tmp/ee3033_comparison_session',
            '-w', '/work', '-e', 'PREVIEW_MAIN_COMMIT='+args.sha+'-'+mode,
            '-e', 'ROS_DOMAIN_ID='+str(122+index), '-e', 'ROS_LOCALHOST_ONLY=1',
            args.image, 'bash', '-lc', launch, capture_output=True)
        created = True
        deadline = time.monotonic()+args.startup_timeout
        while True:
            try:
                state = api.state()
                break
            except (urllib.error.URLError, ConnectionError):
                if time.monotonic() >= deadline:
                    raise TimeoutError('Simulation startup deadline exceeded')
                time.sleep(.5)
        version = api.get('result').get('software_version')
        if version != args.sha+'-'+mode:
            raise RuntimeError('Endpoint software version does not belong to this test container')
        owned_endpoint = True
        if state.get('assessment', {}).get('started') is not None:
            raise RuntimeError('Fresh comparison unexpectedly contains an existing trial')
        if not math.isclose(state.get('mapping_resolution', -1), .05):
            raise RuntimeError('Unexpected mapping resolution')
        for key, value in EXPECTED_CAMERA.items():
            if not math.isclose(state['simulation_camera'][key], value, abs_tol=1e-9):
                raise RuntimeError('Camera changed: '+key)
        if state['simulation_scenario']['target'] != scene['target']:
            raise RuntimeError('Physical target layout mismatch')
        observer = observe_tf(args, container, output)
        api.post('stage', {'phase': 'mapping'})
        api.wait(lambda s: s.get('mission', {}).get('ready') is True, 60, 'Mapping readiness')
        summary['mapping_resolution_readback'] = parameter(args, container, '/slam_toolbox', 'resolution')
        if summary['mapping_resolution_readback'] != 'Double value is: 0.05':
            raise RuntimeError('Actual SLAM parameter is not 0.05 m')
        api.post('start')
        mapped = api.wait(lambda s: s.get('mission', {}).get('state') in ('MAPPED', 'MAPPED_PARTIAL'),
                          args.mapping_timeout, 'Mapping')
        summary['mapping'] = sanitized(mapped['mission'])
        entry = None
        deadline = time.monotonic()+30
        while entry is None and time.monotonic() < deadline:
            state = api.state()
            entry = next((m for m in api.get('maps')['maps']
                          if m['path'] == state.get('saved_map') == state.get('mission', {}).get('saved_map')), None)
            if entry is None:
                time.sleep(.25)
        if entry is None or not state.get('pose'):
            raise RuntimeError('Saved SLAM map or mapping pose handoff unavailable')
        pose = state['pose']
        save(output/'mapping-end-state.json', state)
        save(output/'saved-map-entry.json', entry)
        api.post('review_map', {'map_id': entry['id']})
        api.post('stage', dict(phase='mission', map_id=entry['id'], target_class='cup',
            localization_method='hybrid', return_tolerance=.2, preferred_target_sigma=.1))
        api.wait(lambda s: s.get('phase') == 'mission' and s.get('mission', {}).get('state') == 'WAITING'
                 and s.get('health', {}).get('Mission', {}).get('ok'), 60, 'Mission stack')
        api.post('initial_pose', dict(zip(('x', 'y', 'yaw'), pose)))
        api.wait(lambda s: s.get('mission', {}).get('ready') is True, 60, 'Mission readiness')
        summary['ab_readback'] = parameter(args, container, '/project_mission', 'enable_ab_maneuver')
        if summary['ab_readback'] != 'Boolean value is: '+str(source['enabled']):
            raise RuntimeError('A/B runtime parameter did not match the intended mode')
        mission_start = time.monotonic()
        api.post('start')
        final = api.wait(lambda s: s.get('mission', {}).get('state') in TERMINAL,
                         args.mission_timeout, 'Mission')
        result = api.get('result')
        save(output/'result.json', result)
        save(output/'terminal-state.json', final)
        summary.update(outcome='algorithm_terminal', state=final['mission'].get('state'),
                       mission_wall_seconds=time.monotonic()-mission_start,
                       mission=sanitized(final['mission']))
    except Exception as exc:
        summary.update(outcome='runner_error', exception_type=type(exc).__name__, error=str(exc),
                       traceback=traceback.format_exc())
        print('CASE ERROR', name, mode, type(exc).__name__, str(exc), flush=True)
    finally:
        if owned_endpoint:
            try:
                api.post('stop')
                save(output/'state-after-stop.json', api.state())
            except Exception as exc:
                summary['stop_error'] = str(exc)
        if created:
            try:
                docker(args, 'stop', '-t', '25', container, capture_output=True, timeout=35)
            except Exception as exc:
                summary['container_stop_error'] = str(exc)
        if observer:
            try:
                observer[0].wait(timeout=5)
            except subprocess.TimeoutExpired:
                observer[0].terminate()
            observer[1].close()
            observer[2].close()
        with (output/'container.log').open('w') as stream:
            try:
                if created:
                    docker(args, 'logs', container, stdout=stream, stderr=subprocess.STDOUT, timeout=15)
            except Exception as exc:
                summary['log_error'] = str(exc)
        summary['total_wall_seconds'] = time.monotonic()-started
        summary['truth'] = truth_metrics(output, scene, api, result)
        summary['artifact_sha256'] = {str(p.relative_to(output)): sha_file(p)
                                    for p in sorted(output.rglob('*')) if p.is_file()}
        save(output/'summary.json', summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument('--sha', required=True, help='Exact committed source version to compare')
    parser.add_argument('--output', required=True, type=Path, help='New output directory; never overwritten')
    parser.add_argument('--context', default='colima')
    parser.add_argument('--image', default='ee3033-current-sim:temporary')
    parser.add_argument('--port', type=int, default=8794)
    parser.add_argument('--startup-timeout', type=float, default=90.)
    parser.add_argument('--mapping-timeout', type=float, default=120.)
    parser.add_argument('--mission-timeout', type=float, default=180.)
    parser.add_argument('--run', action='store_true', help='Actually run all four two-stage workflows')
    args = parser.parse_args()
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    modes = prepare(args, root)
    save(root/'protocol.json', dict(source_sha=args.sha, scenarios=scenarios(), modes=['ab_off', 'ab_on'],
        trials=4, each_trial='independent full SLAM + mission', algorithm_thresholds_unchanged=True,
        run_requested=args.run, mapping_runner_deadline=args.mapping_timeout,
        mission_runner_deadline=args.mission_timeout,
        failure_policy='Save every attempt; no automatic retry; runner deadlines censor the outcome'))
    if not args.run:
        print('Prepared four cases; no containers or workflows started:', root)
        return
    summaries = []
    for name, scene in scenarios().items():
        for mode, source in modes.items():
            summaries.append(run_case(args, root, name, scene, mode, source, len(summaries)))
            save(root/'comparison.json', summaries)
    print(json.dumps([dict(case=s['case'], mode=s['mode'], outcome=s['outcome'],
        state=s.get('state'), error_metres=s['truth'].get('target_error_metres'),
        mission_seconds=s.get('mission_wall_seconds')) for s in summaries], indent=2), flush=True)


if __name__ == '__main__':
    main()
