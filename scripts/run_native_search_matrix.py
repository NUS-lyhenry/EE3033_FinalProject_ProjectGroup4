#!/usr/bin/env python3
"""Execute baseline/new synthetic native ROS cases serially on domain 156.

The child fixture runs production Mission and Filter with synthetic detections
and accelerated collision-clear mock actions. This is not YOLO or Nav2 testing.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-root', type=Path, default=Path('/baseline'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=70.)
    parser.add_argument('--time-scale', type=float, default=12.)
    parser.add_argument('--scene', choices=('open','divider','alcove'))
    parser.add_argument('--target', choices=('near','far','occluded','absent'))
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '156' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Use only isolated domain 156 with ROS_LOCALHOST_ONLY=1')
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    sources = {}
    for version, source in (('baseline',args.baseline_root), ('new',root)):
        sources[version] = {name:hashlib.sha256((source/'src/ee3033_project/ee3033_project'/name).read_bytes()).hexdigest()
                            for name in ('mission.py','coverage.py','target_filter.py')}
    report = dict(evidence='Executed native ROS synthetic fixture; mock actions accelerated equally; no YOLO/Nav2-controller/SLAM/hardware validation',
                  source_sha256=sources, time_scale=args.time_scale, cases=results)
    scenes = (args.scene,) if args.scene else ('open','divider','alcove')
    targets = (args.target,) if args.target else ('near','far','occluded','absent')
    for scene in scenes:
        for target in targets:
            for version, strategy, source in (('baseline','adaptive',args.baseline_root), ('new','greedy',root)):
                case = args.output/f'{version}-{scene}-{target}'
                case.mkdir(parents=True, exist_ok=True)
                env = dict(os.environ)
                env['PYTHONPATH'] = str(source/'src/ee3033_project')+':'+str(root/'tests')+':'+env.get('PYTHONPATH','')
                command = [sys.executable,str(root/'tests/native_paired_search.py'),
                    '--scene',scene,'--target',target,'--strategy',strategy,'--version',version,
                    '--output',str(case),'--timeout',str(args.timeout),'--time-scale',str(args.time_scale)]
                with (case/'native.log').open('w') as log:
                    try:
                        completed = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                                   timeout=args.timeout+30.)
                        code = completed.returncode
                    except subprocess.TimeoutExpired:
                        code = 'process_timeout'
                path = case/'comparison.json'
                data = json.loads(path.read_text()) if path.exists() else {}
                result = dict(version=version, scene=scene, target=target,
                    exit_code=code, accepted=bool(data.get('accepted')), final_state=data.get('result',{}).get('state'),
                    task_seconds=data.get('task_seconds'), first_detection_seconds=data.get('first_detection_seconds'),
                    first_report_seconds=data.get('first_report_seconds'), path_metres=data.get('path_metres'),
                    turn_radians=data.get('turn_radians'), evidence=str(path))
                results.append(result)
                report['all_completed_cases_accepted'] = all(r['accepted'] for r in results)
                (args.output/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
                print(json.dumps(result),flush=True)
    return 0 if report['all_completed_cases_accepted'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
