#!/usr/bin/env python3
"""Replay recorded diagnostics up to the first new watchdog decision.

This executes production Mission methods without ROS. It does not reconstruct
unrecorded headings, choose an alternate location or simulate physical motion.
"""
import argparse
import ast
from datetime import datetime, timezone, timedelta
import hashlib
import gzip
import json
import math
from pathlib import Path
from types import SimpleNamespace


class AlternateDecision(Exception):
    pass


def replay(bundle):
    source = Path(__file__).resolve().parents[1]/'src/ee3033_project/ee3033_project/mission.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Mission')
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                and n.name in ('localization_watchdog', 'observe_localization_progress')]
    clock = SimpleNamespace(value=0.)
    scope = dict(math=math, time=SimpleNamespace(monotonic=lambda: clock.value))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), scope)
    results = []
    for name in ('mission_raipogeu', 'mission_1ztiw7lh', 'mission_gm2pkwy8'):
        path = f'mission_runs/{name}/localization_frames.jsonl'
        events = [json.loads(line) for line in bundle['files'][path].splitlines() if line.strip()]
        events = [e for e in events if e.get('mission_state') in ('LOCALIZING', 'ACQUIRING')]
        owner = scope['Mission']()
        start = events[0]['published_stamp']
        owner.target_detected = True
        owner.target = None
        owner.bottom_ranging = True
        owner.p = dict(reacquisition_timeout=10., localization_stall_timeout=10.)
        owner.last_usable_bearing = owner.last_localization_progress = start
        owner.batch_highwater = 0
        owner.stall_alternate_attempted = False
        owner.pose = lambda: (0., 0., 0.)
        owner.planner = owner.plan_cancel = None
        owner.navigation_map = SimpleNamespace(allowed=lambda *args: True)
        owner.observation_position_allowed = owner.navigation_map.allowed
        decision = {}
        owner.request_research = lambda reason: decision.update(action='resume_global_search', reason=reason)
        def alternate(*args, **kwargs):
            raise AlternateDecision()
        owner.acquisition = SimpleNamespace(alternate_observation_goal=alternate)
        last_stamp = None
        for event in events:
            clock.value = event['published_stamp']
            owner.state = event['mission_state']
            selected = event.get('selected') or {}
            associated = event.get('association') or {}
            stamp = selected.get('stamp')
            if (associated.get('matched') and associated.get('frames', 0) >= 2
                    and selected.get('bearing_valid', True) and 'bearing' in selected
                    and stamp != last_stamp and 0 <= clock.value-stamp <= 1.5):
                owner.last_usable_bearing = clock.value
                last_stamp = stamp
            owner.observe_localization_progress(event)
            try:
                owner.localization_watchdog(clock.value)
            except AlternateDecision:
                decision.update(action='try_safe_alternate_site', reason='localization_stalled')
            if decision:
                break
        results.append(dict(task=name, source=path, first_decision=decision or None,
            seconds_after_first_localization_frame=round(clock.value-start, 3),
            decision_time_sgt=datetime.fromtimestamp(clock.value, timezone(timedelta(hours=8))).isoformat(),
            batch_highwater=owner.batch_highwater,
            seconds_without_usable_bearing=round(clock.value-owner.last_usable_bearing, 3),
            seconds_without_sample_progress=round(clock.value-owner.last_localization_progress, 3)))
    assert [r['first_decision']['action'] for r in results] == [
        'resume_global_search', 'resume_global_search', 'try_safe_alternate_site'], results
    return dict(evidence='Recorded diagnostic decision replay only; no alternate-site feasibility or motion replay.',
        source_root=bundle.get('root'), production_mission_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        note='Initial watchdog time uses the first recorded localization diagnostic; callback/timer quantization is retained.',
        tasks=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evidence', type=Path)
    args = parser.parse_args()
    data = gzip.decompress(args.evidence.read_bytes()) if args.evidence.suffix == '.gz' else args.evidence.read_bytes()
    print(json.dumps(replay(json.loads(data)), indent=2))


if __name__ == '__main__':
    main()
