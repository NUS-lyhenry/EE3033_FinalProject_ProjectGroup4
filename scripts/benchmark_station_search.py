#!/usr/bin/env python3
"""Reproducible ideal-frame planner comparison, never physical test evidence."""
import importlib.util
import json
import math
from pathlib import Path
import time
import numpy as np
from ee3033_project.coverage import CoveragePlanner
from ee3033_project.core import angle_difference

BASELINE = Path(__file__).resolve().parents[1]/'validation/historical/search_baseline_coverage.py'


def baseline_class():
    import sys
    name = 'ee3033_project.coverage_station_baseline'
    spec = importlib.util.spec_from_file_location(name, BASELINE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module.CoveragePlanner


def run(grid, cls, station):
    p = cls(grid, .1, (0., 0., 0.), clearance=.15, spacing=.65,
            headings=6, view_range=.95, fov=1.35)
    p.prepare(.75, .75)
    pose = (.75, .75, 0.); seen = set(); attempted = set()
    elapsed = distance_total = turns = dwell = 0.
    station_key = None; views = 0; previous = []; visited = set(); returns = 0
    latencies = []; sequence = []
    for _ in range(120):
        started = time.monotonic()
        if station:
            index = p.select_station_view(*pose[:2], seen, attempted, yaw=pose[2],
                deadline=started+1.5, station_views=views, station_seconds=dwell,
                previous_stations=previous)
        else:
            index = p.lookahead(*pose[:2], seen, attempted, yaw=pose[2],
                deadline=started+1.5, strategy='adaptive')
        latencies.append(time.monotonic()-started)
        if index is None:
            break
        v = p.options[index]; key = p.world_to_cell(v.x, v.y)
        local = math.hypot(v.x-pose[0], v.y-pose[1]) < .15
        if key != station_key:
            if station_key is not None:
                previous.append(station_key)
            returns += key in visited
            visited.add(key); station_key = key; views = 0; dwell = 0.
        field, _ = p.travel_field(*pose[:2])
        distance = 0. if local else float(field[key])
        heading = pose[2] if local else math.atan2(v.y-pose[1], v.x-pose[0])
        turn = abs(angle_difference(heading, pose[2]))+abs(angle_difference(v.yaw, heading))
        work = turn/.4+1.8
        elapsed += distance/.12+work; dwell += work; views += 1
        distance_total += distance; turns += turn
        seen.update(v.visible); attempted.add(index); pose = (v.x, v.y, v.yaw)
        sequence.append([v.x, v.y, v.yaw])
        if len(seen & p.coverable) == len(p.coverable):
            break
    return dict(route_seconds=elapsed, path_metres=distance_total, turn_radians=turns,
                observations=len(attempted), cross_station_returns=returns,
                observed_cells=len(seen), target_cells=len(p.target_cells),
                observed_fraction=len(seen)/len(p.target_cells),
                largest_missing_patch=p.completeness(seen)['largest_missing_patch'],
                planning_p95_seconds=float(np.percentile(latencies,95)), sequence=sequence)


def main():
    result = {'evidence':'Planner-only, identical maps and analytic camera footprint (0.95 m, 1.35 rad); '
              'ideal fresh frames, 0.12 m/s, 0.40 rad/s. No YOLO/Nav2/hardware claim.',
              'baseline_source':str(BASELINE), 'scenarios':{}}
    for name in ('open', 'divider', 'alcove'):
        grid = np.zeros((25,35), dtype=int); grid[[0,-1],:] = 100; grid[:,[0,-1]] = 100
        if name != 'open':grid[8:,17:19] = 100
        if name == 'alcove':grid[8:10,22:31] = 100; grid[8:19,29:31] = 100
        old = run(grid, baseline_class(), False); new = run(grid, CoveragePlanner, True)
        result['scenarios'][name] = dict(baseline=old, station=new,
            acceptance=dict(coverage_not_lower=new['observed_cells']>=old['observed_cells'],
                time_within_110_percent=new['route_seconds']<=1.1*old['route_seconds'],
                returns_not_increased=new['cross_station_returns']<=old['cross_station_returns']))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
