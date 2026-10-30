#!/usr/bin/env python3
"""Deterministic planner-only comparison; not a YOLO, Nav2 or hardware result."""
import json
import math
import time
import numpy as np
from ee3033_project.coverage import CoveragePlanner
from ee3033_project.core import angle_difference


def run(grid,strategy):
    planner=CoveragePlanner(grid,.1,(0,0,0),spacing=.65,headings=6,view_range=1.5)
    planner.prepare(1.,1.)
    pose=(1.,1.,0.);seen=set();attempted=set();elapsed=distance_total=turn_total=0.
    first_seen={};latencies=[]
    for _ in range(120):
        begin=time.monotonic()
        if strategy=='greedy':
            index=planner.next_viewpoint(*pose[:2],seen,attempted,yaw=pose[2],observation_seconds=.8)
        else:
            index=planner.lookahead(*pose[:2],seen,attempted,yaw=pose[2],deadline=begin+1.5, strategy=strategy)
        latencies.append(time.monotonic()-begin)
        if index is None:break
        v=planner.options[index];field,_=planner.travel_field(*pose[:2]);distance=float(field[planner.world_to_cell(v.x,v.y)])
        heading=math.atan2(v.y-pose[1],v.x-pose[0]) if distance>.05 else pose[2]
        turn=abs(angle_difference(heading,pose[2]))+abs(angle_difference(v.yaw,heading))
        elapsed+=distance/.12+turn/.4+1.8;distance_total+=distance;turn_total+=turn
        for cell in v.visible-seen:first_seen[cell]=elapsed
        seen.update(v.visible);attempted.add(index);pose=(v.x,v.y,v.yaw)
    return dict(mean_first_visibility_seconds=float(np.mean(list(first_seen.values()))),
                route_seconds=elapsed,path_metres=distance_total,turn_radians=turn_total,
                stops=len(attempted),observed_cells=len(seen),target_cells=len(planner.target_cells),
                largest_missing_patch=planner.completeness(seen)['largest_missing_patch'],
                planning_p95_seconds=float(np.percentile(latencies,95)))


def main():
    output={'evidence':'planner-only, ideal observations, same 0.12 m/s and 0.40 rad/s; no inference or localization timing','scenarios':{}}
    for name in ('open','divider','alcove'):
        grid=np.zeros((45,65),dtype=int);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
        if name!='open':grid[10:,30:32]=100
        if name=='alcove':grid[10:12,40:58]=100;grid[10:30,56:58]=100
        output['scenarios'][name]={strategy:run(grid,strategy) for strategy in ('greedy','lookahead','adaptive')}
    print(json.dumps(output,indent=2))

if __name__=='__main__':main()
