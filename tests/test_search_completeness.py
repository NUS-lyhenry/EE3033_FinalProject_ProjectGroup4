import math
import time
import numpy as np
from ee3033_project.coverage import CoveragePlanner


def divided_room(blocked=False):
    grid=np.zeros((45,65),dtype=int)
    grid[[0,-1],:]=100;grid[:,[0,-1]]=100
    grid[10:,30:32]=100
    if blocked:grid[:10,30:32]=100
    return grid


def test_reachable_wall_back_is_an_observation_target_not_silently_excluded():
    planner=CoveragePlanner(divided_room(),.1,(0,0,0),clearance=.14,spacing=.65,headings=6,view_range=1.5)
    planner.prepare(1.,1.)
    behind={(r,c) for r,c in planner.target_cells if c>34 and r>20}
    assert behind and behind & planner.coverable
    assert any(v.x>3.4 and v.visible & behind for v in planner.options)
    seen=set();pose=(1.,1.,0.);attempted=set()
    for _ in range(60):
        index=planner.lookahead(*pose[:2],seen,attempted,yaw=pose[2],deadline=time.monotonic()+1.)
        if index is None:break
        v=planner.options[index];seen.update(v.visible);attempted.add(index);pose=(v.x,v.y,v.yaw)
        if behind <= seen:break
    assert behind <= seen


def test_incomplete_candidates_and_failed_views_cannot_claim_completion():
    planner=CoveragePlanner(divided_room(),.1,(0,0,0),spacing=1.,view_range=1.3)
    planner.prepare(1.,1.)
    first=planner.options[0].visible
    assert not planner.completeness(first)['complete']
    assert planner.completeness(first)['missing_cells']>0
    # A caller cannot turn failures into evidence merely by exhausting choices.
    assert planner.lookahead(1.,1.,first,excluded=range(len(planner.options))) is None
    assert not planner.completeness(first)['complete']


def test_partial_map_cannot_report_not_found_even_if_candidates_are_exhausted():
    grid=divided_room();grid[15:18,1:4]=-1
    planner=CoveragePlanner(grid,.1,(0,0,0));planner.prepare(1.,1.)
    assert planner.completeness(planner.target_cells)['map_partial']
    assert not planner.completeness(planner.target_cells)['complete']


def test_native_connection_through_a_wall_is_rejected_without_changing_reachability():
    planner=CoveragePlanner(divided_room(True),.1,(0,0,0));planner.prepare(1.,1.)
    initial=planner.reachable.copy()
    component=int(planner.components[20,45])
    assert not planner.accept_native_connection(component,[(1.,2.),(4.5,2.)])
    assert np.array_equal(initial,planner.reachable)
    assert not planner.native_links
