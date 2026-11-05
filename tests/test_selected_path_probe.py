"""Only an already selected remote position receives a native planner probe."""
import math
from types import SimpleNamespace as NS
import numpy as np
from ee3033_project.coverage import CoveragePlanner,Viewpoint
from ee3033_project.core import angle_difference
from test_mission_policy import mission,namespace


class Done:
    def __init__(self,value):self.value=value
    def done(self):return True
    def result(self):return self.value


def owner(options,statuses=()):
    namespace['angle_difference']=angle_difference
    namespace['ComputePathToPose']=NS(Goal=lambda:NS())
    namespace['GoalStatus']=NS(STATUS_SUCCEEDED=4)
    m=mission();m.initialize_search_recovery();m.seen=set();m.attempted=set();m.failed_options=set()
    m.p=dict(search_strategy='greedy',adaptive_margin=.08,observe_seconds=3.,min_observe_seconds=.8,
             search_linear_speed=.12,search_angular_speed=.4)
    m.get_parameter=lambda name:NS(value=m.p[name]);m.make_pose=lambda *v:v
    m.planner=CoveragePlanner(np.zeros((40,50),int),.1,(0,0,0))
    m.planner.options=tuple(options);m.planner.native_probes=[]
    m.requests=[];m.statuses=iter(statuses)
    def send(goal):
        m.requests.append(goal.goal)
        status=next(m.statuses,4)
        points=[NS(pose=NS(position=NS(x=goal.goal[0],y=goal.goal[1])))]
        value=NS(status=status,result=NS(path=NS(poses=points)))
        return Done(NS(accepted=True,get_result_async=lambda:Done(value),cancel_goal_async=lambda:None))
    m.path_nav=NS(server_is_ready=lambda:True,send_goal_async=send)
    return m


def v(x,y,yaw=0.,cells=((1,1),)):
    return Viewpoint(x,y,yaw,frozenset(cells))


def test_same_site_has_no_candidate_probe_even_when_other_stations_exist():
    m=owner([v(1.05,1.05),v(1.65,1.05,cells=((1,2),))])
    assert m.select_viewpoint((1.05,1.05,0.))==0
    assert m.requests==[]


def test_remote_winner_checks_one_position_without_top_three_comparison():
    m=owner([v(1.65,1.05),v(1.65,1.05,.5),v(2.25,1.05,cells=((1,2),))])
    selected=m.select_viewpoint((1.05,1.05,0.))
    assert selected in (0,1) and len(m.requests)==1
    assert m.requests[0][:2]==(1.65,1.05)


def test_failed_remote_position_selects_another_station_not_another_heading():
    m=owner([v(1.65,1.05),v(1.65,1.05,.5),v(2.25,1.05,cells=((1,2),))],statuses=(6,4))
    assert m.select_viewpoint((1.05,1.05,0.))==2
    assert [r[:2] for r in m.requests]==[(1.65,1.05),(2.25,1.05)]
    assert not m.failed_options  # Native rejection is decision-local.


def test_independent_connectivity_probe_is_retained_when_candidates_exhausted():
    m=owner([])
    probe=v(2.25,1.05)
    m.planner.native_probes=[(2,probe)]
    links=[]
    def connected(component,points,deadline,cancel):
        links.append((component,points));m.planner.options=(probe,)
    m.planner.accept_native_connection=connected
    assert m.select_viewpoint((1.05,1.05,0.))==0
    assert links and links[0][0]==2
    assert len(m.requests)==2  # One connectivity query, one selected position.
