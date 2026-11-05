import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import time
import xml.etree.ElementTree as ET
import pytest
import yaml
from ee3033_project.mapping_heading import terminal_path_yaw, mapping_tangent_tree

ROOT=Path(__file__).parents[1]/'src/ee3033_project'

@pytest.mark.parametrize('points,expected',[
    ([(0,0),(1,0),(1,1)],math.pi/2),
    ([(0,0),(0,1),(1,1)],0.),
    ([(0,0),(1,0),(1,0)],0.),
    ([(0,0),(-1,0)],math.pi),
    ([(0,1),(0,0)],-math.pi/2),
])
def test_terminal_heading_follows_actual_last_nonzero_segment(points,expected):
    assert terminal_path_yaw(points)==pytest.approx(expected)

def test_zero_length_and_nonfinite_paths_do_not_invent_a_frontier_heading():
    assert terminal_path_yaw([(1,1),(1,1)]) is None
    assert terminal_path_yaw([]) is None
    with pytest.raises(ValueError):terminal_path_yaw([(0,0),(float('nan'),1)])

@pytest.mark.parametrize('mode,state,expected',[
    ('automatic','MAPPING','navigate_mapping.xml'),
    ('waypoint','MAPPING',''),
    ('automatic','RETURNING','navigate_home.xml'),
])
def test_only_automatic_frontiers_select_the_tangent_behavior(mode,state,expected):
    source=ROOT/'ee3033_project/mapper.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef))
    env=dict(Node=object,Path=Path,time=time,get_package_share_directory=lambda _:str(ROOT),
             NavigateToPose=NS(Goal=lambda:NS(behavior_tree='')))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),env)
    node=env['Mapper'].__new__(env['Mapper']);sent=[]
    node.pending=False;node.nav_handle=None;node.goal_serial=0;node.mode=mode;node.state=state
    node.navigation_map=NS(fresh=lambda:True,allowed=lambda *a:True)
    node.make_pose=lambda p:p
    node.nav=NS(send_goal_async=lambda g:(sent.append(g) or NS(add_done_callback=lambda f:None)))
    node.send((1.,2.,math.pi/2))
    assert (Path(sent[0].behavior_tree).name if sent[0].behavior_tree else '')==expected

def test_mapping_tree_keeps_tangent_planning_but_uses_position_only_acceptance():
    tree=ET.parse(ROOT/'behavior_trees/navigate_mapping.xml')
    assert tree.find('.//ComputePathToPose').attrib['planner_id']=='MappingTangent'
    checker=tree.find('.//FollowPath').attrib['goal_checker_id']
    assert checker=='mapping_goal_checker'
    for name in ('navigation.yaml','navigation_dwb.yaml','navigation_rpp_native.yaml'):
        params=yaml.safe_load((ROOT/'config'/name).read_text())['controller_server']['ros__parameters']
        assert checker in params['goal_checker_plugins']
        assert params[checker]['yaw_goal_tolerance']==math.pi
        assert params[checker]['xy_goal_tolerance']==.12
        assert params['goal_checker']['yaw_goal_tolerance']==.20
    assert mapping_tangent_tree('/installed/behavior_trees/navigate_mapping.xml')
    assert not mapping_tangent_tree('navigate_home.xml')
    assert not mapping_tangent_tree('navigate_search_scan.xml')
