"""World generation validates physical input, never synthesizes robot output."""
import copy
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('simulation_scenarios',
    Path(__file__).parents[1]/'scripts/simulation_scenarios.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s)


def test_original_scene_is_preserved_and_independent():
    original=s.default_scenario()
    assert s.validate_scenario(original)==original
    assert original['start']==[0.,0.,0.] and original['target']==[.95,.85,0.]
    original['walls'][0][0][0]=999
    assert s.default_scenario()['walls'][0][0][0]==-1.2


def test_random_scenes_are_deterministic_json_and_have_varied_occluders():
    scenes=[s.generate_scenario(seed) for seed in range(15)]
    assert scenes[3]==s.generate_scenario(3)
    assert len({json.dumps(scene['walls']) for scene in scenes})==15
    for scene in scenes:
        assert 12<=len(scene['walls'])<=48
        assert s.validate_scenario(json.loads(json.dumps(scene)))==scene
        assert np.asarray(scene['walls']).shape[1:]==(2,2)
        assert s._clearance(scene['start'][:2],scene['walls'])>=s.ROBOT_CLEARANCE
        assert s._clearance(scene['target'][:2],scene['walls'])>=s.ROBOT_CLEARANCE


def test_manual_target_preserves_world_and_checks_wall_inside_floor_and_connectivity():
    source=s.default_scenario();saved=copy.deepcopy(source)
    result=s.with_target(source,[-.6,.6])
    assert source==saved and result['walls']==saved['walls']
    assert result['target']==[-.6,.6,0.]
    for target,reason in [([.55,.7],'clearance'),([9.,0.],'outside'),
                          ([.8,.8,.1],'floor'),([0.,0.],'initial chassis')]:
        with pytest.raises(ValueError,match=reason):s.with_target(source,target)
    divided=s.default_scenario();divided['walls'].append([[.45,-1.],[.45,1.3]])
    with pytest.raises(ValueError,match='connected'):s.with_target(divided,[.95,.85])


def test_random_target_preserves_map_and_start_and_is_reproducible():
    source=s.generate_scenario(123)
    a=s.randomize_target(source,9);b=s.random_target(source,9)
    assert a==b and a['walls']==source['walls'] and a['start']==source['start']
    assert a['target']!=s.random_target(source,12)['target']
    assert s.validate_scenario(s.random_target(source))


@pytest.mark.parametrize('seed',[-1,2**32,True,'10',1.5,None])
def test_invalid_generation_seed_rejected(seed):
    with pytest.raises(ValueError,match='seed'):s.generate_scenario(seed)


def test_invalid_outer_walls_and_nonfinite_inputs_rejected():
    scene=s.default_scenario();scene['walls'][0][1][0]-=.1
    with pytest.raises(ValueError,match='close'):s.validate_scenario(scene)
    scene=s.default_scenario();scene['start'][0]=float('nan')
    with pytest.raises(ValueError,match='finite'):s.validate_scenario(scene)
    scene=s.default_scenario();scene['walls'].append([[0.,0.],[0.,0.]])
    with pytest.raises(ValueError,match='nonzero'):s.validate_scenario(scene)
    scene=s.default_scenario();scene['version']=True
    with pytest.raises(ValueError,match='version'):s.validate_scenario(scene)


def test_multi_room_spanning_connections_and_loops_reach_every_room_and_door():
    for seed in range(20):
        scene,topology=s._multi_room_layout(seed)
        scene=s.validate_scenario(scene)
        rooms=topology['rooms'];connections=topology['connections']
        assert len(rooms) in (6,9)
        assert len(connections)>=len(rooms)  # spanning tree plus at least one cycle
        assert len(topology['room_distances'])==len(rooms)
        points,reachable=s._connected_grid(scene)
        reachable_points=points[reachable]
        # This checks the inflated physical wall geometry as well as the graph.
        for room in rooms:
            assert np.min(np.linalg.norm(reachable_points-room,axis=-1))<.08
            assert s._clearance(room,scene['walls'])>.5
        for door in topology['doors']:
            assert door['width']>=.74
            if not door['open']:assert door['width']<=.92
            assert s._clearance(door['center'],scene['walls'])>=door['width']/2-1e-8
            assert np.min(np.linalg.norm(reachable_points-door['center'],axis=-1))<.08
        assert topology['room_distances'][topology['target_room']]>=2
        assert np.linalg.norm(scene['target'][:2])>.9
        assert s._segment_blocked(scene['start'][:2],scene['target'][:2],scene['walls'])


def test_complex_rooms_cover_103_seeds_without_invalid_or_disconnected_worlds():
    dimensions=set();loop_counts=set()
    for seed in [*range(100),4294967295,30903,71929]:
        scene,topology=s._multi_room_layout(seed)
        assert s.generate_scenario(seed)==s.validate_scenario(scene)
        dimensions.add((topology['rows'],topology['columns']))
        loop_counts.add(len(topology['connections'])-len(topology['rooms'])+1)
        assert max(scene['bounds'][2]-scene['bounds'][0],scene['bounds'][3]-scene['bounds'][1])>4.
        assert len(scene['walls'])<=48
        points,reachable=s._connected_grid(scene)
        assert all(np.min(np.linalg.norm(points[reachable]-room,axis=-1))<.08
                   for room in topology['rooms'])
        assert s._segment_blocked(scene['start'][:2],scene['target'][:2],scene['walls'])
    assert dimensions=={(2,3),(3,3)} and len(loop_counts)>1


def test_seed_95_door_crossings_have_clearance_from_both_sides():
    # Geometry only: this does not exercise Nav2, the velocity gate or physics.
    scene, topology = s._multi_room_layout(95)
    s.validate_scenario(scene)
    for door in topology['doors']:
        a, b = (np.asarray(topology['rooms'][i]) for i in door['rooms'])
        normal = b-a
        normal /= np.linalg.norm(normal)
        tangent = np.array([-normal[1], normal[0]])
        center = np.asarray(door['center'])
        for direction in (-1, 1):
            for offset in (-.10, 0., .10):
                start = center-direction*.30*normal + offset*tangent
                end = center+direction*.30*normal - offset*tangent
                path = np.linspace(start, end, 101)
                assert np.min(s._clearance(path, scene['walls'])) >= s.ROBOT_CLEARANCE
