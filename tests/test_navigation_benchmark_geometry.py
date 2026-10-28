import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from navigation_benchmark_geometry import physically_reachable,occupancy_grid,wall_distance


def room(divider):
    return dict(bounds=[-1.,-1.,1.,1.],start=[-.5,0.,0.],walls=[
        [[-1.,-1.],[1.,-1.]],[[1.,-1.],[1.,1.]],[[1.,1.],[-1.,1.]],[[-1.,1.],[-1.,-1.]],*divider])


def test_oracle_distinguishes_a_sealed_wall_from_a_clear_door():
    assert not physically_reachable(room([[[0.,-1.],[0.,1.]]]),[.5,0.,0.])
    assert physically_reachable(room([[[0.,-1.],[0.,-.3]],[[0.,.3],[0.,1.]]]),[.5,0.,0.])


def test_oracle_accounts_for_disc_width_and_not_only_line_of_sight():
    assert not physically_reachable(room([[[0.,-1.],[0.,-.14]],[[0.,.14],[0.,1.]]]),[.5,0.,0.])


def test_known_map_rasterizer_covers_thin_walls_and_uses_world_origin():
    data=occupancy_grid(room([[[0.,-1.],[0.,1.]]]))
    a=np.array(data['cells']).reshape(data['height'],data['width'])
    col=int((0-data['origin'][0])/.05)
    assert (a[:,col]==100).sum()>=40
    assert data['origin']==[-1.1,-1.1,0.]


def test_random_corpus_freezes_twenty_complete_physical_worlds():
    paths=sorted((ROOT/'validation/navigation_cases').glob('random_*.json'))
    assert len(paths)==20
    for seed,path in enumerate(paths):
        case=json.loads(path.read_text());assert case['scenario']['seed']==seed
        assert len(case['scenario']['walls'])>=8
        assert case['oracle']['reachable']
        assert physically_reachable(case['scenario'],case['goal'])


def test_live_case_keeps_real_slam_map_and_fixed_waypoint():
    case=json.loads((ROOT/'validation/navigation_cases/seed102_live.json').read_text())
    assert case['goal']==[2.99,-.29,1.5707963267948966]
    assert case['scenario']['seed']==102
    assert len(case['map']['cells'])==case['map']['width']*case['map']['height']
    assert case['stalled_pose'][:2]==[.5379702518903241,-.27937775950876004]
    assert 'map_from_odom' in case


def test_original_1023_world_is_not_the_revised_seed_generator():
    case=json.loads((ROOT/'validation/navigation_cases/seed1023.json').read_text())
    assert case['scenario']['seed']==1023
    assert case['scenario']['name']=='Random multi-room 3x3 1023'
    assert case['scenario']['bounds']==[-.7827150960845805,-3.78817742688364,3.6638419784090064,.7996258662115108]


def test_fixed_turn_and_gap_cases_have_independent_expected_reachability():
    for name,expected in [('successive_right_angles',True),('doorway',True),('tight_passage',True),
                          ('too_narrow',False),('dead_end_exit',True),('unreachable_target',False)]:
        case=json.loads((ROOT/'validation/navigation_cases'/f'{name}.json').read_text())
        assert physically_reachable(case['scenario'],case['goal']) is expected


def test_recorded_map_transform_is_inverted_for_physical_goal_evaluation():
    from navigation_benchmark import map_to_world
    import math
    import pytest
    t=dict(translation=[1.,2.,0.],rotation_xyzw=[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)])
    assert map_to_world([1.,3.,math.pi/2],t)==pytest.approx([1.,0.,0.])


def test_coverage_oracle_does_not_accept_an_unexplored_room_as_complete():
    from navigation_benchmark_geometry import observed_free_coverage
    scene=room([]);data=occupancy_grid(scene)
    t=dict(translation=[0.,0.,0.],rotation_xyzw=[0.,0.,0.,1.])
    assert observed_free_coverage(scene,data,t)['mapped_free_fraction']==1.
    cells=np.array(data['cells']).reshape(data['height'],data['width'])
    cells[:,cells.shape[1]//2:]=-1;data['cells']=cells.ravel().tolist()
    result=observed_free_coverage(scene,data,t)
    assert .4<result['mapped_free_fraction']<.6
    assert result['unknown_fraction']>.4


def test_worker_partitions_cover_the_full_corpus_once_and_keep_modes_together():
    from navigation_benchmark import partition_cases
    cases=list(range(20));parts=[partition_cases(cases,f'{i}/3')[0] for i in range(3)]
    assert sorted(i for part in parts for i,_ in part)==cases
    assert [len(part) for part in parts]==[7,7,6]
    assert all(i%3==shard for shard,part in enumerate(parts) for i,_ in part)


def test_aggregate_never_calls_one_worker_or_mixed_versions_complete():
    from navigation_benchmark import aggregate_matrices
    def matrix(shard):
        return dict(full_matrix_expected_runs=3,expected_runs=1,shard=[shard,3],rows=[dict(
            case_id=f'case{shard}',map_mode='known',task='navigate',repetition=0,controller='rpp',motion='ideal',
            passed=True,source_sha256={'core.py':'one'},source_config_sha256='same',controller_library_sha256='same')])
    parts=[matrix(i) for i in range(3)]
    assert not aggregate_matrices(parts[:1])['passed']
    assert aggregate_matrices(parts)['passed']
    parts[-1]['rows'][0]['source_sha256']['core.py']='changed'
    assert not aggregate_matrices(parts)['passed']


def test_aggregate_rejects_duplicate_runs_even_if_every_row_says_pass():
    from navigation_benchmark import aggregate_matrices
    row=dict(case_id='same',map_mode='known',task='navigate',repetition=0,controller='rpp',motion='ideal',
        passed=True,source_sha256={'core.py':'one'},source_config_sha256='same',controller_library_sha256='same')
    parts=[dict(full_matrix_expected_runs=2,expected_runs=1,shard=[i,2],rows=[dict(row)]) for i in range(2)]
    result=aggregate_matrices(parts)
    assert not result['passed'] and result['duplicate_runs']==1


def test_profile_change_recomputes_old_negative_without_rewriting_the_case():
    from navigation_benchmark_geometry import geometry_profile
    case=json.loads((ROOT/'validation/navigation_cases/too_narrow.json').read_text())
    assert case['door_width']==.28 and case['oracle']['reachable'] is False
    assert not physically_reachable(case['scenario'],case['goal'],radius=geometry_profile('legacy')['footprint_radius'])
    assert physically_reachable(case['scenario'],case['goal'],radius=geometry_profile('measured')['footprint_radius'])
    measured=json.loads((ROOT/'validation/navigation_cases/measured_too_narrow.json').read_text())
    assert not physically_reachable(measured['scenario'],measured['goal'],radius=geometry_profile('measured')['footprint_radius'])


def test_independent_measured_circle_contains_all_rectangle_corners():
    from navigation_benchmark_geometry import geometry_profile
    import math
    g=geometry_profile('measured')
    assert g['length']==.115 and g['width']==.18
    assert g['physical_radius']>=math.hypot(g['length']/2,g['width']/2)
    assert g['physical_radius']==.106801 and g['footprint_radius']==.116801
