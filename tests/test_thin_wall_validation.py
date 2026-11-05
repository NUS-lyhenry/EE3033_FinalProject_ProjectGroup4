"""The raw-map scorer must reject omissions, ghosts and closed/unknown doors."""
from pathlib import Path
import sys

import numpy as np
import pytest

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
from thin_wall_validation import cases,evaluate_map,fixture,fixture_scans,rotate,segment_distances


def ideal_map(angle=0):
    resolution=.05;width=100;height=80;origin=np.array([-2.5,-2.])
    rows,cols=np.indices((height,width))
    points=np.column_stack((origin[0]+(cols.ravel()+.5)*resolution,origin[1]+(rows.ravel()+.5)*resolution))
    grid=np.zeros((height,width),dtype=int)
    distance=np.min(segment_distances(points,fixture()['walls']),axis=1).reshape(height,width)
    grid[distance<=resolution*.51]=100
    return dict(width=width,height=height,resolution=resolution,
        origin=[*rotate([origin],np.deg2rad(angle))[0],np.deg2rad(angle)],cells=grid.ravel().tolist())


@pytest.mark.parametrize('angle',[0,15,30,45])
def test_complete_raw_wall_and_footprint_wide_opening_pass(angle):
    result=evaluate_map(ideal_map(angle),fixture(angle))
    assert result['passed'],result
    assert result['opening_free'] and result['ghost_cells']==0
    assert all(wall['supported_fraction']>=.95 for wall in result['walls'])


def test_wall_omission_cannot_pass_occupied_to_truth_distance_check():
    data=ideal_map();grid=np.array(data['cells']).reshape(data['height'],data['width'])
    grid[39:41,29:41]=0;data['cells']=grid.ravel().tolist()
    result=evaluate_map(data,fixture())
    assert result['ghost_cells']==0
    assert not result['passed']
    assert max(w['longest_missing_m'] for w in result['walls'])>.1


@pytest.mark.parametrize('value',[100,-1])
def test_actual_doorway_must_remain_observed_free(value):
    data=ideal_map();grid=np.array(data['cells']).reshape(data['height'],data['width'])
    grid[40,50]=value;data['cells']=grid.ravel().tolist()
    result=evaluate_map(data,fixture())
    assert not result['passed'] and not result['opening_free']


def test_ghost_obstacle_is_reported_separately_from_wall_recall():
    data=ideal_map();data['cells'][54*data['width']+50]=100
    result=evaluate_map(data,fixture())
    assert result['ghost_cells']==1 and not result['passed']


def test_empty_map_fails_without_nonfinite_report_values():
    data=ideal_map();data['cells']=[-1]*len(data['cells'])
    result=evaluate_map(data,fixture())
    assert not result['passed'] and all(w['supported_fraction']==0 for w in result['walls'])
    import json
    json.dumps(result,allow_nan=False)


def test_frozen_fixture_matrix_and_identical_sensor_sequence():
    settings,fixtures=cases()
    assert len(fixtures)==8
    assert settings['resolution']==.05 and settings['mask_radius']==.20
    assert len(fixture(sides='one')['poses'])==15 and len(fixture(sides='two')['poses'])==30
    scans,digest=fixture_scans(fixtures[0]);second,again=fixture_scans(fixtures[0])
    assert digest==again and scans==second
    assert len(scans[0])==450


def test_revised_slam_retains_matching_and_explicit_evidence_defaults():
    import yaml
    config=yaml.safe_load((SCRIPTS.parent/'src/ee3033_project/config/slam.yaml').read_text())['slam_toolbox']['ros__parameters']
    assert config['occupancy_threshold']==.1 and config['min_pass_through']==2
    assert config['use_scan_matching'] and config['do_loop_closing']
    assert config['correlation_search_space_dimension']==.16
