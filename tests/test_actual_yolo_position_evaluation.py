"""Target errors require a recorded frame alignment, never raw frame mixing."""
from pathlib import Path
import math
import runpy
import pytest

evaluate=runpy.run_path(str(Path(__file__).parents[1]/'scripts/actual_yolo_search_comparison.py'))['evaluate_target_position']


def test_rotated_and_translated_world_truth_is_compared_in_map_frame():
    before=dict(pose=[3.,4.,math.pi],simulation_motion=dict(ground_truth=[1.,2.,math.pi/2]))
    result=evaluate(before,dict(target=dict(x=2.1,y=4.,sigma=.3)),dict(target=[1.,3.,0.]))
    assert result['estimated_truth_map']==pytest.approx([2.,4.])
    assert result['estimated_map_from_world']==pytest.approx([5.,3.,math.pi/2])
    assert result['position_error_m']==pytest.approx(.1)
    assert result['evaluation_only'] is True


def test_missing_alignment_never_assumes_map_and_world_are_identical():
    result=evaluate({},dict(target=dict(x=1.,y=2.)),dict(target=[1.,2.,0.]))
    assert result['position_error_m'] is None
    assert 'Missing' in result['error']


def test_unreported_target_has_no_invented_error_or_coordinate():
    before=dict(pose=[0.,0.,0.],simulation_motion=dict(ground_truth=[0.,0.,0.]))
    result=evaluate(before,dict(target=None),dict(target=[1.,2.,0.]))
    assert result['estimated_truth_map']==[1.,2.]
    assert result['reported_target_map'] is None
    assert result['position_error_m'] is None
