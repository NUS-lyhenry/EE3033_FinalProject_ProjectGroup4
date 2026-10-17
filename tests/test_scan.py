import math
from ee3033_project.scan_core import resample


def test_self_returns_become_unknown_not_false_clear_rays():
    ranges=resample([0.05,0.07,0.15,0.25],-math.pi,math.pi/2,0.,8.,4,0.14)
    assert math.isnan(ranges[0]) and math.isnan(ranges[1])
    assert ranges[2:]==[0.15,0.25]


def test_nearest_obstacle_survives_bin_merge():
    ranges=resample([2.,1.,3.],0.,0.001,0.,8.,450,0.14)
    assert min(ranges)==1.


def test_self_occlusion_wins_over_another_point_in_same_bin():
    ranges=resample([0.05,1.],0.,0.001,0.,8.,450,0.14)
    assert math.isnan(ranges[225])


def test_invalid_ranges_and_empty_scan_do_not_create_obstacles():
    ranges=resample([math.nan,math.inf,-1.,20.],0.,0.5,0.,8.,450,0.14)
    assert all(math.isinf(r) for r in ranges)
