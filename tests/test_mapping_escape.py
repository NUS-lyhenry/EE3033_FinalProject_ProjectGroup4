import json
from pathlib import Path

import numpy as np
from ee3033_project.mapping_escape import mapping_escape_goal


def test_reported_corner_selects_an_open_known_free_repositioning_point():
    fixture = Path(__file__).parent/'fixtures/legacy_corner_slam_map.json'
    m = json.loads(fixture.read_text())
    grid = np.asarray(m['cells']).reshape(m['height'], m['width'])
    goal = mapping_escape_goal(grid, m['resolution'], m['origin'], (.446, .465, .74))
    assert goal is not None
    # This recorded map has open space on the upper-left side of the corner.
    assert -.1 < goal[0] < .25 and .7 < goal[1] < 1.0
    # A stricter configured reserve rejects recovery if the current pose is
    # already too close to a mapped obstacle to certify its entire connector.
    assert mapping_escape_goal(grid, m['resolution'], m['origin'], (.446, .465, .74),
                               requested_clearance=.30) is None


def test_unknown_space_and_blocked_navigation_mask_fail_closed():
    grid = np.full((40, 40), -1)
    assert mapping_escape_goal(grid, .05, (0., 0., 0.), (1., 1., 0.)) is None
    grid[:] = 0
    assert mapping_escape_goal(grid, .05, (0., 0., 0.), (1., 1., 0.),
                               np.zeros_like(grid, dtype=bool)) is None


def test_escape_requires_improved_clearance_and_valid_pose():
    grid = np.zeros((40, 40), dtype=int)
    assert mapping_escape_goal(grid, .05, (0., 0., 0.), (1., 1., 0.)) is None
    assert mapping_escape_goal(grid, .05, (0., 0., 0.), (float('nan'), 1., 0.)) is None


def test_open_far_side_uses_a_short_safe_leg_toward_the_stalled_goal():
    fixture = Path(__file__).parent/'fixtures/legacy_corner_far_side_slam_map.json'
    m = json.loads(fixture.read_text())
    grid = np.asarray(m['cells']).reshape(m['height'], m['width'])
    goal = mapping_escape_goal(grid, m['resolution'], m['origin'], (1.247, -.363, 2.6),
                               requested_clearance=.14, stalled_goal=(-1.005, -.388, 1.1))
    assert goal is not None
    assert .75 < goal[0] < 1.0 and -.1 < goal[1] < .3
