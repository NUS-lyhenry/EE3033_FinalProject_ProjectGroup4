"""Keep the reported corner reproducible after the random generator changes."""
import importlib.util
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location('simulation_scenarios', ROOT/'scripts/simulation_scenarios.py')
scenarios = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scenarios)


def test_reported_corner_scene_has_a_clear_route_around_the_wall():
    scene = scenarios.validate_scenario(json.loads(
        (ROOT/'validation/corner_seed95_legacy_scenario.json').read_text()))
    # This witness is physical geometry only; it does not assert that Nav2
    # will find or drive the route with its incomplete SLAM map.
    route = [(0., 0.), (0., .95), (1.1, .95), (1.1, .1)]
    for start, end in zip(route, route[1:]):
        points = np.linspace(start, end, 101)
        assert np.min(scenarios._clearance(points, scene['walls'])) >= .40
