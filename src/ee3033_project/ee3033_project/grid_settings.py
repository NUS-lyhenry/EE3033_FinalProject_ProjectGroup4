"""Grid spacing controls for a newly loaded mapping or mission stage."""
import json
import math
import os

GRID_DEFAULTS = dict(mapping_resolution=.03, path_search_spacing=.01,
                     viewpoint_spacing=.55, coverage_spacing=.10)
GRID_LIMITS = {
    'mapping_resolution': (.01, .10),
    'path_search_spacing': (.01, .10),
    'viewpoint_spacing': (.15, 1.50),
    'coverage_spacing': (.05, .50),
}
STAGE_GRID_ENV = 'EE3033_STAGE_GRID_SETTINGS'


def validate_grid_settings(values):
    if not isinstance(values, dict) or not values.keys() <= GRID_LIMITS.keys():
        raise ValueError('Provide an object with supported grid settings')
    for key, value in values.items():
        low, high = GRID_LIMITS[key]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not low <= value <= high):
            raise ValueError(f'{key} must be between {low} and {high} m')
    return {key: float(value) for key, value in values.items()}


def subdivision_factor(resolution, spacing):
    # Map resolutions are ROS float32 values.
    return max(1, math.ceil(resolution / spacing - 1e-6))


def stage_grid_settings():
    return validate_grid_settings(json.loads(os.environ.get(STAGE_GRID_ENV, '{}')))
