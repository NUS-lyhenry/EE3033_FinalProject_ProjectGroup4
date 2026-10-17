"""Temporary narrow-passage policy; never changes sensing or the physical floor."""
import math

BASE_RADIUS = 0.14
ROBOT_WIDTH = 0.20  # user estimate; physical validation remains required
SIDE_RESERVE = 0.08
RESERVED_RADIUS = max(BASE_RADIUS, ROBOT_WIDTH / 2 + SIDE_RESERVE)
BASE_CLEARANCE = 0.22
COSTMAPS = ('global_costmap/global_costmap', 'local_costmap/local_costmap')


def parameter_plan(enabled, phase):
    if type(enabled) is not bool:
        raise ValueError('enabled must be true or false')
    if phase not in ('mapping', 'mission'):
        raise ValueError('Invalid stage')
    radius = RESERVED_RADIUS if enabled else BASE_RADIUS
    return {**{name: {'robot_radius': radius} for name in COSTMAPS},
            'project_mapper' if phase == 'mapping' else 'project_mission': {
                'clearance': BASE_CLEARANCE + (radius - BASE_RADIUS)}}


def describe(actual, phase):
    """Only recognize the configured circular envelope with working inflation.

    Read the committed ROS parameters, not a UI preference. A custom polygon or
    disabled inflation could defeat radius-based rejection and must fail closed.
    """
    for name in COSTMAPS:
        p = actual[name]
        if (p['footprint'].replace(' ', '') not in ('', '[]')
                or p['inflation_layer.enabled'] is not True
                or p['footprint_padding'] < 0.01
                or p['inflation_layer.inflation_radius'] < RESERVED_RADIUS + p['footprint_padding']):
            raise ValueError(f'{name}: unsupported footprint/inflation configuration')
    def matches(enabled):
        return all(math.isclose(actual[n][k], v, abs_tol=1e-8)
                   for n, entries in parameter_plan(enabled, phase).items() for k, v in entries.items())
    enabled = True if matches(True) else False if matches(False) else None
    return {'enabled': enabled, 'synchronized': enabled is not None,
            'estimated_robot_width': ROBOT_WIDTH, 'side_reserve': SIDE_RESERVE,
            'nominal_min_opening': 2 * RESERVED_RADIUS,
            'nodes': actual, 'temporary': True}


def guard_clearance(node, inactive):
    from rcl_interfaces.msg import SetParametersResult

    def validate(changes):
        for p in changes:
            if p.name != 'clearance':
                continue
            if not inactive():
                return SetParametersResult(successful=False, reason='Stop before changing clearance')
            if (type(p.value) is not float or not math.isfinite(p.value)
                    or not BASE_CLEARANCE <= p.value <= 0.5):
                return SetParametersResult(successful=False, reason='clearance must be 0.22–0.50 m')
        return SetParametersResult(successful=True)
    node.add_on_set_parameters_callback(validate)
