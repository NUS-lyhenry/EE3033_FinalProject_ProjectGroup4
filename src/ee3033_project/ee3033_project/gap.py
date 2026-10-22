"""Temporary narrow-passage policy; never changes sensing or the physical floor."""
import math

BASE_RADIUS = 0.14
ROBOT_WIDTH = 0.20  # user estimate; physical validation remains required
SIDE_RESERVE = 0.08
RESERVED_RADIUS = max(BASE_RADIUS, ROBOT_WIDTH / 2 + SIDE_RESERVE)
BASE_CLEARANCE = 0.22
AUDACITY = {
    'cautious': ('Cautious', BASE_CLEARANCE),
    'balanced': ('Balanced', 0.18),
    'bold': ('Bold', 0.14),
}
COSTMAPS = ('global_costmap/global_costmap', 'local_costmap/local_costmap')


def validate_audacity(value):
    if not isinstance(value, str) or value not in AUDACITY:
        raise ValueError('audacity must be cautious, balanced or bold')
    return value


def parameter_plan(enabled, phase, audacity='cautious'):
    if type(enabled) is not bool:
        raise ValueError('enabled must be true or false')
    if phase not in ('mapping', 'mission'):
        raise ValueError('Invalid stage')
    validate_audacity(audacity)
    radius = RESERVED_RADIUS if enabled else BASE_RADIUS
    return {**{name: {'robot_radius': radius} for name in COSTMAPS},
            'project_mapper' if phase == 'mapping' else 'project_mission': {
                'clearance': AUDACITY[audacity][1] + (radius - BASE_RADIUS)}}


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
    def matches(enabled, audacity):
        return all(math.isclose(actual[n][k], v, abs_tol=1e-8)
                   for n, entries in parameter_plan(enabled, phase, audacity).items() for k, v in entries.items())
    match = next(((enabled, level) for enabled in (False, True) for level in AUDACITY
                  if matches(enabled, level)), (None, None))
    enabled, audacity = match
    padding = max(actual[n]['footprint_padding'] for n in COSTMAPS)
    radius = RESERVED_RADIUS if enabled else BASE_RADIUS
    reserve = radius - BASE_RADIUS
    options = [dict(id=level, label=label, clearance=clearance+reserve,
                    nominal_min_opening=2*max(clearance+reserve, radius+padding))
               for level, (label, clearance) in AUDACITY.items()]
    return {'enabled': enabled, 'audacity': audacity, 'audacity_options': options,
            'synchronized': audacity is not None,
            'estimated_robot_width': ROBOT_WIDTH, 'side_reserve': SIDE_RESERVE,
            'nominal_min_opening': 2 * RESERVED_RADIUS,
            'effective_min_opening': next((p['nominal_min_opening'] for p in options
                                            if p['id']==audacity), None),
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
                    or not AUDACITY['bold'][1] <= p.value <= 0.5):
                return SetParametersResult(successful=False, reason='clearance must be 0.14–0.50 m')
        return SetParametersResult(successful=True)
    node.add_on_set_parameters_callback(validate)
