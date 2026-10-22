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
TURNING = {
    # angle > pi keeps Cautious identical to the original translation checker.
    'cautious': dict(angle=3.2, closed_loop=True, acceleration=0.8, sample=0.10, stop=0.16),
    'balanced': dict(angle=0.35, closed_loop=False, acceleration=0.4, sample=0.20, stop=0.15),
    'bold': dict(angle=0.20, closed_loop=False, acceleration=0.8, sample=0.30, stop=0.14),
}
PROGRESS_PLUGIN = 'nav2_controller::PoseProgressChecker'
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
    turn = TURNING[audacity]
    return {**{name: {'robot_radius': radius} for name in COSTMAPS},
            'project_mapper' if phase == 'mapping' else 'project_mission': {
                'clearance': AUDACITY[audacity][1] + (radius - BASE_RADIUS)},
            'controller_server': {
                'progress_checker.required_movement_angle': turn['angle'],
                'FollowPath.closed_loop': turn['closed_loop'],
                'FollowPath.max_angular_accel': turn['acceleration'],
                'FollowPath.forward_sampling_distance': turn['sample']},
            'project_velocity_gate': {'rotation_clearance': turn['stop']}}


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
    if actual['controller_server']['progress_checker.plugin'] != PROGRESS_PLUGIN:
        raise ValueError('Reload navigation with the heading-aware progress checker before configuring audacity')
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
                    turn_stop_distance=TURNING[level]['stop'],
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


def guard_rotation(node, inactive):
    from rcl_interfaces.msg import SetParametersResult

    def validate(changes):
        for p in changes:
            if p.name != 'rotation_clearance':
                continue
            if not inactive():
                return SetParametersResult(successful=False, reason='Stop before changing turning clearance')
            if (type(p.value) is not float or not math.isfinite(p.value)
                    or not 0.14 <= p.value <= 0.16):
                return SetParametersResult(successful=False, reason='turning clearance must be 0.14–0.16 m')
        return SetParametersResult(successful=True)
    node.add_on_set_parameters_callback(validate)
