"""Temporary narrow-passage policy; never changes sensing or the physical floor."""
import math

from .robot_geometry import (DEFAULT_STOP_DISTANCE, GEOMETRY_PROFILE, NAVIGATION_RADIUS, ROBOT_WIDTH,
                             FOOTPRINT_PADDING, FOOTPRINT_RADIUS, MIN_CONFIGURED_COLLISION_RADIUS, ROTATION_CLEARANCES,
                             geometry_metadata)

BASE_RADIUS = NAVIGATION_RADIUS
SIDE_RESERVE = 0.08
RESERVED_RADIUS = max(BASE_RADIUS, ROBOT_WIDTH / 2 + SIDE_RESERVE)
BASE_CLEARANCE = 0.10
AUDACITY = {
    'cautious': ('Cautious', BASE_CLEARANCE),
    'balanced': ('Balanced', 0.08),
    'bold': ('Bold', 0.04),
}
DEFAULT_AUDACITY = 'bold'
DEFAULT_INFLATION_RADIUS = 0.12
TURNING = {level:dict(stop=distance) for level,distance in ROTATION_CLEARANCES.items()}
PROGRESS_PLUGIN = 'nav2_controller::SimpleProgressChecker'
COSTMAPS = ('global_costmap/global_costmap', 'local_costmap/local_costmap')
TUNING_LIMITS = {
    'robot_radius': (round(BASE_RADIUS * .60, 6), .30),
    'footprint_radius': (MIN_CONFIGURED_COLLISION_RADIUS, .30),
    'footprint_padding': (FOOTPRINT_PADDING, .10),
    'inflation_radius': (round(BASE_RADIUS * .60 + FOOTPRINT_PADDING, 6), .60),
    'clearance': (AUDACITY[DEFAULT_AUDACITY][1], .50),
    'stop_distance': (round(DEFAULT_STOP_DISTANCE * .60, 6), .50),
    'rotation_clearance': (round(TURNING[DEFAULT_AUDACITY]['stop'] * .60, 6), .50),
}
GATE_FIELDS = ('stop_distance', 'rotation_clearance', 'footprint_radius')


def validate_audacity(value):
    if not isinstance(value, str) or value not in AUDACITY:
        raise ValueError('audacity must be cautious, balanced or bold')
    return value


def validate_tuning(overrides):
    if not isinstance(overrides, dict) or not overrides.keys() <= TUNING_LIMITS.keys():
        raise ValueError('Provide an object with supported clearance settings')
    for name, value in overrides.items():
        low, high = TUNING_LIMITS[name]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not low <= value <= high):
            raise ValueError(f'{name} must be between {low:.6f} and {high:.6f} m')
    return {name: float(value) for name, value in overrides.items()}


def tuning_defaults(audacity):
    return dict(robot_radius=BASE_RADIUS, footprint_radius=FOOTPRINT_RADIUS,
                footprint_padding=FOOTPRINT_PADDING,
                inflation_radius=DEFAULT_INFLATION_RADIUS, clearance=AUDACITY[audacity][1],
                stop_distance=DEFAULT_STOP_DISTANCE, rotation_clearance=TURNING[audacity]['stop'])


MANUAL_DEFAULTS = (dict(robot_radius=.14, footprint_radius=.07, footprint_padding=.01,
                        inflation_radius=.14, clearance=.12, stop_distance=.14,
                        rotation_clearance=.14) if GEOMETRY_PROFILE == 'measured'
                   else tuning_defaults(DEFAULT_AUDACITY))
DEFAULT_CLEARANCE = MANUAL_DEFAULTS['clearance']


def parameter_plan(enabled, phase, audacity=DEFAULT_AUDACITY, tuning=None):
    if type(enabled) is not bool:
        raise ValueError('enabled must be true or false')
    if phase not in ('mapping', 'mission'):
        raise ValueError('Invalid stage')
    validate_audacity(audacity)
    values = tuning_defaults(audacity)
    if tuning is not None:
        values.update(validate_tuning(tuning))
    base_radius = values['robot_radius']
    radius = max(RESERVED_RADIUS, base_radius) if enabled else base_radius
    padding = values['footprint_padding']
    inflation = max(values['inflation_radius'], radius + padding)
    return {**{name: {'robot_radius': radius,
                     'footprint_padding': padding,
                     'inflation_layer.inflation_radius': inflation} for name in COSTMAPS},
            'project_mapper' if phase == 'mapping' else 'project_mission': {
                'clearance': values['clearance'] + (radius - base_radius)},
            'controller_server': {},
            'project_velocity_gate': {key: values[key] for key in GATE_FIELDS}}


def manual_policy(policy):
    """Convert legacy presets to manual values while keeping their effective geometry."""
    plan = parameter_plan(phase='mapping', **policy)
    if not policy['enabled'] and policy['audacity'] == DEFAULT_AUDACITY:
        return policy
    costmap = plan[COSTMAPS[0]]
    values = dict(plan['project_velocity_gate'],
                  robot_radius=costmap['robot_radius'], footprint_padding=costmap['footprint_padding'],
                  inflation_radius=costmap['inflation_layer.inflation_radius'],
                  clearance=plan['project_mapper']['clearance'])
    defaults = tuning_defaults(DEFAULT_AUDACITY)
    return dict(enabled=False, audacity=DEFAULT_AUDACITY,
                tuning={key: value for key, value in values.items() if value != defaults[key]})


def describe(actual, phase, policy=None):
    """Only recognize the configured circular envelope with working inflation.

    Read the committed ROS parameters, not a UI preference. A custom polygon or
    disabled inflation could defeat radius-based rejection and must fail closed.
    """
    for name in COSTMAPS:
        p = actual[name]
        if (p['footprint'].replace(' ', '') not in ('', '[]')
                or p['inflation_layer.enabled'] is not True
                or p['footprint_padding'] < FOOTPRINT_PADDING
                or p['inflation_layer.inflation_radius'] + 1e-9 < p['robot_radius'] + p['footprint_padding']):
            raise ValueError(f'{name}: unsupported footprint/inflation configuration')
    if actual['controller_server']['progress_checker.plugin'] != PROGRESS_PLUGIN:
        raise ValueError('Reload navigation with the original translation progress checker before configuring audacity')
    def matches(enabled, audacity, tuning=None):
        return all(math.isclose(actual[n][k], v, abs_tol=1e-8)
                   for n, entries in parameter_plan(enabled, phase, audacity, tuning).items()
                   for k, v in entries.items())
    overrides = policy.get('tuning', {}) if policy else {}
    manual_default = policy is None and matches(False, DEFAULT_AUDACITY, MANUAL_DEFAULTS)
    if manual_default:
        match = (False, DEFAULT_AUDACITY)
    elif overrides:
        enabled, level = policy['enabled'], policy['audacity']
        match = (enabled, level) if matches(enabled, level, overrides) else (None, None)
    else:
        match = next(((enabled, level) for enabled in (False, True) for level in AUDACITY
                      if matches(enabled, level)), (None, None))
    enabled, audacity = match
    padding = max(actual[n]['footprint_padding'] for n in COSTMAPS)
    consumer = 'project_mapper' if phase == 'mapping' else 'project_mission'
    options = []
    for level, (label, _) in AUDACITY.items():
        plan = parameter_plan(bool(enabled), phase, level, MANUAL_DEFAULTS if manual_default else overrides)
        clearance = plan[consumer]['clearance']
        radius = plan[COSTMAPS[0]]['robot_radius']
        gate = plan['project_velocity_gate']
        options.append(dict(id=level, label=label, clearance=clearance,
                            turn_stop_distance=gate['rotation_clearance'],
                            nominal_min_opening=2*max(clearance, radius+padding, FOOTPRINT_RADIUS,
                                                     gate['footprint_radius'])))
    requested = (dict(MANUAL_DEFAULTS) if manual_default else
                 dict(tuning_defaults(audacity or DEFAULT_AUDACITY), **overrides))
    costmap = actual[COSTMAPS[0]]
    live = dict(actual['project_velocity_gate'], clearance=actual[consumer]['clearance'],
                robot_radius=costmap['robot_radius'], footprint_padding=costmap['footprint_padding'],
                inflation_radius=costmap['inflation_layer.inflation_radius'])
    return {'enabled': enabled, 'audacity': audacity, 'audacity_options': options,
            'synchronized': audacity is not None,
            'estimated_robot_width': ROBOT_WIDTH, 'side_reserve': SIDE_RESERVE,
            'robot_geometry': geometry_metadata(),
            'nominal_min_opening': 2 * RESERVED_RADIUS,
            'effective_min_opening': next((p['nominal_min_opening'] for p in options
                                            if p['id']==audacity), None),
            'tuning': {'defaults': dict(MANUAL_DEFAULTS), 'overrides': overrides,
                       'requested': requested, 'live': live,
                       'limits': {key: {'min': low, 'max': high} for key, (low, high) in TUNING_LIMITS.items()}},
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
                return SetParametersResult(successful=False, reason=f"clearance must be {AUDACITY['bold'][1]:.6f}–0.50 m")
        return SetParametersResult(successful=True)
    node.add_on_set_parameters_callback(validate)


def guard_gate_tuning(node, inactive):
    from rcl_interfaces.msg import SetParametersResult

    def validate(changes):
        for p in changes:
            if p.name not in GATE_FIELDS:
                continue
            if not inactive():
                return SetParametersResult(successful=False, reason='Stop before changing obstacle protection')
            low, high = TUNING_LIMITS[p.name]
            if (type(p.value) is not float or not math.isfinite(p.value)
                    or not low <= p.value <= high):
                return SetParametersResult(successful=False, reason=f'{p.name} must be {low:.6f}–{high:.6f} m')
        return SetParametersResult(successful=True)
    node.add_on_set_parameters_callback(validate)
