"""Bounded, temporary navigation trial settings (not balance-controller PID)."""
import math


LIMITS = {
    'max_linear': (0.04, 0.18, 0.01, 'm/s'),
    'max_angular': (0.15, 0.60, 0.05, 'rad/s'),
    'goal_timeout': (20.0, 120.0, 5.0, 's'),
}


def validate_trial(values):
    if not isinstance(values, dict) or set(values) != set(LIMITS):
        raise ValueError('Provide forward speed, turning speed, and goal timeout')
    result = {}
    for name, (low, high, _, _) in LIMITS.items():
        value = values[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'{name} must be a number')
        value = float(value)
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{name} must be between {low} and {high}')
        result[name] = value
    return result


PROFILES = {
    'original': dict(max_linear=.12, max_angular=.40, goal_timeout=60.),
    'fast': dict(max_linear=.16, max_angular=.60, goal_timeout=60.),
}


def parameter_plan(values, phase='mapping'):
    values = validate_trial(values)
    linear, angular = values['max_linear'], values['max_angular']
    return {
        'project_velocity_gate': {'max_linear': linear, 'max_angular': angular},
        'controller_server': {
            'FollowPath.desired_linear_vel': linear, 'FollowPath.max_linear_vel': linear,
            'FollowPath.rotate_to_heading_angular_vel': angular, 'FollowPath.max_angular_vel': angular,
        },
        'project_navigation_supervisor': {'goal_timeout': values['goal_timeout']},
        'project_mapper' if phase == 'mapping' else 'project_mission':
            ({'goal_timeout': values['goal_timeout']} if phase == 'mapping' else
             {'goal_timeout': values['goal_timeout'], 'search_linear_speed': linear,
              'search_angular_speed': angular}),
    }


def guarded_parameters(node, names, inactive):
    """Validate ROS writes; consumers read the committed parameter each tick."""
    from rcl_interfaces.msg import SetParametersResult

    def validate(changes):
        relevant = [p for p in changes if p.name in names]
        if relevant and not inactive():
            return SetParametersResult(successful=False, reason='Stop mapping before tuning')
        for p in relevant:
            low, high, _, _ = LIMITS[p.name]
            if (type(p.value) is not float or not math.isfinite(p.value)
                    or not low <= p.value <= high):
                return SetParametersResult(successful=False, reason=f'{p.name}: allowed {low}–{high}')
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(validate)
