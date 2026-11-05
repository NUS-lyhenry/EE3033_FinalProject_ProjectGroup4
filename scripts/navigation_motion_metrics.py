"""Read-only, time-weighted command/odometry diagnostics for native benchmarks.

These metrics never modify a command or feed simulated truth to navigation.
Regulation floors may still be reduced by curvature-preserving angular caps;
record the final output instead of inferring actual motion from a parameter.
"""
import math
from threading import Lock


SPEED_PROFILES = {'original': (.12, .40), 'fast': (.16, .60)}


def select_navigation_phase(task, map_mode, requested=None):
    phase = requested or ('mission' if task == 'navigate' and map_mode == 'known' else 'mapping')
    if phase not in ('mapping', 'mission'):
        raise ValueError('Navigation phase must be mapping or mission')
    if phase == 'mission' and (task != 'navigate' or map_mode != 'known'):
        raise ValueError('Mission-phase navigation requires a known map and navigate task')
    return phase


def configure_speed(configuration, controller, profile):
    """Apply equal requested speed limits to either retained controller."""
    linear, angular = SPEED_PROFILES[profile]
    follow = configuration['controller_server']['ros__parameters']['FollowPath']
    if controller == 'dwb':
        follow.update(max_vel_x=linear, max_speed_xy=linear, max_vel_theta=angular)
    else:
        follow.update(desired_linear_vel=linear, rotate_to_heading_angular_vel=angular)
        if controller == 'rpp':
            follow.update(max_linear_vel=linear, max_angular_vel=angular)
    configuration['behavior_server']['ros__parameters']['max_rotational_vel'] = angular
    return dict(max_linear=linear, max_angular=angular)


def arrival_limits(task, goal_kind):
    if task == 'automap':
        return .25, None
    if goal_kind == 'approach':
        return .025, .10
    if goal_kind == 'home':
        return .06, None
    return (.15, .15) if task == 'waypoint' else (.15, .22)


class MotionMetrics:
    """Bounded-memory metrics; stale gaps never count as observed motion."""

    def __init__(self, limits, linear_deadband=.025, angular_deadband=.08):
        self.limits = limits
        self.linear_deadband = linear_deadband
        self.angular_deadband = angular_deadband
        self.streams = {}
        self.lock = Lock()

    def _advance(self, stream, now):
        previous = stream.get('_previous')
        if previous:
            stamp, linear, angular, blocked = previous
            if now < stamp:
                return
            elapsed = now - stamp
            if 0 <= elapsed <= .5:
                stream['sampled_seconds'] += elapsed
                stream['linear_distance_integral'] += abs(linear) * elapsed
                stream['angular_distance_integral'] += abs(angular) * elapsed
                stream['signed_angular_integral'] += angular * elapsed
                for key, active in (
                    ('translation_seconds', abs(linear) >= .005),
                    ('rotation_only_seconds', abs(linear) < .005 and abs(angular) >= .005),
                    ('stopped_seconds', abs(linear) < .005 and abs(angular) < .005),
                    ('below_linear_deadband_seconds', 1e-6 < abs(linear) < self.linear_deadband),
                    ('below_angular_deadband_seconds', 1e-6 < abs(angular) < self.angular_deadband),
                    ('translation_commanded_but_stationary_seconds', blocked),
                    ('positive_angular_seconds', angular >= .02),
                    ('negative_angular_seconds', angular <= -.02),
                ):
                    if active:
                        stream[key] += elapsed
            stream['_previous'] = (now, linear, angular, blocked)

    def observe(self, source, stamp, linear, angular):
        if not all(math.isfinite(v) for v in (stamp, linear, angular)):
            raise ValueError('Motion metric samples must be finite')
        with self.lock:
            stream = self.streams.setdefault(source, dict(samples=0, sampled_seconds=0.,
                peak_linear=0., peak_angular=0., cap_violations=0,
                linear_distance_integral=0., angular_distance_integral=0., signed_angular_integral=0.,
                translation_seconds=0., rotation_only_seconds=0., stopped_seconds=0.,
                positive_angular_seconds=0., negative_angular_seconds=0., angular_sign_changes=0,
                below_linear_deadband_seconds=0., below_angular_deadband_seconds=0.,
                translation_commanded_but_stationary_seconds=0.))
            previous = stream.get('_previous')
            if previous and stamp < previous[0]:
                raise ValueError('Motion metric timestamps must be monotonic')
            self._advance(stream, stamp)
            if abs(angular) >= .02:
                sign = 1 if angular > 0 else -1
                if stream.get('_angular_sign', sign) != sign:
                    stream['angular_sign_changes'] += 1
                stream['_angular_sign'] = sign
            blocked = False
            final = self.streams.get('final', {}).get('_previous')
            if source == 'odometry' and final and 0 <= stamp - final[0] <= .5:
                blocked = abs(final[1]) >= .005 and abs(linear) < .005
            stream['_previous'] = (stamp, linear, angular, blocked)
            stream['samples'] += 1
            stream['peak_linear'] = max(stream['peak_linear'], abs(linear))
            stream['peak_angular'] = max(stream['peak_angular'], abs(angular))
            if abs(linear) > self.limits['max_linear'] + 1e-6 or abs(angular) > self.limits['max_angular'] + 1e-6:
                stream['cap_violations'] += 1

    def summary(self, now):
        with self.lock:
            result = {}
            for name, stream in self.streams.items():
                self._advance(stream, now)
                row = {key: value for key, value in stream.items() if not key.startswith('_')}
                elapsed = row['sampled_seconds']
                row['mean_abs_linear'] = row['linear_distance_integral'] / elapsed if elapsed else None
                row['mean_abs_angular'] = row['angular_distance_integral'] / elapsed if elapsed else None
                row['peak_linear_fraction_of_cap'] = row['peak_linear'] / self.limits['max_linear']
                result[name] = row
            final = result.get('final', {})
            return dict(streams=result, complete=all(result.get(name, {}).get('samples', 0) > 0 for name in ('raw', 'final', 'odometry')),
                final_caps_respected=bool(final.get('samples')) and final.get('cap_violations') == 0,
                deadband_reference=dict(linear=self.linear_deadband, angular=self.angular_deadband),
                deadband_is_simulated_reference=True)
