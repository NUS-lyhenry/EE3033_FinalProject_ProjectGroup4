"""Detect stalled navigation using continuous odometry, independently of Nav2.

Heading changes alone cannot keep a translation goal alive indefinitely. No
motor commands or obstacle-clearing actions are issued by this module.
"""
from collections import deque
import math
from .core import angle_difference


class NavigationProgress:
    def __init__(self, window=12., turn_grace=18.):
        self.window, self.turn_grace = window, turn_grace
        self.reset()

    def reset(self, now=None):
        self.started = now
        self.phase = None
        self.phase_started = None
        self.samples = deque()
        self.reason = None

    def check(self, now, odom, distance, heading_error, rotation_only=False,
              xy_tolerance=.10, heading_required=True):
        if odom is None or not all(math.isfinite(v) for v in (*odom, distance, heading_error)):
            return None  # sensor freshness is handled by the owning node
        if self.started is None:
            self.started = now
        # Arrival alignment gets its own full observation window. Reusing the
        # preceding journey would mistake its final few centering seconds for
        # twelve seconds of failed heading convergence.
        if rotation_only:
            phase = 'heading'
        elif self.phase == 'heading':
            phase = 'translation' if distance > xy_tolerance+.05 else 'heading'
        else:
            phase = 'heading' if distance <= xy_tolerance else 'translation'
        if phase != self.phase:
            self.phase, self.phase_started = phase, now
            self.samples.clear()
            self.reason = None
        self.samples.append((now, *odom, abs(heading_error)))
        while len(self.samples)>1 and self.samples[1][0] <= now-self.window:
            self.samples.popleft()
        first = self.samples[0]
        if now-first[0] < self.window:
            return None
        if self.phase == 'heading':
            if not heading_required or abs(heading_error) <= .025:
                # Hysteresis keeps the phase stable but must not turn its exit
                # band into a looser arrival tolerance. If balancing drift left
                # XY outside tolerance, completion still needs real movement.
                if (not rotation_only and distance > xy_tolerance
                        and math.dist(odom[:2],first[1:3]) < .02):
                    self.reason = 'Arrival position is not converging'
                    return self.reason
                return None
            improving = first[4]-abs(heading_error) >= .05
            if improving:
                return None
            self.reason = 'Final heading is not converging'
        else:
            displacement = math.dist(odom[:2], first[1:3])
            if displacement >= .08:
                return None
            turning = abs(angle_difference(odom[2], first[3])) >= .12
            if turning and now-self.phase_started < self.turn_grace:
                return None
            self.reason = 'No translation progress (blocked, turning in place, or oscillating)'
        return self.reason
