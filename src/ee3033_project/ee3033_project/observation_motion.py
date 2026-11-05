"""Capture-time motion windows for a balancing camera, without ROS imports."""
from collections import deque
import math
from .stationary import StationaryWindow


class ObservationMotion:
    def __init__(self, seconds=.6, linear_limit=.05, angular_limit=.12,
                 position_limit=.04, heading_limit=.10, max_gap=.25):
        self.settings = (seconds, linear_limit, angular_limit, position_limit, heading_limit, max_gap)
        StationaryWindow(*self.settings)  # Validate the same bounds used at evaluation.
        self.history = deque()

    def add(self, stamp, x, y, yaw, linear, angular, frame):
        sample = (stamp, x, y, yaw, linear, angular, frame)
        if (not all(math.isfinite(v) for v in sample[:6]) or not frame
                or (self.history and (stamp < self.history[-1][0] or frame != self.history[-1][6]))):
            self.history.clear()
            return
        if self.history and stamp == self.history[-1][0]:
            return
        self.history.append(sample)
        while self.history and stamp-self.history[0][0] > 6.:
            self.history.popleft()

    def evaluate(self, stamp):
        window = StationaryWindow(*self.settings)
        for sample in self.history:
            # The pose/motion belongs to exposure time, not inference completion.
            if stamp-2*self.settings[0] <= sample[0] <= stamp:
                window.add(*sample)
        return window.evaluate(stamp)

    def uncertainty_at(self, stamp, distance):
        """Capture-time motion adds error; a complete stationary window is optional."""
        past=[p for p in self.history if 0<=stamp-p[0]<=.25]
        if not past:return dict(sigma=.05,verified=False,reason='capture_motion_unavailable')
        sample=max(past,key=lambda p:p[0])
        return dict(sigma=math.hypot(.1*abs(sample[4]),.1*abs(distance*sample[5])),
            verified=True,stamp=sample[0],linear=sample[4],angular=sample[5],
            reason='capture_motion_uncertainty')
