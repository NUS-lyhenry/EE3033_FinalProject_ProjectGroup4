"""Bounded calibration movements. The existing lidar velocity gate stays final."""
import math


class CalibrationMotion:
    def __init__(self):
        self.mode = 'idle'
        self.reason = 'Stopped'
        self.generation = 0
        self.lease = -math.inf
        self.began = 0.
        self.start_pose = None
        self.direction = 0.
        self.aligned = 0

    def stop(self, reason='Stopped'):
        self.mode, self.reason = 'idle', reason
        self.generation += 1

    def start(self, mode, now, pose, generation):
        if generation != self.generation:
            raise ValueError('Movement context changed; refresh before retrying')
        if self.mode != 'idle':
            raise ValueError('Wait for the current movement to stop')
        if mode not in ('center', 'forward', 'backward'):
            raise ValueError('Select centre, forward, or backward')
        self.mode, self.reason = mode, 'Moving: '+mode
        self.began, self.lease, self.start_pose = now, now, tuple(pose)
        self.direction = 1. if mode == 'forward' else -1.
        self.aligned = 0

    def renew(self, now, generation):
        if generation == self.generation and self.mode != 'idle':
            self.lease = now

    def tick(self, now, pose, detection, healthy):
        if self.mode == 'idle':
            return 0., 0.
        if not healthy or now-self.lease > .6:
            self.stop('Stopped: sensor, control ownership, or browser heartbeat lost')
            return 0., 0.
        distance = math.hypot(pose[0]-self.start_pose[0], pose[1]-self.start_pose[1])
        turn = abs(math.atan2(math.sin(pose[2]-self.start_pose[2]), math.cos(pose[2]-self.start_pose[2])))
        if now-self.began > (8. if self.mode == 'center' else 5.) or distance >= .10 or turn > .45:
            self.stop('Movement limit reached; measure distance again')
            return 0., 0.
        if detection is None:
            self.stop('Stopped: target lost or multiple bottles visible')
            return 0., 0.
        if self.mode == 'center':
            error = detection['error']
            if abs(error) < .018:
                self.aligned += 1
                if self.aligned >= 6:
                    self.stop('Bottle centred in image; remeasure forward and left offsets')
                return 0., 0.
            self.aligned = 0
            # Image right is negative ROS yaw. Minimum speed avoids stalls; both
            # requested and final motion remain capped by the velocity gate.
            return 0., -math.copysign(min(.15, max(.055, abs(error)*.6)), error)
        return self.direction*.035, 0.
