"""Bounded single-target association and map-safe observation goals."""
import math
import time
from .core import angle_difference


class SingleTarget:
    def __init__(self, max_steps=3, timeout=15.):
        self.max_steps, self.timeout = max_steps, timeout
        self.reset()

    def reset(self):
        self.candidate = None
        self.frames = 0
        self.started = None
        self.steps = 0
        self.visited = set()

    def update(self, detections, now, armed_at):
        choices = []
        for d in detections:
            try:
                values = [d['stamp'], d['confidence'], d['bearing'], *d['origin']]
                if (len(d['origin']) == 2 and all(math.isfinite(v) for v in values)
                        and armed_at <= d['stamp'] <= now and now-d['stamp'] <= 1.5
                        and .60 <= d['confidence'] <= 1.):
                    choices.append(d)
            except (KeyError, TypeError, ValueError):
                continue
        if not choices:
            if self.candidate and now-self.candidate['stamp'] > 1.2:
                self.frames = 0
            return False
        if self.candidate:
            previous = self.candidate
            def same(d):
                if d.get('label') != previous.get('label'):
                    return False
                if all(k in d and k in previous for k in ('x', 'y')):
                    return math.hypot(d['x']-previous['x'], d['y']-previous['y']) < .4
                return abs(angle_difference(d['bearing'], previous['bearing'])) < .25
            matched = [d for d in choices if same(d)]
            # During acquisition, do not silently switch to another detection.
            if self.started is not None and not matched:
                return False
            if not matched:
                self.frames = 0
                self.candidate = None
            choices = matched or choices
        d = max(choices, key=lambda d: d['confidence'])
        if self.candidate and d['stamp'] <= self.candidate['stamp']:
            return False
        consistent = bool(self.candidate and d['stamp']-self.candidate['stamp'] <= 1.2 and same(d))
        self.frames = self.frames+1 if consistent else 1
        self.candidate = d
        return self.frames >= 2

    def expired(self, now):
        return self.started is not None and (now-self.started >= self.timeout or self.steps >= self.max_steps)

    def observation_goal(self, planner, pose, deadline=None, cancel=None):
        """Prefer a safe .6 m standoff; bearing-only detections only turn in place."""
        d = self.candidate
        if d is None:
            return None
        if not all(k in d and math.isfinite(d[k]) for k in ('x', 'y')):
            key = ('turn', round(d['bearing'], 1))
            if key in self.visited:
                return None
            self.visited.add(key)
            return (pose[0], pose[1], d['bearing']-(planner.camera_model.yaw if planner.camera_model else 0.))
        tx, ty = d['x'], d['y']
        distances, _ = planner.travel_field(*pose[:2], deadline=deadline, cancel=cancel)
        goals = []
        base_angle = math.atan2(pose[1]-ty, pose[0]-tx)
        for radius in (.6, .8, 1.):
            for offset in (0., -.45, .45, -.9, .9):
                if (deadline is not None and time.monotonic() >= deadline) or (cancel is not None and cancel.is_set()):
                    return None
                a = base_angle+offset
                x, y = tx+radius*math.cos(a), ty+radius*math.sin(a)
                cell = planner.world_to_cell(x, y)
                if (not planner.in_bounds(*cell) or not planner.safe[cell]
                        or not math.isfinite(distances[cell]) or cell in self.visited):
                    continue
                x, y = planner.cell_to_world(*cell)
                # Reject an approach if a mapped obstacle hides the target.
                target = planner.world_to_cell(tx, ty)
                count = max(abs(target[0]-cell[0]), abs(target[1]-cell[1]), 1)
                line = [(round(cell[0]+(target[0]-cell[0])*i/count),
                         round(cell[1]+(target[1]-cell[1])*i/count)) for i in range(count+1)]
                if any(not planner.in_bounds(*p) or not planner.free[p] for p in line):
                    continue
                yaw = math.atan2(ty-y, tx-x)
                if planner.camera_model is not None:
                    yaw -= planner.camera_model.yaw
                    dx, dy = tx-x, ty-y
                    relative = [[math.cos(yaw)*dx+math.sin(yaw)*dy,
                                 -math.sin(yaw)*dx+math.cos(yaw)*dy]]
                    if not planner.camera_model.contains_floor(relative)[0]:
                        continue
                goals.append((float(distances[cell])+abs(offset)*.2+abs(radius-.6),
                              cell, (x, y, yaw)))
        if not goals:
            return None
        _, cell, goal = min(goals)
        self.visited.add(cell)
        return goal
