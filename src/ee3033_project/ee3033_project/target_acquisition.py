"""Bounded single-target association and map-safe observation goals."""
import math
import time
from .core import angle_difference
from .near_view import requires_reframing, retreat_goal
from .visibility import observation_visibility
from .motion_safety import polyline_map_clearance


class SingleTarget:
    def __init__(self, max_steps=5, timeout=45., enable_ab_maneuver=False):
        self.max_steps, self.timeout = max_steps, timeout
        self.enable_ab_maneuver = bool(enable_ab_maneuver)
        self.reset()

    def reset(self):
        self.candidate = None
        self.frames = 0
        self.started = None
        self.steps = 0
        self.visited = set()
        self.locked = False
        self.positions = []
        self.active_seconds = self.travel_seconds = 0.
        self.clock_stamp = None
        self.phase = 'A: centering and sampling'
        self.recovery_origin = None
        self.recovery_step = 0
        self.recovery_started = None
        self.turn_sites = []
        self.last_goal_reason = None
        self.last_goal_action = None
        self.reframe_positions = []
        self.reframe_attempts = 0
        self.reframe_distance = 0.

    def turns_at(self, pose):
        site=next((s for s in self.turn_sites if math.dist(s['origin'],pose[:2])<.18),None)
        return site['turns'] if site else 0

    def turn_goal(self, pose, yaw, reason):
        # Balance jitter and intermittent detection must not spend every motion
        # target on another almost identical in-place correction.
        if abs(angle_difference(yaw,pose[2]))<=math.radians(2.) or self.turns_at(pose)>=2:
            return None
        site=next((s for s in self.turn_sites if math.dist(s['origin'],pose[:2])<.18),None)
        if site is None:
            site=dict(origin=tuple(pose[:2]),turns=0);self.turn_sites.append(site)
        site['turns']+=1
        self.last_goal_reason=reason
        return (*pose[:2],yaw)

    def advance_clock(self, now, working=False, traveling=False):
        if self.clock_stamp is not None:
            elapsed=max(0.,now-self.clock_stamp)
            # Charge the interval to the previous state, not the new state.
            if self.clock_working:self.active_seconds+=elapsed
            if self.clock_traveling:self.travel_seconds+=elapsed
        self.clock_stamp=now
        self.clock_working,self.clock_traveling=working,traveling

    def recovery_goal(self, pose, now):
        """A bounded local scan about the last world bearing; never global search."""
        d=self.candidate
        if not d:return None
        if self.recovery_started is None:
            self.recovery_started=self.active_seconds
            self.recovery_origin=d.get('bearing', d.get('steering_bearing',pose[2]))
            self.recovery_step=0
        self.phase='Locally reacquiring locked target'
        if self.active_seconds-self.recovery_started>=10. or self.turns_at(pose)>=2:
            return None
        offsets=(0.,math.radians(15.),-math.radians(15.))
        while self.recovery_step<len(offsets):
            yaw=self.recovery_origin+offsets[self.recovery_step]
            self.recovery_step+=1
            goal=self.turn_goal(pose,yaw,'Local reacquisition turn')
            if goal is not None:return goal
        return None

    def update(self, detections, now, armed_at):
        choices = []
        for d in detections:
            try:
                values = [d['stamp'], d['confidence']]
                if (all(math.isfinite(v) for v in values)
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
                    if math.hypot(d['x']-previous['x'], d['y']-previous['y']) < .4:
                        return True
                    if not self.locked:
                        return False
                if 'bearing' in d and 'bearing' in previous:
                    if abs(angle_difference(d['bearing'], previous['bearing'])) < .25:
                        return True
                    # A lateral baseline changes map bearing. Check the ray
                    # intersection instead of rejecting all useful parallax.
                    if self.locked and 'origin' in d and 'origin' in previous:
                        import numpy as np
                        a, b = d['bearing'], previous['bearing']
                        rays = np.array([[math.cos(a), -math.cos(b)],
                                         [math.sin(a), -math.sin(b)]])
                        if abs(np.linalg.det(rays)) > .05:
                            distances = np.linalg.solve(rays, np.array(previous['origin'])-d['origin'])
                            return bool(np.all((distances > .1) & (distances < 3.)))
                    return False
                if 'box' in d and 'box' in previous:
                    a, b = d['box'], previous['box']
                    overlap = max(0., min(a[2], b[2])-max(a[0], b[0]))*max(0., min(a[3], b[3])-max(a[1], b[1]))
                    return overlap > .25*min((a[2]-a[0])*(a[3]-a[1]), (b[2]-b[0])*(b[3]-b[1]))
                return False
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
        self.recovery_started = None
        if self.frames >= 2:
            self.locked = True
        return self.locked

    def expired(self, now):
        # Steps are evidence/diagnostics, never a reason to abandon a locked
        # target before its actual localization-work time has been used.
        return self.started is not None and self.active_seconds >= self.timeout

    def observation_goal(self, planner, pose, deadline=None, cancel=None, force_baseline=False, allowed=None):
        """Collect parallax; bound local turns and retain a depth-free fallback."""
        d = self.candidate
        if d is None:
            return None
        self.last_goal_action=None
        near=requires_reframing(d) or ('bearing' not in d and 'steering_bearing' in d)
        if near:
            # Framing recovery is not an A/B/C capture station: a cropped
            # view can contain no usable measurement at all. Keep its bounded
            # route history separate so two retreats still permit valid A/B.
            if not any(math.dist(pose[:2],p)<.12 for p in self.reframe_positions):
                self.reframe_positions.append(tuple(pose[:2]))
            if self.reframe_attempts<2 and self.reframe_distance<.6-1e-6:
                goal,reason=retreat_goal(planner,pose,d,self.visited,self.reframe_positions,
                    allowed,deadline,cancel)
                self.last_goal_reason=reason
                if goal is not None:
                    self.reframe_attempts+=1
                    self.reframe_distance+=math.dist(goal[:2],pose[:2])
                    self.visited.add(planner.world_to_cell(*goal[:2]))
                    self.last_goal_action='retreat'
                    return goal
            else:self.last_goal_reason='Bounded near-field retreat budget used'
            # A still-invalid cropped view must not consume a capture site.
            # A valid independent horizontal bearing may proceed even when
            # floor contact remains vertically clipped or below minimum range.
            if not d.get('bearing_valid',False):
                return None
        if 'bearing' not in d:
            return None
        camera_yaw = planner.camera_model.yaw if planner.camera_model else 0.
        desired = d['bearing']-camera_yaw
        if not force_baseline and abs(angle_difference(desired, pose[2])) > math.radians(2.) and self.turns_at(pose)<2:
            key = ('center', round(desired, 2))
            if key not in self.visited:
                self.visited.add(key)
                goal=self.turn_goal(pose,desired,'Center locked target')
                if goal is not None:return goal
            # A repeated correction is not a reason to abandon the baseline.
        if not self.enable_ab_maneuver:
            return self.approach_goal(planner, pose, deadline, cancel, allowed)
        # Retain up to three distinct capture positions across local moves.
        if not any(math.dist(pose[:2], p) < .12 for p in self.positions):
            if len(self.positions)>=3:
                self.last_goal_reason='Three observation positions already used'
                return None
            self.positions.append(tuple(pose[:2]))
        if len(self.positions) >= 3:
            return None
        if (d.get('projection_valid') is False or d.get('range_state') in ('too_near','too_far')
                or not all(k in d and math.isfinite(d[k]) for k in ('x', 'y'))):
            return self.bearing_baseline(planner,pose,deadline,cancel,allowed)
        tx, ty = d['x'], d['y']
        distances, _ = planner.travel_field(*pose[:2], deadline=deadline, cancel=cancel)
        goals = []
        base_angle = math.atan2(pose[1]-ty, pose[0]-tx)
        preferred = .45 if d.get('close_view') is False else .6
        for radius in dict.fromkeys((preferred, .6, .8, 1.)):
            for offset in (0., -.45, .45, -.9, .9):
                if (deadline is not None and time.monotonic() >= deadline) or (cancel is not None and cancel.is_set()):
                    return None
                a = base_angle+offset
                x, y = tx+radius*math.cos(a), ty+radius*math.sin(a)
                cell = planner.world_to_cell(x, y)
                if (not planner.in_bounds(*cell) or not planner.safe[cell]
                        or not math.isfinite(distances[cell]) or cell in self.visited
                        or (allowed is not None and allowed(x,y) is False)):
                    continue
                x, y = planner.cell_to_world(*cell)
                if allowed is not None and allowed(x,y) is False:
                    continue
                yaw = math.atan2(ty-y, tx-x)
                if planner.camera_model is not None:
                    yaw -= planner.camera_model.yaw
                    dx, dy = tx-x, ty-y
                    relative = [[math.cos(yaw)*dx+math.sin(yaw)*dy,
                                 -math.sin(yaw)*dx+math.cos(yaw)*dy]]
                    if not planner.camera_model.contains_floor(relative)[0]:
                        continue
                visibility=observation_visibility(planner,(x,y,yaw),target=(tx,ty),
                    camera_model=planner.camera_model,require_floor=planner.camera_model is not None,
                    deadline=deadline,cancel=cancel)
                if not visibility.visible:continue
                # A goal already inside Nav2's arrival tolerance changes no
                # observation. Do not spend the approach budget repeating it.
                if math.hypot(x-pose[0], y-pose[1]) < .12 and abs(angle_difference(yaw, pose[2])) < .2:
                    continue
                if any(math.dist((x, y), p) < .25 for p in self.positions):
                    continue
                crossing = abs(angle_difference(math.atan2(ty-y, tx-x), d['bearing']))
                parallax_penalty = 2. if crossing < math.radians(15.) else 0.
                goals.append((float(distances[cell])+abs(offset)*.2+2*abs(radius-preferred)+parallax_penalty,
                              cell, (x, y, yaw)))
        if not goals:
            # Invalid/range-biased floor depth must not prevent two-ray ranging.
            return self.bearing_baseline(planner,pose,deadline,cancel,allowed)
        _, cell, goal = min(goals)
        self.visited.add(cell)
        self.last_goal_reason='Change observation position for parallax'
        return goal

    def approach_goal(self, planner, pose, deadline=None, cancel=None, allowed=None):
        """Try a short, continuously checked advance toward the observed ray.

        Image bearings select an action, never an invented range measurement.
        The normal Nav2 route proxy and final gate still check the action.
        """
        d=self.candidate
        if d is None or 'bearing' not in d or 'origin' not in d:
            self.last_goal_reason='A fresh target direction is required before approaching'
            return None
        if not any(math.dist(pose[:2],p)<.04 for p in self.positions):
            self.positions.append(tuple(pose[:2]))
        camera=getattr(planner,'camera_model',None)
        bearing=d['bearing']
        yaw=bearing-(camera.yaw if camera else 0.)
        for step in (.10,.075,.05):
            if (deadline is not None and time.monotonic()>=deadline) or (cancel is not None and cancel.is_set()):
                return None
            x,y=pose[0]+step*math.cos(bearing),pose[1]+step*math.sin(bearing)
            cell=planner.world_to_cell(x,y)
            if (not planner.in_bounds(*cell) or not planner.safe[cell] or cell in self.visited
                    or any(math.dist((x,y),p)<.04 for p in self.positions)):
                continue
            # Check the whole physical envelope, not just the destination cell.
            check=polyline_map_clearance(planner.grid,planner.resolution,planner.origin,[pose[:2],(x,y)])
            if not check.allowed:continue
            count=max(2,math.ceil(step/max(.005,planner.resolution/2))+1)
            if allowed is not None and any(allowed(pose[0]+i/(count-1)*(x-pose[0]),
                    pose[1]+i/(count-1)*(y-pose[1])) is not True for i in range(count)):
                continue
            if all(k in d for k in ('x','y')) and d.get('projection_valid') is True:
                visible=observation_visibility(planner,(x,y,yaw),target=(d['x'],d['y']),
                    camera_model=camera,require_floor=camera is not None,deadline=deadline,cancel=cancel)
            else:
                visible=observation_visibility(planner,(x,y,yaw),source_origin=d['origin'],
                    bearing=bearing,camera_model=camera,require_range=False,deadline=deadline,cancel=cancel)
            if not visible.visible:continue
            self.visited.add(cell)
            self.last_goal_action='approach'
            self.last_goal_reason='Advance toward locked target and resample'
            return (x,y,yaw)
        self.last_goal_reason='No safe visible short approach; continue observing'
        return None

    def bearing_baseline(self, planner, pose, deadline=None, cancel=None, allowed=None):
        """Rank safe lateral baselines by conservative parallax and path cost.

        The target has unknown range. Use multiple depths along the *measured*
        source ray for scoring; a midpoint only aims the camera and never becomes
        a coordinate measurement. Keep the existing map/camera visibility gate.
        """
        d = self.candidate
        if d is None or 'bearing' not in d:
            return None
        source = d.get('origin')
        if source is None:
            self.last_goal_reason = 'Baseline requires the last image camera origin'
            return None
        distances, _ = planner.travel_field(*pose[:2], deadline=deadline, cancel=cancel)
        camera = planner.camera_model
        camera_yaw = camera.yaw if camera else 0.
        near, far = (camera.min_range, camera.max_range) if camera else (.3, 2.)
        bearing = d['bearing']
        mid = (near + far) / 2.
        tx = source[0] + mid * math.cos(bearing)
        ty = source[1] + mid * math.sin(bearing)
        options = []
        for radius in (.4, -.4, .6, -.6, .8, -.8, -.3):
            if cancel is not None and cancel.is_set():
                return None
            if deadline is not None and time.monotonic() >= deadline:
                break
            angle = bearing + (math.pi / 2 if radius != -.3 else 0.)
            x, y = pose[0] + radius * math.cos(angle), pose[1] + radius * math.sin(angle)
            cell = planner.world_to_cell(x, y)
            if (not planner.in_bounds(*cell) or not planner.safe[cell]
                    or not math.isfinite(distances[cell]) or cell in self.visited
                    or any(math.dist((x, y), p) < .25 for p in self.positions)
                    or (allowed is not None and allowed(x, y) is False)):
                continue
            yaw = math.atan2(ty - y, tx - x) - camera_yaw
            visibility = observation_visibility(planner, (x, y, yaw), source_origin=source,
                bearing=bearing, camera_model=camera, deadline=deadline, cancel=cancel)
            if cancel is not None and cancel.is_set():
                return None
            if deadline is not None and time.monotonic() >= deadline:
                break
            if not visibility.visible:
                self.last_goal_reason = 'Baseline visibility rejected: ' + visibility.reason
                continue
            # At longer unknown ranges the same baseline gives weaker parallax.
            # Reward the worst of near/mid/far hypotheses, not just a fabricated
            # target at the range midpoint.
            camera_origin = visibility.camera_origin or (x, y)
            parallax = []
            for depth in (near, mid, far):
                px = source[0] + depth * math.cos(bearing)
                py = source[1] + depth * math.sin(bearing)
                new_bearing = math.atan2(py - camera_origin[1], px - camera_origin[0])
                parallax.append(abs(math.sin(angle_difference(new_bearing, bearing))))
            turn = abs(angle_difference(yaw, pose[2]))
            path_length = float(distances[cell])
            score = (2.0 * min(parallax) + .25 * visibility.support
                     + .15 * visibility.range_support - .10 * path_length - .025 * turn)
            options.append((score, -path_length, cell, (x, y, yaw), min(parallax), radius))
        # A later visibility query can exhaust Mission's 0.2 s callback budget.
        # Keep earlier fully checked choices; cancellation still discards all.
        if cancel is not None and cancel.is_set():
            return None
        if not options:
            self.last_goal_reason = 'No map-safe connected baseline candidate'
            return None
        _, _, cell, goal, _, radius = max(options)
        self.visited.add(cell)
        self.last_goal_reason = ('Depth-free lateral baseline' if radius != -.3
                                 else 'Reframe by moving away')
        return goal
