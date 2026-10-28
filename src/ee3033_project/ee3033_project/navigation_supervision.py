"""ROS-independent navigation progress and bounded recovery policy.

Only odometry translation can demonstrate progress along a route. Heading
convergence is observed separately, and cannot restart a translation deadline.
"""
from collections import deque
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ProgressWatch:
    def __init__(self, translation_window=12., translation_distance=.08,
                 heading_window=3., heading_improvement=.05):
        self.translation_window = translation_window
        self.translation_distance = translation_distance
        self.heading_window = heading_window
        self.heading_improvement = heading_improvement
        self.reset()

    def reset(self):
        self.translation = deque()
        self.heading = deque()
        self.heading_reference = None
        self.phase = None

    def update(self, now, odom, heading_error, *, aligning=False, arrived=False,
               reference=None):
        """Return a failure reason, or None; all arguments are observed values."""
        if not all(math.isfinite(v) for v in (now, *odom, heading_error)):
            return 'Invalid navigation progress observation'
        self.translation.append((now, odom[0], odom[1]))
        while len(self.translation)>1 and self.translation[1][0] <= now-self.translation_window:
            self.translation.popleft()
        phase = 'final_heading' if arrived else 'heading' if aligning else 'translation'
        changed = (reference is not None and self.heading_reference is not None
                   and abs(wrap(reference-self.heading_reference))>.3)
        if self.phase != phase or changed:
            self.heading.clear()
            self.heading_reference = reference
            self.phase = phase
        if aligning or arrived:
            self.heading.append((now, abs(heading_error)))
            while len(self.heading)>1 and self.heading[1][0] <= now-self.heading_window:
                self.heading.popleft()
            first = self.heading[0]
            if (now-first[0] >= self.heading_window and abs(heading_error)>.05
                    and first[1]-abs(heading_error)<self.heading_improvement):
                return 'Final heading is not converging' if arrived else 'Path heading is not converging'
        # A changing path/reference must never keep a translation goal alive.
        if not arrived:
            first = self.translation[0]
            if (now-first[0] >= self.translation_window
                    and math.hypot(odom[0]-first[1], odom[1]-first[2])<self.translation_distance):
                return 'No translation progress'
        return None


class RecoveryBudget:
    """A per-outer-goal budget; retries never reset the absolute deadline."""
    def __init__(self, now, timeout=60., maximum=2):
        if not math.isfinite(timeout) or timeout<=0 or maximum<0:
            raise ValueError('Invalid navigation budget')
        self.deadline = now+timeout
        self.maximum = maximum
        self.count = 0

    def expired(self, now):
        return now >= self.deadline

    def next(self, now, path_received):
        if self.expired(now) or self.count>=self.maximum:
            return None
        # The first retry refreshes planning only. Reversing without a route
        # would turn an unreachable goal into unnecessary physical motion.
        if self.count==1 and not path_received:
            return None
        self.count += 1
        return 'refresh' if self.count==1 else 'reverse'


def path_reference(path, pose, ahead=.18):
    """Project onto the closest segment, then advance by path arc length."""
    if len(path)<2:
        return None
    best=None
    for i,(a,b) in enumerate(zip(path,path[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1]
        length2=dx*dx+dy*dy
        if length2<=1e-12:
            continue
        t=max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/length2))
        p=(a[0]+t*dx,a[1]+t*dy)
        candidate=(math.hypot(p[0]-pose[0],p[1]-pose[1]),i,t,p)
        if best is None or candidate[0]<best[0]:best=candidate
    if best is None:return None
    _,i,t,p=best
    remaining=ahead
    for end in path[i+1:]:
        length=math.dist(p,end)
        if length>=remaining and length>0:
            f=remaining/length
            target=(p[0]+f*(end[0]-p[0]),p[1]+f*(end[1]-p[1]))
            break
        remaining-=length
        p=end
    else:target=path[-1]
    return math.atan2(target[1]-pose[1],target[0]-pose[0])
