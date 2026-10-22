"""Bounded motion over a complete, fresh odometry window, without ROS imports."""
from collections import deque
import math
from .core import angle_difference


class StationaryWindow:
    def __init__(self, seconds=2.0, linear_limit=.035, angular_limit=.10,
                 position_limit=.05, heading_limit=.12, max_gap=.25):
        values=(seconds,linear_limit,angular_limit,position_limit,heading_limit,max_gap)
        if not all(math.isfinite(v) and v>0 for v in values):
            raise ValueError('Stationary window limits must be finite and positive')
        self.seconds,self.linear_limit,self.angular_limit=seconds,linear_limit,angular_limit
        self.position_limit,self.heading_limit,self.max_gap=position_limit,heading_limit,max_gap
        self.samples=deque()

    def reset(self):
        self.samples.clear()

    def add(self, stamp, x, y, yaw, linear, angular, frame):
        if (not all(math.isfinite(v) for v in (stamp,x,y,yaw,linear,angular))
                or not frame):
            self.reset()
            return
        if self.samples:
            previous=self.samples[-1]
            if stamp<previous[0] or stamp-previous[0]>self.max_gap or frame!=previous[6]:
                self.reset()
            elif stamp==previous[0]:return  # Repeated messages cannot extend a hold.
            else:
                yaw=previous[3]+angle_difference(yaw,previous[3])
        self.samples.append((stamp,x,y,yaw,abs(linear),abs(angular),frame))
        # Retain one sample before the window boundary for time integration.
        cutoff=stamp-self.seconds
        while len(self.samples)>2 and self.samples[1][0]<=cutoff:
            self.samples.popleft()

    def evaluate(self, now):
        result=dict(verified=False,duration=0.,samples=len(self.samples),
                    linear_mean=None,angular_mean=None,position_span=None,heading_span=None,
                    reason='Collecting a complete stationary window',
                    limits=dict(seconds=self.seconds,linear_mean=self.linear_limit,
                                angular_mean=self.angular_limit,position_span=self.position_limit,
                                heading_span=self.heading_limit))
        if not self.samples:return result
        if not math.isfinite(now) or not 0<=now-self.samples[-1][0]<=self.max_gap:
            result['reason']='Stationary odometry is stale'
            return result
        duration=self.samples[-1][0]-self.samples[0][0]
        result['duration']=min(duration,self.seconds)
        if duration+1e-7<self.seconds or len(self.samples)<3:return result
        cutoff=self.samples[-1][0]-self.seconds
        samples=list(self.samples)
        # Clip/interpolate the first segment to exactly the requested window.
        if samples[0][0]<cutoff:
            a,b=samples[:2]
            f=(cutoff-a[0])/(b[0]-a[0])
            samples[0]=(cutoff,*(a[i]+f*(b[i]-a[i]) for i in range(1,6)),a[6])
        means=[sum((b[0]-a[0])*(a[i]+b[i])*.5 for a,b in zip(samples,samples[1:]))/self.seconds
               for i in (4,5)]
        dx=max(s[1] for s in samples)-min(s[1] for s in samples)
        dy=max(s[2] for s in samples)-min(s[2] for s in samples)
        position_span=math.hypot(dx,dy)
        heading_span=max(s[3] for s in samples)-min(s[3] for s in samples)
        result.update(linear_mean=means[0],angular_mean=means[1],position_span=position_span,
                      heading_span=heading_span)
        checks=[(means[0]<=self.linear_limit,'Mean linear speed is too high'),
                (means[1]<=self.angular_limit,'Mean angular speed is too high'),
                (position_span<=self.position_limit,'Position is still drifting'),
                (heading_span<=self.heading_limit,'Heading is still drifting')]
        result['reason']=next((reason for passed,reason in checks if not passed),'Stationary window verified')
        result['verified']=all(passed for passed,_ in checks)
        return result
