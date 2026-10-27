"""Conservative camera visibility on a 2-D occupancy map, without ROS.

Robot reachability is deliberately a caller concern: a navigable route around a
wall does not make the target visible through that wall at the destination.
Occupied/unknown cells are treated as vertical occluders. This is conservative
for low obstacles and cannot model shelves, overhangs, glass, target height, or
new unmapped objects; it is not a substitute for the live camera or Nav2.
"""
from dataclasses import dataclass
import math
import time

import numpy as np

from .core import angle_difference


@dataclass(frozen=True)
class VisibilityResult:
    visible: bool
    reason: str
    support: float = 0.
    hypotheses: int = 0
    visible_hypotheses: int = 0
    camera_origin: object = None
    range_support: float = 0.

    def __bool__(self):
        return self.visible


def supercover_cells(start, end, resolution, origin, margin=0.):
    """All closed grid squares touched by the segment's Euclidean capsule.

    Exact segment/AABB intersection includes both neighbours at grid edges and
    all corner-touching cells. The nonzero visibility margin also includes
    cells whose square intersects the margin, rather than sampling ray centres.
    Returned (row, column) cells may be outside the map; callers must reject
    those rather than clipping them into apparent free space.
    """
    start,end=np.asarray(start,dtype=float),np.asarray(end,dtype=float)
    origin=np.asarray(origin,dtype=float)
    if (start.shape!=(2,) or end.shape!=(2,) or origin.shape!=(3,)
            or not np.isfinite([*start,*end,*origin,resolution,margin]).all()
            or resolution<=0 or margin<0):
        raise ValueError('Invalid line-of-sight geometry')
    c,s=math.cos(origin[2]),math.sin(origin[2])
    rotation=np.array([[c,s],[-s,c]])
    a=rotation@(start-origin[:2]);b=rotation@(end-origin[:2])
    delta=b-a
    # Include a neighbour when an endpoint lies exactly on its closed boundary.
    lo=np.floor((np.minimum(a,b)-margin)/resolution).astype(int)-1
    hi=np.floor((np.maximum(a,b)+margin)/resolution).astype(int)+1
    size=hi-lo+1
    if int(size[0])*int(size[1])>1_000_000:
        raise ValueError('Line-of-sight geometry exceeds grid budget')
    rows,cols=np.mgrid[lo[1]:hi[1]+1,lo[0]:hi[0]+1]
    lower=np.stack([cols*resolution,rows*resolution],axis=-1)
    upper=lower+resolution
    epsilon=1e-10*max(1.,resolution)
    enter=np.zeros(rows.shape);leave=np.ones(rows.shape)
    intersects=np.ones(rows.shape,dtype=bool)
    for axis in range(2):
        if abs(delta[axis])<=epsilon:
            intersects&=(a[axis]>=lower[...,axis]-epsilon)&(a[axis]<=upper[...,axis]+epsilon)
        else:
            t0=(lower[...,axis]-a[axis])/delta[axis]
            t1=(upper[...,axis]-a[axis])/delta[axis]
            enter=np.maximum(enter,np.minimum(t0,t1))
            leave=np.minimum(leave,np.maximum(t0,t1))
    intersects&=enter<=leave+epsilon
    # For non-intersecting squares, the shortest segment/rectangle distance is
    # attained by an endpoint against a square, or a square corner against the
    # segment. Keeping this exact avoids excessive square-grid dilation.
    distance2=np.minimum(
        np.sum(np.maximum(np.maximum(lower-a,a-upper),0.)**2,axis=-1),
        np.sum(np.maximum(np.maximum(lower-b,b-upper),0.)**2,axis=-1))
    length2=float(delta@delta)
    for x_high,y_high in ((False,False),(False,True),(True,False),(True,True)):
        corner=np.stack([upper[...,0] if x_high else lower[...,0],
                         upper[...,1] if y_high else lower[...,1]],axis=-1)
        along=np.zeros(rows.shape) if length2<=epsilon**2 else np.clip(
            np.sum((corner-a)*delta,axis=-1)/length2,0.,1.)
        closest=a+along[...,None]*delta
        distance2=np.minimum(distance2,np.sum((corner-closest)**2,axis=-1))
    selected=intersects|(distance2<=(margin+epsilon)**2)
    return frozenset(zip(rows[selected].tolist(),cols[selected].tolist()))


class _MapVisibility:
    def __init__(self,planner,margin):
        self.grid=np.asarray(planner.grid)
        self.resolution=float(planner.resolution)
        self.origin=tuple(planner.origin)
        self.margin=float(margin)
        if (self.grid.ndim!=2 or not self.grid.size
                or not np.isfinite(self.grid).all()
                or np.any((self.grid < -1)|(self.grid > 100))
                or not math.isfinite(self.margin) or self.margin<=0):
            raise ValueError('Visibility requires a valid map and nonzero LOS margin')

    def line(self,start,end):
        cells=supercover_cells(start,end,self.resolution,self.origin,self.margin)
        if any(r<0 or c<0 or r>=self.grid.shape[0] or c>=self.grid.shape[1] for r,c in cells):
            return False,'Camera line of sight leaves the known map'
        values=np.array([self.grid[r,c] for r,c in cells])
        if np.any(values<0):return False,'Unknown space blocks camera line of sight'
        if np.any(values>=25):return False,'Mapped obstacle blocks camera line of sight'
        return True,'Known-free camera line of sight'


def observation_visibility(planner,pose,target=None,*,source_origin=None,bearing=None,
                           camera_model=None,range_bounds=None,
                           bearing_sigma=math.radians(2.),los_margin=.04,
                           min_support=.60,require_floor=False,require_range=True,
                           deadline=None,cancel=None):
    """Check a candidate base pose against a trusted point or bearing support.

    A bearing has no known depth. Sample a bounded depth/angular distribution,
    conditioned on known-free rays from the last actual camera observation.
    Evaluate the *fraction* that the candidate could see, not one fabricated
    point at 1 m. The result exposes support counts, never a target estimate.
    ``range_bounds`` applies only to source hypotheses; source near/clipped
    detections need not satisfy floor projection or camera minimum range.

    Candidate horizontal FOV and range are checked using the actual mount XY
    offset/yaw. Vertical contact-point FOV is optional because vertically cropped
    targets can still supply valid horizontal bearings. ``require_floor=True``
    additionally asks the camera model to validate contact-point visibility.
    ``require_range=False`` is only for intermediate near-field reframing: it
    permits support below the reliable minimum but still enforces maximum range,
    field of view and occlusion. ``range_support`` stays separate so such a move
    cannot be mistaken for a valid distance measurement or a target estimate.
    Unknown cells and outside-map capsule cells always block visibility.
    """
    def stopped():
        return ((cancel is not None and cancel.is_set())
                or (deadline is not None and time.monotonic()>=deadline))

    try:
        geometry=_MapVisibility(planner,los_margin)
        pose=np.asarray(pose,dtype=float)
        camera=camera_model if camera_model is not None else getattr(planner,'camera_model',None)
        mount=np.asarray(camera.translation[:2],dtype=float) if camera is not None else np.zeros(2)
        mount_yaw=float(camera.yaw) if camera is not None else 0.
        near=float(camera.min_range) if camera is not None else .3
        far=float(camera.max_range) if camera is not None else 2.
        fov=float(camera.fov) if camera is not None else float(getattr(planner,'fov',1.35))
        if (pose.shape!=(3,) or mount.shape!=(2,)
                or not np.isfinite([*pose,*mount,mount_yaw,near,far,fov,min_support,bearing_sigma]).all()
                or not 0<near<far or not 0<fov<2*math.pi or not 0<min_support<=1
                or not 0<=bearing_sigma<=math.pi/4):
            raise ValueError('Invalid camera visibility geometry')
        c,s=math.cos(pose[2]),math.sin(pose[2])
        camera_xy=pose[:2]+np.array([c*mount[0]-s*mount[1],s*mount[0]+c*mount[1]])
        camera_heading=pose[2]+mount_yaw

        def candidate_sees(point):
            dx,dy=point-camera_xy
            distance=math.hypot(dx,dy)
            if (distance<=1e-10 or distance>far+1e-10
                    or (require_range and distance<near-1e-10)):
                return False,'Target support is outside candidate camera range'
            if abs(angle_difference(math.atan2(dy,dx),camera_heading))>fov/2+1e-10:
                return False,'Target support is outside candidate horizontal field of view'
            if require_floor:
                if camera is None or not hasattr(camera,'contains_floor'):
                    return False,'Floor visibility needs a valid camera model'
                delta=point-pose[:2]
                local=[[c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]]]
                if not camera.contains_floor(local)[0]:
                    return False,'Target contact is outside candidate floor field of view'
            return geometry.line(camera_xy,point)

        if stopped():return VisibilityResult(False,'Visibility evaluation canceled or timed out')
        if target is not None:
            point=np.asarray(target,dtype=float)
            if point.shape!=(2,) or not np.isfinite(point).all():
                raise ValueError('Invalid trusted target position')
            okay,reason=candidate_sees(point)
            within=near-1e-10<=np.linalg.norm(point-camera_xy)<=far+1e-10
            return VisibilityResult(okay,reason,float(okay),1,int(okay),tuple(camera_xy),float(okay and within))

        source=np.asarray(source_origin,dtype=float)
        bounds=np.asarray(range_bounds if range_bounds is not None else (near,far),dtype=float)
        if (source.shape!=(2,) or bounds.shape!=(2,) or bearing is None
                or not np.isfinite([*source,*bounds,bearing]).all()
                or not .05<=bounds[0]<bounds[1]<=5.):
            raise ValueError('A bearing needs an actual source origin and bounded depth distribution')
        depths=np.linspace(bounds[0],bounds[1],min(25,max(3,
            math.ceil((bounds[1]-bounds[0])/.15)+1)))
        angular_offsets=(0.,) if bearing_sigma==0 else (-2.,-1.,0.,1.,2.)
        source_support=[]
        for index,depth in enumerate(depths):
            for offset in angular_offsets:
                if stopped():return VisibilityResult(False,'Visibility evaluation canceled or timed out')
                theta=bearing+offset*bearing_sigma
                point=source+depth*np.array([math.cos(theta),math.sin(theta)])
                if geometry.line(source,point)[0]:
                    source_support.append((index,point,math.exp(-offset*offset/2)))
        if len({i for i,_,_ in source_support})<3:
            return VisibilityResult(False,'Insufficient known-visible depth hypotheses',
                                    hypotheses=len(source_support),camera_origin=tuple(camera_xy))
        total_weight=sum(weight for _,_,weight in source_support)
        visible_weight=0.;range_weight=0.;visible_count=0;visible_depths=set()
        reasons={}
        for index,point,weight in source_support:
            if stopped():return VisibilityResult(False,'Visibility evaluation canceled or timed out')
            okay,reason=candidate_sees(point)
            reasons[reason]=reasons.get(reason,0)+1
            if okay:
                visible_weight+=weight;visible_count+=1;visible_depths.add(index)
                if near-1e-10<=np.linalg.norm(point-camera_xy)<=far+1e-10:
                    range_weight+=weight
        support=visible_weight/total_weight
        okay=support+1e-12>=min_support and len(visible_depths)>=2
        reason=('Candidate sees bounded bearing support' if okay else
                'Insufficient candidate bearing support: '+max(reasons,key=reasons.get))
        return VisibilityResult(okay,reason,support,len(source_support),visible_count,tuple(camera_xy),range_weight/total_weight)
    except (ValueError,TypeError,AttributeError,OverflowError) as exc:
        return VisibilityResult(False,'Invalid visibility geometry: '+str(exc))
