"""Independent physical-world acceptance oracle; never imported by navigation.

Uses finite wall segments and a conservative four-connected grid. It does not
call CoveragePlanner, frontier selection, Nav2 costmaps or production planners.
"""
from collections import deque
import math
import numpy as np


def geometry_profile(name='measured'):
    """Independent acceptance dimensions, recorded alongside every result."""
    if name=='measured':
        exact=math.hypot(.18/2,.115/2)
        body=math.ceil(exact*1_000_000.)/1_000_000.
        return dict(name=name,model='circumscribed circle of measured wheel-inclusive rectangle',
            length=.115,width=.18,physical_radius=body,exact_circumradius=exact,rounding='outward_micrometre',
            padding=.01,footprint_radius=round(body+.01,6))
    if name=='legacy':
        return dict(name=name,model='original conservative circular stress envelope',
            length=None,width=.20,physical_radius=.14,padding=.01,footprint_radius=.15)
    raise ValueError('geometry profile must be measured or legacy')


def wall_distance(points,walls):
    p=np.asarray(points,dtype=float);w=np.asarray(walls,dtype=float)
    a=w[:,0];v=w[:,1]-a;d=p[...,None,:]-a
    u=np.clip(np.sum(d*v,axis=-1)/np.sum(v*v,axis=-1),0.,1.)
    return np.min(np.linalg.norm(d-u[...,None]*v,axis=-1),axis=-1)


def reachable_grid(scene,start=None,radius=.15,resolution=.025):
    xmin,ymin,xmax,ymax=scene['bounds'];start=scene['start'][:2] if start is None else start[:2]
    xs=np.arange(xmin+resolution/2,xmax,resolution);ys=np.arange(ymin+resolution/2,ymax,resolution)
    xx,yy=np.meshgrid(xs,ys);points=np.stack([xx,yy],axis=-1)
    safe=wall_distance(points,scene['walls'])>=radius+resolution/math.sqrt(2)
    near=np.argwhere(safe & (np.linalg.norm(points-np.asarray(start),axis=-1)<=2*resolution))
    seen=np.zeros(safe.shape,bool);queue=deque()
    for r,c in near:
        connector=np.linspace(start,points[r,c],9)
        if np.min(wall_distance(connector,scene['walls']))>=radius:
            queue.append((int(r),int(c)));seen[r,c]=True
    while queue:
        r,c=queue.popleft()
        for rr,cc in ((r-1,c),(r+1,c),(r,c-1),(r,c+1)):
            if 0<=rr<safe.shape[0] and 0<=cc<safe.shape[1] and safe[rr,cc] and not seen[rr,cc]:
                seen[rr,cc]=True;queue.append((rr,cc))
    return points,seen


def physically_reachable(scene,goal,start=None,radius=.15,resolution=.025):
    if wall_distance(goal[:2],scene['walls'])<radius:return False
    # A coarse four-connected grid can miss a valid narrow connector whose
    # centre lies between rows. Refine the independent oracle, rather than
    # changing the robot, door or navigation occupancy map to force a pass.
    for sample_resolution in (resolution,resolution/2,resolution/4):
        points,seen=reachable_grid(scene,start,radius,sample_resolution)
        choices=np.argwhere(seen & (np.linalg.norm(points-np.asarray(goal[:2]),axis=-1)<=2*sample_resolution))
        if any(np.min(wall_distance(np.linspace(points[r,c],goal[:2],9),scene['walls']))>=radius for r,c in choices):
            return True
    return False


def occupancy_grid(scene,resolution=.05):
    xmin,ymin,xmax,ymax=scene['bounds'];xmin-=2*resolution;ymin-=2*resolution;xmax+=2*resolution;ymax+=2*resolution
    width=math.ceil((xmax-xmin)/resolution);height=math.ceil((ymax-ymin)/resolution)
    xx,yy=np.meshgrid(xmin+(np.arange(width)+.5)*resolution,ymin+(np.arange(height)+.5)*resolution)
    occupied=wall_distance(np.stack([xx,yy],axis=-1),scene['walls'])<=resolution/math.sqrt(2)
    return dict(width=width,height=height,resolution=resolution,origin=[xmin,ymin,0.],cells=np.where(occupied,100,0).ravel().tolist())


def farthest_goal(scene,start=None):
    start=scene['start'] if start is None else start
    points,reachable=reachable_grid(scene,start,radius=.24,resolution=.05)
    choices=points[reachable]
    if not len(choices):raise ValueError('Independent oracle found no free start component')
    target=choices[np.argmax(np.linalg.norm(choices-np.asarray(start[:2]),axis=-1))]
    return [float(target[0]),float(target[1]),0.]


def observed_free_coverage(scene,map_data,map_from_world,start=None,resolution=.05,radius=.15):
    """Evaluate mapped free coverage of physically reachable body-centre space.

    This evaluator is never imported by robot code. The transform is produced
    by the localization algorithm, not corrected using simulated truth.
    """
    points,reachable=reachable_grid(scene,start=start,radius=radius,resolution=resolution)
    p=points[reachable]
    if not len(p):raise ValueError('No physical acceptance samples')
    tx,ty,_=map_from_world['translation'];qx,qy,qz,qw=map_from_world['rotation_xyzw']
    a=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));c,s=math.cos(a),math.sin(a)
    mx=tx+c*p[:,0]-s*p[:,1];my=ty+s*p[:,0]+c*p[:,1]
    ox,oy,b=map_data['origin'];cb,sb=math.cos(b),math.sin(b);r=map_data['resolution']
    cols=np.floor((cb*(mx-ox)+sb*(my-oy))/r).astype(int)
    rows=np.floor((-sb*(mx-ox)+cb*(my-oy))/r).astype(int)
    inside=(rows>=0)&(cols>=0)&(rows<map_data['height'])&(cols<map_data['width'])
    values=np.full(len(p),-1)
    cells=np.asarray(map_data['cells']).reshape(map_data['height'],map_data['width'])
    values[inside]=cells[rows[inside],cols[inside]]
    return dict(physical_reachable_samples=len(p),sample_resolution=resolution,
        mapped_free_fraction=float(np.mean((values>=0)&(values<25))),
        unknown_fraction=float(np.mean(values<0)),occupied_in_physical_free_fraction=float(np.mean(values>=25)),
        out_of_map_fraction=float(np.mean(~inside)))
