"""Validated physical simulation worlds; never supplies SLAM or mission outputs.

Distances are metres. Walls are finite XY line segments, targets are floor XYZ,
start is chassis XY/yaw, and bounds is [xmin, ymin, xmax, ymax]. The first four
walls form the closed rectangular exterior. Interior walls may occlude targets.
Validation uses a conservative 5 cm four-connected robot-clearance grid; this
is scene setup validation, not a route or map supplied to the robot algorithms.
"""
import copy
import math
import random
from collections import deque

import numpy as np

SCHEMA_VERSION = 1
ROBOT_CLEARANCE = .24
GRID_RESOLUTION = .05


def default_scenario():
    """Return an independent JSON-compatible copy of the original preview world."""
    return dict(version=1, name='Original divider room', seed=None,
        bounds=[-1.2, -1., 1.4, 1.3], start=[0., 0., 0.], target=[.95, .85, 0.],
        walls=[[[-1.2, -1.], [1.4, -1.]], [[1.4, -1.], [1.4, 1.3]],
               [[1.4, 1.3], [-1.2, 1.3]], [[-1.2, 1.3], [-1.2, -1.]],
               [[.55, .15], [.55, 1.3]]])


def _vector(value, length, field):
    if (not isinstance(value, (list, tuple)) or len(value) != length
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in value)):
        raise ValueError(field+' must contain '+str(length)+' finite numbers')
    return [float(v) for v in value]


def _seed(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 2**32-1:
        raise ValueError('seed must be an integer from 0 to 4294967295')
    return value


def _clearance(points, walls):
    points=np.asarray(points,dtype=float)
    segments=np.asarray(walls,dtype=float)
    edge=segments[:,1]-segments[:,0]
    offset=points[...,None,:]-segments[:,0]
    along=np.clip(np.sum(offset*edge,axis=-1)/np.sum(edge*edge,axis=-1),0.,1.)
    return np.min(np.linalg.norm(offset-along[...,None]*edge,axis=-1),axis=-1)


def _connected_grid(scene):
    """Return physical-world sample points reachable from the validated start."""
    xmin,ymin,xmax,ymax=scene['bounds']
    xs=np.arange(xmin+GRID_RESOLUTION/2,xmax,GRID_RESOLUTION)
    ys=np.arange(ymin+GRID_RESOLUTION/2,ymax,GRID_RESOLUTION)
    xx,yy=np.meshgrid(xs,ys)
    points=np.stack([xx,yy],axis=-1)
    # Inflate by half a grid diagonal so any path through adjacent grid cells
    # retains the requested chassis clearance, including between sample points.
    safe=_clearance(points,scene['walls']) >= ROBOT_CLEARANCE+GRID_RESOLUTION/math.sqrt(2)
    start=np.asarray(scene['start'][:2])
    # A short safe connector is needed when start is not a cell centre.
    distances=np.linalg.norm(points-start,axis=-1)
    choices=np.argwhere(safe & (distances<=GRID_RESOLUTION*2))
    visited=np.zeros(safe.shape,dtype=bool)
    if not len(choices):return points,visited
    seeds=[]
    for r,c in choices:
        connector=np.linspace(start,points[r,c],5)
        if np.all(_clearance(connector,scene['walls'])>=ROBOT_CLEARANCE):
            seeds.append((int(r),int(c)));visited[r,c]=True
    queue=deque(seeds)
    while queue:
        r,c=queue.popleft()
        for rr,cc in ((r-1,c),(r+1,c),(r,c-1),(r,c+1)):
            if 0<=rr<safe.shape[0] and 0<=cc<safe.shape[1] and safe[rr,cc] and not visited[rr,cc]:
                visited[rr,cc]=True;queue.append((rr,cc))
    return points,visited


def validate_scenario(value):
    """Return normalized JSON data, or raise ValueError for an unsafe scene."""
    if not isinstance(value,dict):raise ValueError('scenario must be an object')
    if value.get('version') != SCHEMA_VERSION or isinstance(value.get('version'),bool):
        raise ValueError('Unsupported scenario version')
    name=value.get('name','Custom room')
    if not isinstance(name,str) or not 1<=len(name)<=100:raise ValueError('Invalid scenario name')
    seed=value.get('seed')
    if seed is not None:seed=_seed(seed)
    bounds=_vector(value.get('bounds'),4,'bounds')
    xmin,ymin,xmax,ymax=bounds
    if not (1.5<=xmax-xmin<=8. and 1.5<=ymax-ymin<=8.) or max(map(abs,bounds))>20:
        raise ValueError('Room dimensions must be between 1.5 and 8 metres')
    walls=value.get('walls')
    if not isinstance(walls,list) or not 4<=len(walls)<=48:
        raise ValueError('Provide 4 to 48 wall segments')
    normalized=[]
    for wall in walls:
        if not isinstance(wall,(list,tuple)) or len(wall)!=2:raise ValueError('Invalid wall segment')
        a,b=(_vector(point,2,'wall endpoint') for point in wall)
        if math.dist(a,b)<.01:raise ValueError('Wall segments must have nonzero length')
        if any(not (xmin-1e-8<=p[0]<=xmax+1e-8 and ymin-1e-8<=p[1]<=ymax+1e-8) for p in (a,b)):
            raise ValueError('Wall endpoint is outside room bounds')
        normalized.append([a,b])
    corners=[[xmin,ymin],[xmax,ymin],[xmax,ymax],[xmin,ymax]]
    expected=[[corners[i],corners[(i+1)%4]] for i in range(4)]
    if not np.allclose(normalized[:4],expected,rtol=0,atol=1e-8):
        raise ValueError('First four walls must close the room bounds in counterclockwise order')
    start=_vector(value.get('start'),3,'start')
    target=_vector(value.get('target'),3,'target')
    if abs(target[2])>1e-8:raise ValueError('Target must stand on the floor (z = 0)')
    for field,point in (('Start',start),('Target',target)):
        if not xmin<point[0]<xmax or not ymin<point[1]<ymax:
            raise ValueError(field+' is outside the room')
        if _clearance(point[:2],normalized)<ROBOT_CLEARANCE:
            raise ValueError(field+' has insufficient wall clearance')
    if math.dist(start[:2],target[:2])<.3:raise ValueError('Target is too close to the initial chassis position')
    start[2]=math.atan2(math.sin(start[2]),math.cos(start[2]))
    scene=dict(version=1,name=name,seed=seed,bounds=bounds,walls=normalized,start=start,target=target)
    points,reachable=_connected_grid(scene)
    near=reachable & (np.linalg.norm(points-np.asarray(target[:2]),axis=-1)<=GRID_RESOLUTION*2)
    if not any(np.all(_clearance(np.linspace(points[r,c],target[:2],5),normalized)>=ROBOT_CLEARANCE)
               for r,c in np.argwhere(near)):
        raise ValueError('Target is not connected to start with robot clearance')
    return scene


def with_target(scenario,position):
    """Validate a manually placed floor target without mutating the source."""
    if not isinstance(position,(list,tuple)) or len(position) not in (2,3):
        raise ValueError('Target position must contain x, y, and optionally z = 0')
    result=copy.deepcopy(scenario)
    result['target']=list(position)+([0.] if len(position)==2 else [])
    return validate_scenario(result)


def randomize_target(scenario,seed):
    """Choose a reproducible reachable target in an existing physical world."""
    result=validate_scenario(scenario)
    rng=random.Random(_seed(seed))
    points,reachable=_connected_grid(result)
    choices=points[reachable]
    choices=choices[np.linalg.norm(choices-np.asarray(result['start'][:2]),axis=-1)>=.6]
    if not len(choices):raise ValueError('No separated reachable target location is available')
    target=choices[rng.randrange(len(choices))]
    result['target']=[float(target[0]),float(target[1]),0.]
    return validate_scenario(result)


def generate_scenario(seed):
    """Generate deterministic, connected rooms with varied occluding walls."""
    seed=_seed(seed);rng=random.Random(seed)
    width=rng.uniform(2.8,3.8);height=rng.uniform(2.6,3.6)
    xmin,ymin=-width/2,-height/2;xmax,ymax=width/2,height/2
    corners=[[xmin,ymin],[xmax,ymin],[xmax,ymax],[xmin,ymax]]
    scene=dict(version=1,name='Random room '+str(seed),seed=seed,
        bounds=[xmin,ymin,xmax,ymax],start=[0.,0.,0.],target=[.7,.7,0.],
        walls=[[corners[i],corners[(i+1)%4]] for i in range(4)])
    # Alternate boundary-attached partitions and freestanding walls. Each wall
    # leaves a generous passage to its opposite boundary; validate the complete
    # world before returning rather than assuming that individually safe walls
    # compose into a connected layout.
    for _ in range(rng.randint(1,3)):
        vertical=rng.choice((True,False));side=rng.choice((-1,1))
        if vertical:
            x=rng.choice((-1,1))*rng.uniform(.55,min(1.0,width/2-.35))
            begin=ymin if side<0 else ymax
            end=begin-side*rng.uniform(.65,height-.8)
            wall=[[x,begin],[x,end]]
        else:
            y=rng.choice((-1,1))*rng.uniform(.55,min(.95,height/2-.35))
            begin=xmin if side<0 else xmax
            end=begin-side*rng.uniform(.65,width-.8)
            wall=[[begin,y],[end,y]]
        if _clearance([0.,0.],scene['walls']+[wall])>=.35:
            scene['walls'].append(wall)
    # Find a first reachable point without needing a placeholder target to be
    # valid. Do not inject knowledge of the target position into perception
    # or the navigation algorithms.
    points,reachable=_connected_grid(scene)
    candidates=points[reachable]
    candidates=candidates[np.linalg.norm(candidates,axis=-1)>=.75]
    if not len(candidates):raise ValueError('Generated room has no reachable target placement')
    chosen=candidates[rng.randrange(len(candidates))]
    scene['target']=[float(chosen[0]),float(chosen[1]),0.]
    return validate_scenario(scene)


def random_target(scenario,seed=None):
    """Convenience alias; callers should supply a seed to reproduce placement."""
    if seed is None:seed=random.SystemRandom().randrange(2**32)
    return randomize_target(scenario,seed)
