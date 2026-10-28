"""Validate Navfn paths and repair unsafe connectors on known safe map cells."""
from dataclasses import dataclass
import heapq
import math
import time

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt

from .motion_safety import FOOTPRINT_RADIUS, OCCUPIED_THRESHOLD, polyline_map_clearance


@dataclass(frozen=True)
class PlanCheck:
    path: tuple
    reason: str
    replaced: bool = False

    @property
    def allowed(self):
        return bool(self.path)


def world_cell(point, resolution, origin):
    dx,dy=point[0]-origin[0],point[1]-origin[1]
    c,s=math.cos(origin[2]),math.sin(origin[2])
    return math.floor((-s*dx+c*dy)/resolution),math.floor((c*dx+s*dy)/resolution)


def cell_world(cell, resolution, origin):
    y,x=(cell[0]+.5)*resolution,(cell[1]+.5)*resolution
    c,s=math.cos(origin[2]),math.sin(origin[2])
    return origin[0]+c*x-s*y,origin[1]+s*x+c*y


def costmap_mask(shape,resolution,origin,grid,cost_resolution,cost_origin):
    """Project a Nav2 inflated 0..255 costmap onto raw occupancy cell centres."""
    grid=np.asarray(grid)
    if (grid.ndim!=2 or not grid.size or resolution<=0 or cost_resolution<=0
            or not np.isfinite(grid).all() or np.any((grid<0)|(grid>255))):
        raise ValueError('Invalid costmap')
    rows,cols=np.indices(shape)
    c,s=math.cos(origin[2]),math.sin(origin[2])
    x=origin[0]+c*(cols+.5)*resolution-s*(rows+.5)*resolution
    y=origin[1]+s*(cols+.5)*resolution+c*(rows+.5)*resolution
    c,s=math.cos(cost_origin[2]),math.sin(cost_origin[2])
    dx,dy=x-cost_origin[0],y-cost_origin[1]
    cc=np.floor((c*dx+s*dy)/cost_resolution).astype(int)
    rr=np.floor((-s*dx+c*dy)/cost_resolution).astype(int)
    inside=(rr>=0)&(cc>=0)&(rr<grid.shape[0])&(cc<grid.shape[1])
    mask=np.zeros(shape,dtype=bool)
    mask[inside]=grid[rr[inside],cc[inside]]<253
    return mask


def validate_path(grid,resolution,origin,points,*,mask=None,radius=FOOTPRINT_RADIUS):
    check=polyline_map_clearance(grid,resolution,origin,points,radius)
    if not check.allowed:return check
    if mask is not None:
        from .motion_safety import MotionCheck
        mask=np.asarray(mask,dtype=bool)
        if mask.shape!=np.asarray(grid).shape:
            return MotionCheck(False,'invalid_navigation_mask')
        points=np.asarray(points,dtype=float)
        for a,b in zip(points[:-1],points[1:]):
            n=max(1,math.ceil(math.dist(a,b)/(resolution/4)))
            for p in np.linspace(a,b,n+1):
                r,c=world_cell(p,resolution,origin)
                if not (0<=r<mask.shape[0] and 0<=c<mask.shape[1] and mask[r,c]):
                    return MotionCheck(False,'inflated_path_obstacle')
        if len(points)==1:
            r,c=world_cell(points[0],resolution,origin)
            if not (0<=r<mask.shape[0] and 0<=c<mask.shape[1] and mask[r,c]):
                return MotionCheck(False,'inflated_path_obstacle')
    return check


def safe_grid(grid,resolution,radius=FOOTPRINT_RADIUS):
    """Exact cell-square dilation at cell centres, including outside-map walls."""
    grid=np.asarray(grid)
    envelope=radius+resolution/4 # continuous-path sample-gap allowance
    n=math.ceil(envelope/resolution+.5)
    yy,xx=np.mgrid[-n:n+1,-n:n+1]
    dx=np.maximum(np.abs(xx)-.5,0)*resolution
    dy=np.maximum(np.abs(yy)-.5,0)*resolution
    structure=dx*dx+dy*dy<=envelope*envelope+1e-12
    blocked=(grid<0)|(grid>=OCCUPIED_THRESHOLD)
    padded=np.pad(blocked,n,constant_values=True)
    return ~binary_dilation(padded,structure=structure)[n:-n,n:-n]


def guarded_plan(grid,resolution,origin,start,goal,candidate=(),*,mask=None,
                 radius=FOOTPRINT_RADIUS,deadline=None,cancel=None):
    """Preserve a safe route, else compute one complete collision-free A* route.

    The raw requested start and goal are retained. Neither unknown space nor an
    occupied cell may be used to repair a Navfn route, and the final continuous
    footprint validator is authoritative over the discrete search.
    """
    grid=np.asarray(grid)
    if (grid.ndim!=2 or not grid.size or resolution<=0 or radius<=0
            or not np.isfinite(grid).all() or np.any((grid < -1)|(grid>100))
            or not all(math.isfinite(v) for v in (*origin,*start,*goal))):
        return PlanCheck((),'Invalid planning geometry')
    start,goal=tuple(start[:2]),tuple(goal[:2])
    def stopped():
        return (cancel is not None and cancel()) or (deadline is not None and time.monotonic()>=deadline)
    if stopped():return PlanCheck((),'Planning canceled or time limit exceeded')
    for point,label in ((start,'Start'),(goal,'Goal')):
        if not validate_path(grid,resolution,origin,[point],mask=mask,radius=radius).allowed:
            return PlanCheck((),label+' footprint is not in known safe space')
    points=tuple(tuple(map(float,p[:2])) for p in candidate)
    if points:
        # The endpoint connector belongs to the route and must be checked too.
        points=(start,)+points+(goal,)
        if validate_path(grid,resolution,origin,points,mask=mask,radius=radius).allowed:
            return PlanCheck(points,'Original route passed full-footprint validation')
    safe=safe_grid(grid,resolution,radius)
    if mask is not None:
        mask=np.asarray(mask,dtype=bool)
        if mask.shape!=safe.shape:return PlanCheck((),'Invalid navigation mask')
        safe &= mask
    cells=np.argwhere(safe)
    if not len(cells):return PlanCheck((),'No footprint-clear known route')
    # Validate both endpoint connectors; a safe continuous point need not be at
    # the centre of its raster cell, especially beside a door jamb.
    def neighbours(point):
        candidates=[]
        for rr,cc in cells[np.argsort(np.sum((cells-np.array(world_cell(point,resolution,origin)))**2,axis=1))[:32]]:
            cell=(int(rr),int(cc));world=cell_world(cell,resolution,origin)
            if math.dist(point,world)>max(.18,3*resolution):continue
            if validate_path(grid,resolution,origin,[point,world],mask=mask,radius=radius).allowed:
                candidates.append((cell,math.dist(point,world)))
        return candidates
    starts,ends=neighbours(start),dict(neighbours(goal))
    if not starts or not ends:return PlanCheck((),'No safe connector to route grid')
    dist=distance_transform_edt(np.pad((grid>=0)&(grid<OCCUPIED_THRESHOLD),1))[1:-1,1:-1]*resolution
    queue=[];costs={};parents={}
    for cell,cost in starts:
        costs[cell]=cost;parents[cell]=None
        heapq.heappush(queue,(cost+math.dist(cell_world(cell,resolution,origin),goal),cost,cell))
    best=None;best_cost=math.inf;steps=0
    while queue:
        estimate,cost,cell=heapq.heappop(queue)
        if estimate>=best_cost:break
        if cost!=costs.get(cell):continue
        steps+=1
        if steps%64==0 and stopped():return PlanCheck((),'Planning canceled or time limit exceeded')
        if cell in ends and cost+ends[cell]<best_cost:
            best,best_cost=cell,cost+ends[cell]
        r,c=cell
        # Four-way edges stay inside the union of individually safe cells.
        # The complete world route still passes the continuous check below.
        for dr,dc in ((1,0),(-1,0),(0,1),(0,-1)):
            nxt=(r+dr,c+dc)
            nr,nc=nxt
            if not (0<=nr<safe.shape[0] and 0<=nc<safe.shape[1] and safe[nr,nc]):continue
            clearance=max(.025,dist[nr,nc]-radius)
            new=cost+math.hypot(dr,dc)*resolution*(1.+.05/clearance)
            if new<costs.get(nxt,math.inf):
                costs[nxt]=new;parents[nxt]=cell
                heapq.heappush(queue,(new+math.dist(cell_world(nxt,resolution,origin),goal),new,nxt))
    if best is None:return PlanCheck((),'No footprint-clear known route')
    route=[]
    while best is not None:
        route.append(cell_world(best,resolution,origin));best=parents[best]
    points=(start,)+tuple(reversed(route))+(goal,)
    if stopped():return PlanCheck((),'Planning canceled or time limit exceeded')
    if not validate_path(grid,resolution,origin,points,mask=mask,radius=radius).allowed:
        return PlanCheck((),'Computed route failed continuous-footprint validation')
    return PlanCheck(points,'Replaced unsafe route with footprint-clear A* route',True)
