"""Reachable frontier selection in a changing SLAM occupancy grid."""
import math
import numpy as np
from scipy.ndimage import binary_dilation, label
from .coverage import CoveragePlanner
from .core import angle_difference
from .motion_safety import FOOTPRINT_RADIUS, polyline_map_clearance
from .path_validation import safe_grid, validate_path


def connection_cell(planner, point, safe_mask=None, allowed=None):
    """Find a nearby safe raster anchor without changing the requested point.

    Both the real point and its entire connector must fit the physical disc;
    proximity alone cannot authorize crossing a corner or an unknown cell.
    """
    if not validate_path(planner.grid,planner.resolution,planner.origin,[point[:2]],
                         mask=safe_mask,radius=FOOTPRINT_RADIUS).allowed:
        return None
    candidates=planner.safe if allowed is None else planner.safe & allowed
    cells=np.argwhere(candidates)
    if not len(cells):return None
    rc=np.asarray(planner.world_to_cell(*point[:2]))
    for row,col in cells[np.argsort(np.sum((cells-rc)**2,axis=1))[:32]]:
        target=planner.cell_to_world(int(row),int(col))
        if math.dist(point[:2],target)>.18:continue
        if validate_path(planner.grid,planner.resolution,planner.origin,[point[:2],target],
                         mask=safe_mask,radius=FOOTPRINT_RADIUS).allowed:
            return int(row),int(col)
    return None


def reachable_space(planner, robot, safe_mask=None):
    """Keep physical connectivity separate from the preferred soft reserve.

    A fresh Nav2 mask (required by the caller) authorizes using every fully
    known, footprint-safe connector. Soft clearance ranks destinations; it
    must not turn a physically traversable doorway into a disconnected wall.
    Without a navigation envelope retain the conservative soft graph.
    """
    physical=safe_grid(planner.grid,planner.resolution,FOOTPRINT_RADIUS)
    if safe_mask is not None:
        mask=np.asarray(safe_mask,dtype=bool)
        if mask.shape!=planner.safe.shape:
            raise ValueError('Navigation costmap mask does not match the current map')
        physical &= mask
    preferred=planner.safe & physical
    planner.safe=physical if safe_mask is not None else preferred
    planner.preferred_safe=preferred
    r,c=planner.world_to_cell(*robot[:2])
    if not planner.in_bounds(r,c):
        raise ValueError('Robot outside the current SLAM map')
    if not np.any(planner.safe):
        raise ValueError('No footprint-clear known mapped space; map remains incomplete')
    anchor=connection_cell(planner,robot,safe_mask)
    if anchor is None:
        raise ValueError('No connected footprint-clear known space; use an open starting area')
    components,_=label(planner.safe,structure=[[0,1,0],[1,1,1],[0,1,0]])
    reachable=components==components[anchor]
    preferred_anchor=connection_cell(planner,robot,safe_mask,allowed=preferred)
    preferred_components,_=label(preferred,structure=[[0,1,0],[1,1,1],[0,1,0]])
    planner.preferred_reachable=(preferred_components==preferred_components[preferred_anchor]
        if preferred_anchor is not None else np.zeros_like(preferred))
    return reachable


def frontier_goal(data, resolution, origin, robot, failed=(), clearance=0.22,
                  min_frontier_cells=5, approach_distance=0.75, toward=None,
                  safe_mask=None, visited=()):
    planner = CoveragePlanner(data,resolution,origin,clearance=max(FOOTPRINT_RADIUS,clearance))
    reachable = reachable_space(planner,robot,safe_mask)
    frontier = planner.free & binary_dilation(planner.grid<0,
                                               structure=[[0,1,0],[1,1,1],[0,1,0]])
    # Slanted SLAM boundaries form diagonal chains. Four-neighbour grouping
    # breaks them into tiny fragments that the noise filter discards. Group
    # boundaries with eight neighbours; driving reachability stays four-way
    # through the independently clearance-checked safe grid above.
    groups,count = label(frontier,structure=np.ones((3,3),dtype=bool))
    # Try the comfortable connected area first, then comfortable destinations
    # reached through a narrow connector, finally physical-safe destinations.
    tiers=[reachable & planner.preferred_reachable,
           reachable & planner.preferred_safe & ~planner.preferred_reachable,
           reachable & ~planner.preferred_safe]
    safe_cells=[np.argwhere(tier) for tier in tiers]
    options=[]
    valid_groups=0
    for group in range(1,count+1):
        cells=np.argwhere(groups==group)
        # Tiny unresolved frontiers may be noise, but cannot prove completion.
        valid_groups+=1
        if len(cells)<min_frontier_cells:
            continue
        # A frontier can be a ring whose centroid is far from its boundary.
        # Sample actual frontier cells instead of navigating toward that centroid.
        representatives=cells[np.linspace(0,len(cells)-1,min(12,len(cells)),dtype=int)]
        for edge in representatives:
            for tier,tier_cells in enumerate(safe_cells):
                distances=np.sum((tier_cells-edge)**2,axis=1)
                candidates=tier_cells[np.argsort(distances)[:20]]
                for row,col in candidates:
                    if np.linalg.norm(edge-[row,col])*resolution>approach_distance:
                        continue
                    x,y=planner.cell_to_world(int(row),int(col))
                    if not polyline_map_clearance(planner.grid,resolution,origin,[(x,y)],
                                                   radius=FOOTPRINT_RADIUS).allowed:
                        continue
                    # Reject a failed approach, not the entire frontier around it.
                    if any(math.hypot(x-fx,y-fy)<max(.12,2*resolution) for fx,fy in failed):
                        continue
                    n=int(max(abs(edge[0]-row),abs(edge[1]-col))*2)+1
                    line=np.rint(np.linspace([row,col],edge,n)).astype(int)
                    if not np.all(planner.free[line[:,0],line[:,1]]):
                        continue
                    cx,cy=planner.cell_to_world(edge[0],edge[1])
                    yaw=math.atan2(cy-y,cx-x)
                    # A stored view suppresses the same position AND facing. A
                    # legacy XY-only record suppresses just its local approach, not
                    # every direction into a room that still contains frontiers.
                    if any(math.hypot(x-v[0],y-v[1])<(.40 if len(v)>=3 else max(.12,2*resolution))
                            and (len(v)<3 or abs(angle_difference(yaw,v[2]))<.6) for v in visited):
                        continue
                    distance=math.hypot(x-robot[0],y-robot[1])
                    if distance<0.35:
                        continue
                    turn=abs(angle_difference(math.atan2(y-robot[1],x-robot[0]),robot[2]))
                    score=len(cells)*resolution/(0.5+distance+0.4*turn)
                    if toward is not None:
                        score/=0.25+math.hypot(cx-toward[0],cy-toward[1])
                    options.append((-tier,score,(x,y,yaw)))
                    break
    return max(options,key=lambda p:p[:2])[2] if options else None,valid_groups
