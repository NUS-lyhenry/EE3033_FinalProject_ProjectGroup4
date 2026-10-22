"""Reachable frontier selection in a changing SLAM occupancy grid."""
import math
import numpy as np
from scipy.ndimage import binary_dilation, label
from .coverage import CoveragePlanner
from .core import angle_difference


def frontier_goal(data, resolution, origin, robot, failed=(), clearance=0.22,
                  min_frontier_cells=5, approach_distance=0.75):
    planner = CoveragePlanner(data,resolution,origin,clearance=clearance)
    r,c = planner.world_to_cell(*robot[:2])
    if not planner.in_bounds(r,c):
        raise ValueError('Robot outside the current SLAM map')
    components,_ = label(planner.safe,structure=[[0,1,0],[1,1,1],[0,1,0]])
    if not planner.safe[r,c]:
        safe = np.argwhere(planner.safe)
        if not len(safe):
            raise ValueError(f'No mapped space with {clearance:.2f} m clearance; map remains incomplete')
        near = safe[np.argmin(np.sum((safe-np.array([r,c]))**2,axis=1))]
        if np.linalg.norm(near-[r,c])*resolution > 0.18:
            raise ValueError(f'No connected space with {clearance:.2f} m clearance; use an open starting area')
        r,c = map(int,near)
    reachable = components==components[r,c]
    frontier = planner.free & binary_dilation(planner.grid<0,
                                               structure=[[0,1,0],[1,1,1],[0,1,0]])
    # Slanted SLAM boundaries form diagonal chains. Four-neighbour grouping
    # breaks them into tiny fragments that the noise filter discards. Group
    # boundaries with eight neighbours; driving reachability stays four-way
    # through the independently clearance-checked safe grid above.
    groups,count = label(frontier,structure=np.ones((3,3),dtype=bool))
    safe_cells = np.argwhere(reachable)
    options=[]
    valid_groups=0
    for group in range(1,count+1):
        cells=np.argwhere(groups==group)
        if len(cells)<min_frontier_cells:
            continue
        valid_groups+=1
        # A frontier can be a ring whose centroid is far from its boundary.
        # Sample actual frontier cells instead of navigating toward that centroid.
        representatives=cells[np.linspace(0,len(cells)-1,min(12,len(cells)),dtype=int)]
        for edge in representatives:
            distances=np.sum((safe_cells-edge)**2,axis=1)
            candidates=safe_cells[np.argsort(distances)[:20]]
            for row,col in candidates:
                if np.linalg.norm(edge-[row,col])*resolution>approach_distance:
                    continue
                x,y=planner.cell_to_world(int(row),int(col))
                if any(math.hypot(x-fx,y-fy)<0.40 for fx,fy in failed):
                    continue
                n=int(max(abs(edge[0]-row),abs(edge[1]-col))*2)+1
                line=np.rint(np.linspace([row,col],edge,n)).astype(int)
                if not np.all(planner.free[line[:,0],line[:,1]]):
                    continue
                cx,cy=planner.cell_to_world(edge[0],edge[1])
                yaw=math.atan2(cy-y,cx-x)
                distance=math.hypot(x-robot[0],y-robot[1])
                if distance<0.35:
                    continue
                turn=abs(angle_difference(math.atan2(y-robot[1],x-robot[0]),robot[2]))
                score=len(cells)*resolution/(0.5+distance+0.4*turn)
                options.append((score,(x,y,yaw)))
                break
    return max(options,key=lambda p:p[0])[1] if options else None,valid_groups
