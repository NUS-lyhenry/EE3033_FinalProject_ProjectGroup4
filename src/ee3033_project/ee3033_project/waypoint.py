"""Map-derived approaches to an operator's stage-1 position and heading."""
import math
import numpy as np
from scipy.ndimage import label
from .coverage import CoveragePlanner
from .frontiers import frontier_goal


def validate_waypoint(data,resolution,origin,target):
    if len(target)!=3 or not all(math.isfinite(v) for v in target):
        raise ValueError('Waypoint position and heading must be finite')
    planner=CoveragePlanner(data,resolution,origin)
    row,col=planner.world_to_cell(*target[:2])
    if not planner.in_bounds(row,col):raise ValueError('Choose a waypoint inside the current map')
    if planner.grid[row,col]>=25:raise ValueError('Choose free or unexplored space, away from mapped obstacles')
    return tuple(map(float,target[:2]))+(math.atan2(math.sin(target[2]),math.cos(target[2])),)


def waypoint_goal(data,resolution,origin,robot,target,failed=(),visited=(),clearance=.22):
    target=validate_waypoint(data,resolution,origin,target)
    planner=CoveragePlanner(data,resolution,origin,clearance=clearance)
    row,col=planner.world_to_cell(*robot[:2]);tr,tc=planner.world_to_cell(*target[:2])
    if not planner.in_bounds(row,col):raise ValueError('Robot is outside the current map')
    components,_=label(planner.safe,structure=[[0,1,0],[1,1,1],[0,1,0]])
    if planner.safe[row,col] and planner.safe[tr,tc] and components[row,col]==components[tr,tc]:
        if any(math.hypot(target[0]-x,target[1]-y)<.4 for x,y in failed):return None,1,False
        return target,0,True
    goal,groups=frontier_goal(data,resolution,origin,robot,tuple(failed)+tuple(visited),clearance,toward=target[:2])
    return goal,groups,False
