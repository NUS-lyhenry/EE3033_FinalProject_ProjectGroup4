"""Map-derived approaches to an operator's stage-1 position and heading."""
import math
from .coverage import CoveragePlanner
from .frontiers import frontier_goal,reachable_space,connection_cell


def validate_waypoint(data,resolution,origin,target):
    if len(target)!=3 or not all(math.isfinite(v) for v in target):
        raise ValueError('Waypoint position and heading must be finite')
    planner=CoveragePlanner(data,resolution,origin)
    row,col=planner.world_to_cell(*target[:2])
    if not planner.in_bounds(row,col):raise ValueError('Choose a waypoint inside the current map')
    if planner.grid[row,col]>=25:raise ValueError('Choose free or unexplored space, away from mapped obstacles')
    return tuple(map(float,target[:2]))+(math.atan2(math.sin(target[2]),math.cos(target[2])),)


def waypoint_goal(data,resolution,origin,robot,target,failed=(),visited=(),clearance=.22,safe_mask=None):
    """Reach the exact goal through known physical-safe space, if necessary.

    ``safe_mask`` is a fresh Nav2 envelope supplied by Mapper. With that
    envelope, a soft reserve cannot disconnect a valid narrow connector; the
    exact target footprint, connector and costmap still must all be safe.
    """
    target=validate_waypoint(data,resolution,origin,target)
    planner=CoveragePlanner(data,resolution,origin,clearance=clearance)
    reachable=reachable_space(planner,robot,safe_mask)
    anchor=connection_cell(planner,target,safe_mask,allowed=reachable)
    if anchor is not None:
        if any(math.hypot(target[0]-x,target[1]-y)<max(.12,2*resolution) for x,y in failed):
            return None,1,False
        return target,0,True
    # Unknown destinations remain exploration requests. Completed approaches are
    # separate view records, not failed regions or a reason to skip whole rooms.
    goal,groups=frontier_goal(data,resolution,origin,robot,failed,clearance,
        toward=target[:2],safe_mask=safe_mask,visited=visited)
    return goal,groups,False
