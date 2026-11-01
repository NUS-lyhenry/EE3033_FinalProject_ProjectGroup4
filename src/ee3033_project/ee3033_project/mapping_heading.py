"""Terminal-heading policy scoped to automatic frontier mapping."""
import math

MAPPING_PLANNER_ID = 'MappingTangent'
MAPPING_TREE = 'navigate_mapping.xml'


def mapping_tangent_tree(behavior_tree):
    return str(behavior_tree).rsplit('/', 1)[-1] == MAPPING_TREE


def terminal_path_yaw(points):
    """Last nonzero XY segment, ignoring repeated endpoint samples."""
    points=[(float(p[0]),float(p[1])) for p in points]
    if not all(math.isfinite(v) for point in points for v in point):
        raise ValueError('Path tangent requires finite XY coordinates')
    if not points:return None
    end=points[-1]
    for previous in reversed(points[:-1]):
        if math.dist(previous,end)>1e-6:
            return math.atan2(end[1]-previous[1],end[0]-previous[0])
    return None
