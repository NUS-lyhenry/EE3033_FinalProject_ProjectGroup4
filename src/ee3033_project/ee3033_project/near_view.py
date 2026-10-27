"""Bounded near-field reframing; image hints are never target measurements."""
import math
import time
from .visibility import observation_visibility


def requires_reframing(detection):
    return bool(detection.get('reframe_required') or detection.get('range_state')=='too_near'
        or any(code in (detection.get('reason_codes') or ()) for code in
               ('bearing_horizontal_clipping','detection_vertical_clipping')))


def retreat_goal(planner, pose, detection, visited=(), positions=(), allowed=None,
                 deadline=None, cancel=None):
    """Choose a 0.18–0.30 m map-safe move away from the last observed ray.

    The caller caps the number and total extent of these moves. This returns
    a navigation observation pose, not a direct reverse motor command. Nav2
    may turn before translating; all navigation/velocity-gate checks remain.
    """
    if len(positions)>=3:
        return None,'Three observation positions already used'
    try:
        bearing=float(detection.get('bearing',detection.get('steering_bearing')))
        source=tuple(float(v) for v in detection.get('origin',detection.get('steering_origin')))
        valid=len(source)==2 and all(math.isfinite(v) for v in (*pose,*source,bearing))
    except (TypeError,ValueError,OverflowError):
        valid=False
    if not valid:
        return None,'Reframing needs the last image bearing and camera origin'
    camera=planner.camera_model
    camera_yaw=camera.yaw if camera else 0.
    try:
        minimum=float(detection.get('camera_min_range',camera.min_range if camera else .35))
        maximum=float(detection.get('camera_max_range',camera.max_range if camera else 2.))
    except (TypeError,ValueError,OverflowError):
        return None,'Invalid configured observation range'
    if not 0<minimum<maximum:
        return None,'Invalid configured observation range'
    # Unaccepted contact range is a broad action-selection hint, never a
    # coordinate sent to the estimator. Cropped bottom edges supply no hint.
    try:hint=float(detection['range_hint_m']) if detection.get('range_hint_m') is not None else None
    except (TypeError,ValueError,OverflowError):hint=None
    if hint is not None and math.isfinite(hint) and hint>0:
        bounds=(max(.08,hint-.15),min(maximum,hint+.15))
    else:
        bounds=(max(.08,minimum-.6),min(maximum,minimum+.15))
    if bounds[0]>=bounds[1]:return None,'No bounded near-field range hypotheses'
    distances,_=planner.travel_field(*pose[:2],deadline=deadline,cancel=cancel)
    failure='No connected, clear, visible near-field retreat'
    for distance in (.30,.24,.18):
        if (deadline is not None and time.monotonic()>=deadline) or (cancel is not None and cancel.is_set()):
            return None,'Near-field candidate selection canceled or timed out'
        x=pose[0]-distance*math.cos(bearing);y=pose[1]-distance*math.sin(bearing)
        yaw=bearing-camera_yaw
        cell=planner.world_to_cell(x,y)
        if (not planner.in_bounds(*cell) or not planner.safe[cell]
                or not math.isfinite(distances[cell]) or cell in visited
                or any(math.dist((x,y),p)<.12 for p in positions)):
            continue
        # A retreat must have a directly clear corridor, rather than a grid
        # route which reaches the rear endpoint by driving around an obstacle.
        count=max(2,math.ceil(distance/max(.01,planner.resolution/2))+1)
        clear=True
        for step in range(count):
            f=step/(count-1);sx=pose[0]+f*(x-pose[0]);sy=pose[1]+f*(y-pose[1])
            rc=planner.world_to_cell(sx,sy)
            if (not planner.in_bounds(*rc) or not planner.safe[rc]
                    or (allowed is not None and allowed(sx,sy) is False)):
                clear=False;break
        if not clear:continue
        result=observation_visibility(planner,(x,y,yaw),source_origin=source,
            bearing=bearing,camera_model=camera,range_bounds=bounds,
            bearing_sigma=math.radians(12. if 'bearing' not in detection else 3.),
            require_range=False,deadline=deadline,cancel=cancel)
        if not result.visible:
            failure='Retreat visibility rejected: '+result.reason
            continue
        reason='Move away to recover near/cropped target framing'
        if getattr(result,'range_support',1.)<.6:
            reason+='; intermediate view still needs valid measurement range'
        return (x,y,yaw),reason
    return None,failure
