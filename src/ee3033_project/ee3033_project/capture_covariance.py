"""Finite capture-time uncertainty propagation from real AMCL and odometry."""
import math


def capture_covariance(history, image_stamp, max_age=1.0, *, odometry=(), max_propagation_age=15.0, max_gap=.25):
    """Never substitute future covariance or invent a missing localization anchor.

    Odometry tuples are (stamp,x,y,yaw,linear,angular,frame). Propagation uses
    engineering process noise, not an independently validated error bound.
    """
    samples=[]
    for stamp,variance in history:
        if (math.isfinite(stamp) and len(variance)==2 and
                all(math.isfinite(v) and v>=0 for v in variance)):
            samples.append((float(stamp),tuple(map(float,variance))))
    latest=max((t for t,_ in samples),default=None)
    past=[s for s in samples if s[0]<=image_stamp]
    anchor=max(past,key=lambda s:s[0]) if past else None
    age=image_stamp-anchor[0] if anchor else None
    info=dict(image_stamp=float(image_stamp),max_age_seconds=float(max_age),
        max_propagation_age_seconds=float(max_propagation_age),sample_count=len(samples),
        latest_pose_stamp=latest,latest_pose_age_seconds=image_stamp-latest if latest is not None else None,
        selected_pose_stamp=anchor[0] if anchor else None,pose_age_seconds=age,
        accepted=False,propagated=False,reason='missing' if not samples else 'future_only' if not anchor else 'stale')
    if not math.isfinite(image_stamp):info['reason']='invalid_capture_time';return None,info
    if anchor is None:return None,info
    if age<=max_age:
        info.update(accepted=True,reason='selected');return anchor[1],info
    if age>max_propagation_age:
        info['reason']='propagation_age_exceeded';return None,info
    points=[s for s in odometry if len(s)>=7 and all(math.isfinite(v) for v in s[:6]) and s[6]]
    # Include the sample before the anchor and never use a sample after exposure.
    before=[s for s in points if s[0]<=anchor[0]]
    start=max(before,key=lambda s:s[0]) if before else None
    track=[s for s in points if start is not None and start[0]<=s[0]<=image_stamp]
    if (start is None or anchor[0]-start[0]>max_gap or len(track)<2 or
            image_stamp-track[-1][0]>max_gap or len({s[6] for s in track})!=1 or
            any(not 0<b[0]-a[0]<=max_gap for a,b in zip(track,track[1:]))):
        info['reason']='odometry_coverage_missing' if points else 'stale';return None,info
    distance=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(track,track[1:]))
    rotation=sum(abs(math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]))) for a,b in zip(track,track[1:]))
    net=math.hypot(track[-1][1]-track[0][1],track[-1][2]-track[0][2])
    xy,yaw=anchor[1]
    propagated=(xy+net*net*yaw+.01**2*age+.03**2*distance,
                yaw+.02**2*age+.05**2*rotation)
    info.update(accepted=True,propagated=True,reason='propagated_odometry',
        odometry_samples=len(track),distance_m=distance,rotation_rad=rotation,
        process_variance_xy=propagated[0]-xy,process_variance_yaw=propagated[1]-yaw)
    return propagated,info
