"""Fixed-bin scan conversion, preserving near returns unless explicitly masked."""
import math


def resample(ranges,angle_min,angle_increment,range_min,range_max,bins,self_mask_radius=0.):
    # Missing/invalid input is unknown, not an invented obstacle-free ray.
    out=[math.nan]*bins
    blocked=set()
    for i,distance in enumerate(ranges):
        if math.isnan(distance) or distance<range_min or (math.isfinite(distance) and distance>range_max):
            continue
        a=(angle_min+i*angle_increment+math.pi)%(2*math.pi)-math.pi
        j=round((a+math.pi)/(2*math.pi/bins))%bins
        if distance<self_mask_radius:
            blocked.add(j)
        else:
            out[j]=min(out[j],distance) if not math.isnan(out[j]) else distance
    # Never turn an occluded self-return into an artificial clear ray.
    for j in blocked:
        out[j]=math.nan
    return out
