"""Fixed-bin scan conversion with a robot-interior self-return mask."""
import math


def resample(ranges,angle_min,angle_increment,range_min,range_max,bins,self_mask_radius):
    out=[math.inf]*bins
    blocked=set()
    for i,distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min<=distance<=range_max:
            continue
        a=(angle_min+i*angle_increment+math.pi)%(2*math.pi)-math.pi
        j=round((a+math.pi)/(2*math.pi/bins))%bins
        if distance<self_mask_radius:
            blocked.add(j)
        else:
            out[j]=min(out[j],distance)
    # Never turn an occluded self-return into an artificial clear ray.
    for j in blocked:
        out[j]=math.nan
    return out
