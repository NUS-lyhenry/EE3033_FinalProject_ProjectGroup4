"""Bounded per-frame localization evidence; no ROS or file-system dependencies."""
from collections import Counter, deque
import math
import numpy as np


def finite_json(value, depth=0):
    """Keep diagnostics valid JSON and bounded, including exceptional frames."""
    if depth>7:return None
    if isinstance(value,dict):
        return {str(k)[:80]:finite_json(v,depth+1) for k,v in list(value.items())[:40]}
    if isinstance(value,(list,tuple,np.ndarray)):
        return [finite_json(v,depth+1) for v in list(value)[:16]]
    if isinstance(value,(float,np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value,(bool,np.bool_)):return bool(value)
    if isinstance(value,(int,np.integer)):return int(value)
    if isinstance(value,str):return value[:500]
    if value is None or isinstance(value,bool):return value
    return str(value)[:200]


class DiagnosticHistory:
    def __init__(self,size=32):
        self.events=deque(maxlen=size)
        self.counts=Counter()
        self.total=0

    def reset(self):
        self.events.clear();self.counts.clear();self.total=0

    def add(self,event):
        event=finite_json(event)
        event['reason_codes']=list(dict.fromkeys(event.get('reason_codes',[])))[:32]
        self.events.append(event);self.total+=1
        self.counts.update(event['reason_codes'])
        return event

    def summary(self):
        latest=self.events[-1] if self.events else None
        return dict(frames=self.total,retained=len(self.events),
                    reason_counts=dict(self.counts.most_common(32)),latest=latest)


def mmse_rejection_reason(tracker,d,now):
    """Explain a failed call without changing the authoritative MMSE checks."""
    if tracker.frozen is not None:return 'mmse_already_reported'
    if tracker.grid is None or tracker.points is None or not len(tracker.points):return 'mmse_map_support_missing'
    if tracker.factor is not None:return 'mmse_floor_evidence_replaced_by_intersection'
    try:
        stamp,x,y,sigma,confidence=(float(d[k]) for k in ('stamp','x','y','sigma','confidence'))
        if not all(math.isfinite(v) for v in (stamp,x,y,sigma,confidence)):return 'mmse_nonfinite_measurement'
        if not 0<=now-stamp<=tracker.max_age:return 'mmse_measurement_stale_or_future'
        if not tracker.confidence<=confidence<=1:return 'mmse_confidence_rejected'
        if sigma<=0 or not math.isfinite(sigma*sigma):return 'mmse_invalid_uncertainty'
        covariance=np.asarray(d.get('covariance',np.eye(2)*sigma*sigma),dtype=float)
        if (covariance.shape!=(2,2) or not np.isfinite(covariance).all()
                or not np.allclose(covariance,covariance.T)
                or np.linalg.eigvalsh(covariance).min()<=0):return 'mmse_invalid_covariance'
        if tracker.samples and stamp<=tracker.samples[-1][0]:return 'mmse_duplicate_or_reordered_evidence'
        if tracker.result is not None and tracker.samples:
            delta=np.array([x,y])-np.asarray(tracker.result['likelihood_mean'])
            innovation=np.asarray(tracker.result['likelihood_covariance'])+covariance
            if float(delta@np.linalg.solve(innovation,delta))>16.:return 'mmse_innovation_rejected'
    except (KeyError,TypeError,ValueError,np.linalg.LinAlgError):return 'mmse_invalid_measurement'
    return 'mmse_measurement_rejected'
