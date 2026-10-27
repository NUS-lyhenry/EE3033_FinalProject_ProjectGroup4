"""Capture-time, continuous-odometry bearing intersection; no ROS dependencies."""
import math
import numpy as np
from .core import angle_difference


class BearingIntersection:
    def __init__(self, frames=4, max_age=120., max_sites=3):
        self.frames, self.max_age, self.max_sites = frames, max_age, max_sites
        self.reset()

    def reset(self):
        self.sites = []
        self.anchor = None
        self.label = None
        self.last_stamp = -math.inf
        self.last_sources = set()
        self.reason = 'Collecting first observation'
        self.reason_code = 'awaiting_first_observation'
        self.last_events = []
        self.frame_accepted = False
        self.crossing_angle = None

    def reject(self, code, reason):
        self.reason_code,self.reason=code,reason
        self.last_events.append(code)
        return None

    def add(self, d):
        self.last_events=[]
        self.frame_accepted=False
        try:
            stamp = float(d['stamp'])
            origin = np.asarray(d['odom_origin'], dtype=float)
            angle = float(d['odom_bearing'])
            anchor = np.asarray(d['map_odom'], dtype=float)
            raw_sigma = float(d['bearing_sigma'])
            center_error = float(d.get('center_error', math.inf))
            confidence = float(d['confidence'])
            label = d['label']
            position_sigma = float(d.get('position_sigma', .02))
            sigma = max(math.radians(1.), raw_sigma)
            if origin.shape != (2,) or anchor.shape != (3,):
                return self.reject('bearing_invalid_geometry_shape','Bearing origin or map-to-odom has invalid dimensions')
            if not np.isfinite([*origin,*anchor,angle,raw_sigma,stamp,center_error,confidence,position_sigma]).all():
                return self.reject('bearing_nonfinite_geometry','Bearing frame has nonfinite geometry, timestamp or uncertainty')
            failures=[]
            if raw_sigma<=0 or position_sigma<0:failures.append(('bearing_invalid_uncertainty','Invalid bearing uncertainty'))
            if not d.get('settled'):failures.append(('bearing_motion_unsettled','Capture-time motion window did not pass'))
            if abs(center_error)>math.radians(2.):failures.append(('bearing_not_centered','Horizontal centering error exceeds two degrees'))
            if not .6<=confidence<=1.:failures.append(('bearing_low_confidence','Detection confidence is below 0.60 or invalid'))
            if stamp<=self.last_stamp:failures.append(('bearing_duplicate_or_reordered_frame','Frame timestamp was already consumed or reordered'))
            if failures:
                self.last_events.extend(code for code,_ in failures[1:])
                return self.reject(*failures[0])
        except (KeyError, TypeError, ValueError) as exc:
            return self.reject('bearing_missing_or_invalid_fields','Cannot parse bearing frame: '+str(exc))
        if self.anchor is not None and (self.label != d['label']
                or np.linalg.norm(anchor[:2]-self.anchor[:2]) > .15
                or abs(angle_difference(anchor[2], self.anchor[2])) > math.radians(5.)):
            self.reset()
            self.last_events=['bearing_localization_or_target_changed']
            self.reason = 'Localization changed; recollecting bearings'
        self.anchor, self.label, self.last_stamp = anchor, d['label'], stamp
        self.sites = [s for s in self.sites if stamp-s[-1]['stamp'] <= self.max_age]
        sample = dict(stamp=stamp, origin=origin, map_origin=d.get('origin'), angle=angle, sigma=sigma,
                      position_sigma=max(.02, float(d.get('position_sigma', .02))))
        site = next((s for s in self.sites if np.linalg.norm(s[-1]['origin']-origin) < .08), None)
        if site is None:
            if len(self.sites) >= self.max_sites:
                return self.reject('bearing_site_limit','Three distinct capture positions are already in use')
            self.sites.append([sample])
        else:
            # Each frame already passed capture-time motion, centering and
            # association checks. Intermittent valid frames are not a reset:
            # collect four within six seconds, retaining A while travelling B.
            site[:]=[p for p in site if stamp-p['stamp']<=6.]
            site.append(sample)
            del site[:-12]
        self.frame_accepted=True
        ready = [s for s in self.sites if len(s) >= self.frames]
        if len(ready) < 2:
            counts='/'.join(str(len(s)) for s in self.sites)
            return self.reject('bearing_collecting_site_frames',f'Collecting distinct settled frames at A/B; site frame counts: {counts}')
        origins = np.array([np.mean([p['origin'] for p in s], axis=0) for s in ready])
        angles = np.array([math.atan2(np.mean([math.sin(p['angle']) for p in s]),
                                    np.mean([math.cos(p['angle']) for p in s])) for s in ready])
        crossing = max(abs(math.sin(a-b)) for a in angles for b in angles)
        self.crossing_angle=float(math.asin(crossing))
        if crossing < math.sin(math.radians(15.)):
            return self.reject('bearing_weak_crossing_angle','Insufficient intersection angle: '+str(round(math.degrees(self.crossing_angle),2))+' degrees')
        rays = np.column_stack([np.cos(angles), np.sin(angles)])
        normals = np.column_stack([-np.sin(angles), np.cos(angles)])
        bearing_sigma = np.array([max(p['sigma'] for p in s) for s in ready])
        position_sigma = np.array([max(p['position_sigma'] for p in s) for s in ready])
        ranges = np.ones(len(ready))
        for _ in range(3):
            variances = position_sigma**2 + (ranges*bearing_sigma)**2
            information = normals.T @ (normals/variances[:, None])
            if np.linalg.cond(information) > 1e4:
                return self.reject('bearing_ill_conditioned','Bearing intersection information matrix is ill-conditioned')
            covariance = np.linalg.inv(information)
            point = covariance @ (normals.T @ (np.sum(normals*origins, axis=1)/variances))
            ranges = np.linalg.norm(point-origins, axis=1)
        if np.any(np.sum((point-origins)*rays, axis=1) <= 0) or np.any(ranges > 3.):
            return self.reject('bearing_intersection_outside_envelope','Intersection behind camera or outside observation envelope')
        residual = np.sum(normals*(point-origins), axis=1)
        if np.any(residual**2 > 9*variances):
            return self.reject('bearing_inconsistent_views','Inconsistent views; do not switch targets')
        c, s = math.cos(anchor[2]), math.sin(anchor[2])
        rotation = np.array([[c, -s], [s, c]])
        point = rotation @ point + anchor[:2]
        covariance = rotation @ covariance @ rotation.T + np.eye(2)*.02**2
        sources = {p['stamp'] for site in ready for p in site}
        if sources == self.last_sources:
            return self.reject('bearing_duplicate_evidence','The same raw frame set already produced an intersection')
        self.last_sources = sources
        self.reason = 'Valid bearing intersection'
        self.reason_code='bearing_intersection_valid'
        return dict(stamp=stamp, x=float(point[0]), y=float(point[1]),
                    sigma=math.sqrt(float(np.linalg.eigvalsh(covariance).max())),
                    covariance=covariance.tolist(), label=d['label'], confidence=d['confidence'],
                    source='bearing_intersection', source_frames=sorted(sources),
                    sites=len(ready), crossing_angle=math.asin(crossing),
                    baseline=float(max(np.linalg.norm(a-b) for a in origins for b in origins)))

    def progress(self):
        return dict(reason=self.reason,reason_code=self.reason_code,frame_accepted=self.frame_accepted,
            reason_codes=list(self.last_events),crossing_angle=self.crossing_angle,sites=[dict(
            frames=len(s), ready=len(s)>=self.frames, stamp=s[-1]['stamp'],
            first_stamp=s[0]['stamp'],
            origin=s[-1].get('map_origin')) for s in self.sites if s])
