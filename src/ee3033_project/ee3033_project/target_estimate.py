"""Shared grid MMSE posterior with map support and a common-error model."""
from collections import deque
import hashlib
import json
import math
import numpy as np
from scipy.ndimage import label as components
from scipy.special import logsumexp


def map_signature(grid, resolution, origin):
    a = np.asarray(grid, dtype=np.int16)
    header = json.dumps([list(a.shape), float(resolution), list(map(float, origin))])
    return hashlib.sha256(header.encode()+a.tobytes()).hexdigest()


class MapMMSE:
    """Discrete posterior with correlated bias and nonzero unknown-cell prior.

    z_i = position + common_bias + independent_noise_i. The common variance
    envelope prevents correlated video frames from manufacturing precision.
    Its fraction is an explicit conservative model assumption, not a calibrated
    estimate of camera bias. Outputs are conditional on the loaded map extent.
    """
    def __init__(self, frames=4, seconds=8., max_samples=30, max_gap=1.2,
                 max_age=2., confidence=.6, common_fraction=.8, unknown_weight=.25):
        if (frames < 2 or max_samples < frames or not 0 <= common_fraction < 1
                or not 0 < unknown_weight <= 1 or min(seconds,max_gap,max_age) <= 0):
            raise ValueError('Invalid MMSE settings')
        self.frames,self.seconds,self.max_gap,self.max_age = frames,seconds,max_gap,max_age
        self.confidence,self.common_fraction,self.unknown_weight = confidence,common_fraction,unknown_weight
        self.samples=deque(maxlen=max_samples)
        self.bearings=deque(maxlen=max_samples)
        self.signature=None
        self.grid=self.points=self.prior=self.component_grid=None
        self.component=None
        self.reset()

    def reset(self):
        self.samples.clear()
        self.bearings.clear()
        self.streak=0
        self.result=None
        self.frozen=None
        self.label=None
        self.factor = None

    def update_factor(self, factor, now):
        """Replace correlated pixel projections with one joint geometric factor.

        A/B/C are jointly refitted, never appended as independent pairs sharing
        A. Confirmation uses their distinct source frames, not repeated factors.
        """
        try:
            sources=sorted(set(float(s) for s in factor.get('source_frames',[])))
            stamp=float(factor['stamp'])
            if not sources or not np.isfinite([*sources,stamp]).all() or max(sources)>stamp:return False
        except (ValueError,TypeError,KeyError,OverflowError):return False
        if (len(set(sources)) < self.frames*2 or factor.get('sites', 0) < 2
                or self.frozen is not None):
            return False
        if stamp<self._latest_stamp():return False
        if self.factor and (stamp <= self.factor['stamp'] or sources==self.factor['source_frames']):
            return False
        factor=dict(factor,source_frames=sources)
        # All floor measurements of this locked object share the same images
        # and mount error. The joint intersection replaces that likelihood.
        previous = self.samples, self.bearings, self.result, self.streak, self.label, self.factor
        self.samples = deque(maxlen=self.samples.maxlen)
        self.bearings = deque(maxlen=self.bearings.maxlen)
        self.result = None
        self.streak = 0
        self.factor = None
        if not self.update(factor, now):
            self.samples, self.bearings, self.result, self.streak, self.label, self.factor = previous
            return False
        self.factor = dict(factor)
        self.streak = len(set(sources))
        self.result.update({k: factor[k] for k in ('source', 'source_frames', 'sites', 'crossing_angle', 'baseline') if k in factor})
        self.result.update(first_source_stamp=float(min(sources)),last_source_stamp=float(max(sources)))
        return True

    def set_map(self, grid, resolution, origin):
        grid=np.asarray(grid,dtype=np.int16)
        if (grid.ndim!=2 or not 0 < grid.size <= 1000000 or not math.isfinite(resolution)
                or resolution<=0 or len(origin)!=3 or not np.isfinite(origin).all()
                or np.any((grid < -1)|(grid > 100))):
            raise ValueError('Invalid MMSE map')
        signature=map_signature(grid,resolution,origin)
        if signature==self.signature:return
        self.grid,self.resolution,self.origin=grid.copy(),float(resolution),tuple(origin)
        self.signature=signature
        # Target placement uses uninflated space; the robot radius is irrelevant.
        self.component_grid,_=components(grid < 65,structure=[[0,1,0],[1,1,1],[0,1,0]])
        self.component=None
        self._prior()
        self.reset()

    def set_seed(self,x,y):
        if self.grid is None or self.frozen is not None:return
        r,c=self.cell(x,y)
        if not (0<=r<self.grid.shape[0] and 0<=c<self.grid.shape[1]):return
        component=int(self.component_grid[r,c])
        if component and component!=self.component:
            self.component=component
            self._prior()
            self.reset()

    def cell(self,x,y):
        ox,oy,a=self.origin;dx,dy=x-ox,y-oy
        return math.floor((-math.sin(a)*dx+math.cos(a)*dy)/self.resolution),math.floor((math.cos(a)*dx+math.sin(a)*dy)/self.resolution)

    def _prior(self):
        weights=np.where(self.grid<0,self.unknown_weight,np.maximum(.05,1-self.grid/100.))
        weights[self.grid>=65]=0.
        if self.component is not None:weights[self.component_grid!=self.component]=0.
        self.prior_grid=weights
        r,c=np.nonzero(weights)
        x,y=(c+.5)*self.resolution,(r+.5)*self.resolution
        ox,oy,a=self.origin
        self.points=np.column_stack([ox+math.cos(a)*x-math.sin(a)*y,oy+math.sin(a)*x+math.cos(a)*y])
        self.prior=weights[r,c]

    def miss(self,stamp,deferred=False):
        latest=self._latest_stamp()
        if stamp<latest:return
        if not deferred or latest==-math.inf or stamp-latest>self.max_gap:
            self.streak=0

    def _latest_stamp(self):
        return max(self.samples[-1][0] if self.samples else -math.inf,
                   self.bearings[-1]['stamp'] if self.bearings else -math.inf)

    def _source_frames(self):
        return {s[5] for s in self.samples}|{s['source_frame'] for s in self.bearings}

    def _trim(self,stamp):
        while self.samples and stamp-self.samples[0][0]>self.seconds:self.samples.popleft()
        while self.bearings and stamp-self.bearings[0]['stamp']>self.seconds:self.bearings.popleft()
        while len(self.samples)+len(self.bearings)>=self.samples.maxlen:
            if self.samples and (not self.bearings or self.samples[0][0]<=self.bearings[0]['stamp']):
                self.samples.popleft()
            else:self.bearings.popleft()

    def update_bearing(self,d,now):
        """Condition map support on a ray without inventing a depth measurement.

        A source image contributes at most one raw likelihood. Frames from a
        nearly unchanged camera site share their angular/origin uncertainty;
        they provide confirmation, without a sqrt(N) precision improvement.
        Only explicitly supplied depth_bounds_m restrict distance along a ray.
        """
        if self.factor is not None or self.frozen is not None or self.grid is None or not len(self.points):return False
        try:
            stamp=float(d['stamp']);source=float(d.get('source_frame',stamp))
            origin=np.asarray(d['origin'],dtype=float)
            angle,sigma,confidence=[float(d[k]) for k in ('bearing','bearing_sigma','confidence')]
            position_sigma=float(d.get('position_sigma',.02));name=d['label']
            center_error=float(d.get('center_error',math.inf))
            if (origin.shape!=(2,) or not np.isfinite([*origin,stamp,source,angle,sigma,confidence,position_sigma,center_error]).all()
                    or not 0<=now-stamp<=self.max_age or not 0<=now-source<=self.max_age or source>stamp
                    or not self.confidence<=confidence<=1
                    or sigma<=0 or position_sigma<0 or not math.isfinite(sigma*sigma+position_sigma*position_sigma)
                    or not isinstance(name,str)
                    or d.get('settled') is not True or abs(center_error)>math.radians(2.)):
                return False
            bounds=d.get('depth_bounds_m')
            if bounds is not None:
                bounds=np.asarray(bounds,dtype=float)
                if bounds.shape!=(2,) or not np.isfinite(bounds).all() or not 0<=bounds[0]<bounds[1]:return False
        except (ValueError,TypeError,KeyError,OverflowError):return False
        previous=self.samples.copy(),self.bearings.copy(),self.result,self.streak,self.label
        if self.label!=name:self.reset();self.label=name
        latest=self._latest_stamp()
        if stamp<=latest or source<=latest or source in self._source_frames():return False
        if stamp-latest>self.max_gap:self.streak=0
        self._trim(stamp)
        self.bearings.append(dict(stamp=stamp,source_frame=source,origin=origin,
            bearing=angle,sigma=max(math.radians(1.),sigma),position_sigma=max(.02,position_sigma),
            confidence=confidence,depth_bounds_m=bounds))
        result=self._posterior()
        if result is None:
            self.samples,self.bearings,self.result,self.streak,self.label=previous
            return False
        self.streak+=1
        self.result=result
        return True

    def update(self,d,now):
        if self.factor is not None:
            # Raw floor projections must not re-enter a joint bearing likelihood.
            return False
        if self.frozen is not None or self.grid is None or not len(self.points):return False
        try:
            stamp,x,y,sigma,confidence=[float(d[k]) for k in ('stamp','x','y','sigma','confidence')]
            source=float(d.get('source_frame',stamp))
            name=d['label']
            if (not all(math.isfinite(v) for v in (stamp,source,x,y,sigma,confidence)) or sigma<=0
                    or not math.isfinite(sigma*sigma) or not 0<=now-stamp<=self.max_age
                    or not 0<=now-source<=self.max_age or source>stamp
                    or not self.confidence<=confidence<=1 or not isinstance(name,str)):
                return False
            covariance=np.asarray(d.get('covariance',np.eye(2)*sigma*sigma),dtype=float)
            if covariance.shape!=(2,2) or not np.isfinite(covariance).all() or not np.allclose(covariance,covariance.T):return False
            if np.linalg.eigvalsh(covariance).min()<=0:return False
        except (ValueError,TypeError,KeyError):return False
        previous=self.samples.copy(),self.bearings.copy(),self.result,self.streak,self.label
        if self.label!=name:
            self.reset();self.label=name
        latest=self._latest_stamp()
        if stamp<=latest or source<=latest or source in self._source_frames():return False
        if stamp-latest>self.max_gap:self.streak=0
        self._trim(stamp)
        if self.result is not None and self.samples:
            delta=np.array([x,y])-np.asarray(self.result['likelihood_mean'])
            innovation=np.asarray(self.result['likelihood_covariance'])+covariance
            if float(delta@np.linalg.solve(innovation,delta))>16.:
                self.streak=0
                return False
        self.samples.append((stamp,x,y,covariance,confidence,source))
        result=self._posterior()
        if result is None:
            self.samples,self.bearings,self.result,self.streak,self.label=previous
            return False
        self.streak+=1
        self.result=result
        return True

    def _posterior(self):
        mean=likelihood=None
        if self.samples:
            positions=np.array([[s[1],s[2]] for s in self.samples])
            covariances=np.array([s[3] for s in self.samples])
            precisions=np.linalg.inv(covariances*(1-self.common_fraction))
            independent=np.linalg.inv(precisions.sum(axis=0))
            mean=independent@np.einsum('nij,nj->i',precisions,positions)
            common=max(np.linalg.eigvalsh(c).max() for c in covariances)*self.common_fraction
            residual=positions-mean
            # Retain model-mismatch scatter, rather than dividing it away with N.
            likelihood=independent+np.eye(2)*common+(residual.T@residual)/len(residual)
            delta=self.points-mean
            quadratic=np.einsum('ni,ij,nj->n',delta,np.linalg.inv(likelihood),delta)
            # Broad likelihoods are uninformative, not evidence of a map conflict.
            support_covariance=likelihood+np.eye(2)*self.resolution**2/12
            support_distance=float(np.min(np.einsum('ni,ij,nj->n',delta,np.linalg.inv(support_covariance),delta)))
            log_likelihood=-quadratic/2
        if self.bearings:
            ray_log,ray_distance=self._bearing_likelihood()
            if self.samples:
                # Covariance intersection: floor and rays from different video
                # frames still share camera/pose bias. Do not multiply them as
                # independent sensors merely because the stamps differ.
                log_likelihood=(log_likelihood+ray_log)/2
                support_distance=max(support_distance,ray_distance)
            else:
                log_likelihood=ray_log
                support_distance=ray_distance
        # Broad likelihoods are uninformative, not evidence of a map conflict.
        # Cell quantization also prevents rejecting a precise point between centres.
        log_weights=np.log(self.prior)+log_likelihood
        normalizer=logsumexp(log_weights)
        if not math.isfinite(float(normalizer)):return None
        weights=np.exp(log_weights-normalizer)
        posterior_mean=weights@self.points
        centered=self.points-posterior_mean
        covariance=(centered.T*weights)@centered+np.eye(2)*self.resolution**2/12
        point=posterior_mean.copy()
        r,c=self.cell(*point)
        projected=not (0<=r<self.grid.shape[0] and 0<=c<self.grid.shape[1] and self.prior_grid[r,c]>0)
        if projected:
            point=self.points[np.argmin(np.sum((self.points-point)**2,axis=1))].copy()
            shift=point-posterior_mean
            covariance+=np.outer(shift,shift)
        if likelihood is not None:
            floor_normalizer=logsumexp(np.log(self.prior)-quadratic/2)
            log_mass=floor_normalizer+2*math.log(self.resolution)-math.log(2*math.pi)-np.linalg.slogdet(likelihood)[1]/2
            prior_mass=float(math.exp(min(0.,log_mass)))
        else:
            prior_mass=float(np.sum(self.prior*np.exp(ray_log-np.max(ray_log)))/self.prior.sum())
        frames=sorted(self._source_frames())
        confidence=[s[4] for s in self.samples]+[s['confidence'] for s in self.bearings]
        stamp=self._latest_stamp()
        position_confidence=float(np.clip(weights[np.sum((self.points-point)**2,axis=1)<=.2**2].sum(),0.,1.))
        variance=max(1e-8,float(np.linalg.eigvalsh(covariance).max()))
        return dict(x=float(point[0]),y=float(point[1]),sigma=math.sqrt(variance),variance=variance,
            covariance=covariance.tolist(),posterior_mean=posterior_mean.tolist(),projected=projected,
            likelihood_mean=(mean if mean is not None else posterior_mean).tolist(),
            likelihood_covariance=(likelihood if likelihood is not None else covariance).tolist(),
            prior_mass=prior_mass,prior_conflict=bool(support_distance>25.),support_mahalanobis_squared=support_distance,
            prior_scope='loaded map extent; unknown cells retained; uninflated connected space',
            map_signature=self.signature,label=self.label,method='map-constrained MMSE',
            stamp=float(stamp),estimate_id=self.signature[:12]+':'+format(stamp,'.9f'),
            samples=len(self.samples)+len(self.bearings),confidence=float(np.mean(confidence)),
            source='floor_projection' if self.samples else 'bearing_only',source_frames=frames,
            first_source_stamp=float(min(frames)),last_source_stamp=float(max(frames)),
            position_confidence=position_confidence,confidence_radius_m=.2,
            position_confidence_scope='normalized discrete posterior conditional on loaded map support')

    def _bearing_likelihood(self):
        sites=[]
        for sample in self.bearings:
            site=next((s for s in sites if np.linalg.norm(s[-1]['origin']-sample['origin'])<.08),None)
            if site is None:sites.append([sample])
            else:site.append(sample)
        total_log=np.zeros(len(self.points));total_distance=np.zeros(len(self.points))
        for site in sites:
            origins=np.array([s['origin'] for s in site])
            origin=origins.mean(axis=0)
            angles=np.array([s['bearing'] for s in site])
            angle=math.atan2(np.sin(angles).mean(),np.cos(angles).mean())
            scatter=np.arctan2(np.sin(angles-angle),np.cos(angles-angle))
            angular_variance=max(s['sigma']**2 for s in site)+float(np.mean(scatter**2))
            position_variance=max(s['position_sigma']**2 for s in site)+float(np.mean(np.sum((origins-origin)**2,axis=1)))
            delta=self.points-origin
            squared_range=np.sum(delta**2,axis=1)
            residual=np.arctan2(np.sin(np.arctan2(delta[:,1],delta[:,0])-angle),
                                np.cos(np.arctan2(delta[:,1],delta[:,0])-angle))
            variance=angular_variance+position_variance/np.maximum(squared_range,self.resolution**2/12)
            distance=residual**2/variance
            log_likelihood=-distance/2-np.log(variance)/2
            supported=delta@np.array([math.cos(angle),math.sin(angle)])>0
            for sample in site:
                bounds=sample['depth_bounds_m']
                if bounds is not None:
                    supported&=(squared_range>=bounds[0]**2)&(squared_range<=bounds[1]**2)
            log_likelihood[~supported]=-np.inf
            distance[~supported]=np.inf
            total_log+=log_likelihood;total_distance+=distance
        # Unknown cross-site correlation is conservatively bounded by a
        # geometric mean, until the separately refitted joint factor replaces
        # these raw rays. More frames cannot create extra independent sensors.
        return total_log/len(sites),float(np.min(total_distance/len(sites)))

    def snapshot(self,now):
        source=self.frozen or self.result
        if source is None:return None
        age=now-source['stamp']
        if self.frozen is None and not 0<=age<=30:return None
        return dict(source,age=max(0.,age),stale=self.frozen is None and age>self.max_age,
            stable=self.frozen is not None or (self.streak>=self.frames and (len(self.samples)+len(self.bearings)>=self.frames or self.factor is not None) and 0<=age<=self.max_age and not source['prior_conflict']),
            frames=source.get('frames',source.get('confirmation_frames',self.streak)) if self.frozen is not None else self.streak,confirmed=self.frozen is not None,frame_id='map')

    def freeze(self,estimate):
        if estimate.get('map_signature')!=self.signature:return
        self.frozen=json.loads(json.dumps(estimate))
