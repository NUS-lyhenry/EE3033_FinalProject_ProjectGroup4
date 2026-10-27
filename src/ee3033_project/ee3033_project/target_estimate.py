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
        self.signature=None
        self.grid=self.points=self.prior=self.component_grid=None
        self.component=None
        self.reset()

    def reset(self):
        self.samples.clear()
        self.streak=0
        self.result=None
        self.frozen=None
        self.label=None

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
        if self.samples and stamp<self.samples[-1][0]:return
        if not deferred or not self.samples or stamp-self.samples[-1][0]>self.max_gap:
            self.streak=0

    def update(self,d,now):
        if self.frozen is not None or self.grid is None or not len(self.points):return False
        try:
            stamp,x,y,sigma,confidence=[float(d[k]) for k in ('stamp','x','y','sigma','confidence')]
            name=d['label']
            if (not all(math.isfinite(v) for v in (stamp,x,y,sigma,confidence)) or sigma<=0
                    or not math.isfinite(sigma*sigma) or not 0<=now-stamp<=self.max_age
                    or not self.confidence<=confidence<=1 or not isinstance(name,str)):
                return False
            covariance=np.asarray(d.get('covariance',np.eye(2)*sigma*sigma),dtype=float)
            if covariance.shape!=(2,2) or not np.isfinite(covariance).all() or not np.allclose(covariance,covariance.T):return False
            if np.linalg.eigvalsh(covariance).min()<=0:return False
        except (ValueError,TypeError,KeyError):return False
        if self.label!=name:
            self.reset();self.label=name
        if self.samples and stamp<=self.samples[-1][0]:return False
        while self.samples and stamp-self.samples[0][0]>self.seconds:self.samples.popleft()
        if self.samples and stamp-self.samples[-1][0]>self.max_gap:self.streak=0
        if self.result is not None and self.samples:
            delta=np.array([x,y])-np.asarray(self.result['likelihood_mean'])
            innovation=np.asarray(self.result['likelihood_covariance'])+covariance
            if float(delta@np.linalg.solve(innovation,delta))>16.:
                self.streak=0
                return False
        self.samples.append((stamp,x,y,covariance,confidence))
        self.streak+=1
        self.result=self._posterior()
        return True

    def _posterior(self):
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
        log_weights=np.log(self.prior)-quadratic/2
        normalizer=logsumexp(log_weights)
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
        log_mass=normalizer+2*math.log(self.resolution)-math.log(2*math.pi)-np.linalg.slogdet(likelihood)[1]/2
        variance=max(1e-8,float(np.linalg.eigvalsh(covariance).max()))
        return dict(x=float(point[0]),y=float(point[1]),sigma=math.sqrt(variance),variance=variance,
            covariance=covariance.tolist(),posterior_mean=posterior_mean.tolist(),projected=projected,
            likelihood_mean=mean.tolist(),likelihood_covariance=likelihood.tolist(),
            prior_mass=float(math.exp(min(0.,log_mass))),prior_conflict=bool(log_mass<math.log(1e-5)),
            prior_scope='loaded map extent; unknown cells retained; uninflated connected space',
            map_signature=self.signature,label=self.label,method='map-constrained MMSE',
            stamp=float(self.samples[-1][0]),estimate_id=self.signature[:12]+':'+format(self.samples[-1][0],'.9f'),
            samples=len(self.samples),confidence=float(np.mean([s[4] for s in self.samples])))

    def snapshot(self,now):
        source=self.frozen or self.result
        if source is None:return None
        age=now-source['stamp']
        if self.frozen is None and not 0<=age<=30:return None
        return dict(source,age=max(0.,age),stale=self.frozen is None and age>self.max_age,
            stable=self.frozen is not None or (self.streak>=self.frames and 0<=age<=self.max_age and not source['prior_conflict']),
            frames=self.streak,confirmed=self.frozen is not None,frame_id='map')

    def freeze(self,estimate):
        if estimate.get('map_signature')!=self.signature:return
        self.frozen=json.loads(json.dumps(estimate))
