"""Fixed-mount bottle ranging in ORIGINAL, unrectified 640x480 pixels.

The fitted distance is axle-to-base-centre ground range. Neither lens offset
nor bottle radius is added. Tilt telemetry rejects motion; it never corrects
this fit. All limits below are engineering assumptions, not accuracy claims.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np

MODEL = 'bottom_inverse_17pt_20261031'
A, B = 214.25159, -0.1282935
Y_MIN, Y_MAX = 188.70762634277344, 445.91217041015625
SIGMA_FLOOR = .02
CALIBRATION_K = [303.58978001, 0.0, 313.75809193, 0.0, 302.97064273, 258.71992369, 0.0, 0.0, 1.0]
CALIBRATION_D = [0.06801198, -0.0985489, -0.0005351, -0.00040454, 0.02653797]
MODEL_SHA256 = '9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef'


@dataclass
class BottomCamera:
    width: int
    height: int
    k: object
    d: object
    pixel_sigma: float = 2.
    margin: float = 8.
    # Discovery coverage is separate from the calibrated ranging envelope.
    min_range: float = .35
    max_range: float = .95
    center_tolerance: float = 32.
    source: str = MODEL

    def __post_init__(self):
        self.k = np.asarray(self.k, float).reshape(3, 3)
        # Only the bearing uses lens intrinsics. Bottom y is NEVER undistorted.
        self.d = np.asarray(self.d, float)
        if ((self.width, self.height) != (640, 480) or not np.isfinite(self.k).all()
                or not np.isfinite(self.d).all() or min(self.k[0,0], self.k[1,1]) <= 0
                or not np.isfinite([self.pixel_sigma, self.margin, self.min_range,
                    self.max_range, self.center_tolerance]).all()
                or not .35 <= self.min_range < self.max_range <= 1.
                or not 0 < self.pixel_sigma <= 10 or not 0 <= self.margin <= 20
                or not 0 < self.center_tolerance <= 32):
            raise ValueError('Bottom model requires original 640x480 image and valid CameraInfo')
        # This is a horizontal bearing convention, NOT a camera pitch model.
        self.rotation = np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        self.translation = np.zeros(3)  # the fit already includes the axle reference
        self.pitch_sigma = self.roll_sigma = 0.
        self.attitude_pivot = None

    @property
    def yaw(self): return 0.

    @property
    def fov(self): return 2*math.atan(self.width/(2*self.k[0,0]))

    def metadata(self):
        return dict(model=MODEL, width=self.width, height=self.height,
            k=self.k.tolist(), d=self.d.tolist(), pixel_sigma=self.pixel_sigma,
            margin=self.margin, min_range=self.min_range, max_range=self.max_range,
            center_tolerance=self.center_tolerance, source=self.source)

    def bearing_ray(self, box):
        # Horizontal angle only, independent of pitch or bottom y. For the
        # accepted +/-10 px window, no off-axis distance calibration is assumed.
        u=(float(box[0])+float(box[2]))/2
        return np.array([1., -(u-self.width/2)/self.k[0,0], 0.])

    def view_requirements(self, box):
        x1,y1,x2,y2=map(float,box)
        valid=np.isfinite([x1,y1,x2,y2]).all() and 0<=x1<x2<=640 and 0<=y1<y2<=480
        horizontal=not valid or x1<self.margin or x2>640-self.margin
        bottom=not valid or y2>480-self.margin
        centered=valid and abs((x1+x2)/2-320)<=self.center_tolerance
        raw_distance=A/y2+B if valid and y2>0 and not bottom else None
        near=valid and (bottom or y2>Y_MAX or (raw_distance is not None and raw_distance<.35))
        far=not near and (not valid or y2<Y_MIN or raw_distance is None or raw_distance>self.max_range)
        below_minimum=not near and not far and raw_distance<self.min_range
        state='too_near' if near else 'too_far' if far else 'below_selected_minimum' if below_minimum else 'within_range'
        reasons=[]
        if horizontal: reasons.append('horizontal_clipping')
        if bottom: reasons.append('bottom_clipping')
        if not centered: reasons.append('not_centered')
        if state!='within_range': reasons.append('bottom_'+state)
        retreat=horizontal  # A near/cropped contact may be estimated if retreat is unsafe.
        return dict(range_model=MODEL, range_state=state, range_hint_m=None,
            range_hint_is_measurement=False, camera_min_range=self.min_range, camera_max_range=self.max_range,
            range_m=((.12+.35)/2 if near else raw_distance) if valid and not far and not below_minimum else None,
            search_eligible=valid and not far and not below_minimum, ranging_mode='near_guess' if near else 'calibrated',
            centered=centered, center_error_px=(x1+x2)/2-320,
            reframe_required=retreat, reframe_reasons=reasons,
            reframe_action='move_away' if retreat else 'center' if not centered else
                'approach' if state=='too_far' else 'observe',
            calibrated_y_px=[Y_MIN,Y_MAX], distance_reference='wheel_axle_to_bottle_base_center')

    def project_box(self, box, coarse_max_range=None):
        x1,y1,x2,y2=map(float,box)
        view=self.view_requirements(box)
        if (not np.isfinite([x1,y1,x2,y2]).all() or not 0<=y1<y2<=480
                or not self.margin<=x1<x2<=640-self.margin or x2-x1<4 or y2-y1<8):
            raise ValueError('Invalid or too small bottle contact')
        offset=abs(view['center_error_px'])
        if not view['search_eligible']:
            raise ValueError('Target exceeds the selected camera detection range')
        ray=self.bearing_ray(box); ray/=np.linalg.norm(ray[:2])
        if view['ranging_mode']=='near_guess':
            # Bounded near-field prior, never inverse extrapolation from y=480.
            # Map fusion will restrict support to this near part of the ray.
            distance=(.12+.35)/2
            sigma=.12
        else:
            distance=A/y2+B
            sigma=math.hypot(SIGMA_FLOOR,A*self.pixel_sigma/y2**2)
        # Off-axis samples are uncalibrated estimates. Prefer centering, but
        # retain an explicit error penalty after a bounded improvement wait.
        sigma=math.hypot(sigma,max(0.,offset-10.)*.002)
        return distance*ray, sigma, distance

    def contains_floor(self, points):
        # Discovery coverage, not permission to produce an off-centre range.
        p=np.asarray(points,float).reshape(-1,2)
        distance=np.linalg.norm(p,axis=1)
        return ((distance>=self.min_range)&(distance<=self.max_range)&
            (np.abs(np.arctan2(p[:,1],p[:,0]))<=self.fov/2))


def attitude_window(samples, stamp, seconds=.6, max_gap=.15, max_span=math.radians(1.5)):
    """Reject capture-time oscillation without an old reference or tilt correction."""
    points=[p for p in samples if stamp-seconds-max_gap<=p[0]<=stamp]
    if len(points)<5 or points[-1][0]-points[0][0]<seconds or stamp-points[-1][0]>max_gap:
        raise ValueError('attitude_window_incomplete')
    if any(b[0]-a[0]>max_gap for a,b in zip(points,points[1:])):
        raise ValueError('attitude_window_gap')
    span=np.ptp(np.array([p[1:] for p in points]),axis=0)
    if np.max(span)>max_span: raise ValueError('attitude_oscillating')
    return dict(valid=True, mode='stability_only', span=span.tolist(),
        compensation=False, fixed_pose_bias_checked=False,
        limitation='Mount and stationary pose must still match calibration')


class SettledBatch:
    """Eight distinct same-target exposures from one stationary observation site.

    Robustly fuse *capture-time map positions*. Rolling windows replace prior
    evidence as a whole; shared image sources are never appended independently.
    """
    def __init__(self, frames=8, max_gap=1.2, seconds=8.):
        self.frames,self.max_gap,self.seconds=frames,max_gap,seconds
        self.samples=deque(maxlen=24)
        self.last_stamp=-math.inf
        self.improve_since=None

    def reset(self):
        self.samples.clear()
        self.improve_since=None

    def add(self, observation, detection):
        if observation is None or detection is None:
            self.reset(); return None
        stamp=float(observation['stamp'])
        if stamp<=self.last_stamp: return None
        self.last_stamp=stamp
        if (observation.get('method')!='bottom_inverse' or detection.get('settled') is not True
                or detection.get('projection_valid') is not True
                or observation.get('confidence',0)<.60):
            self.reset(); return None
        preferred=detection.get('centered') and detection.get('attitude',{}).get('valid')
        if not preferred:
            if self.improve_since is None: self.improve_since=stamp
            if stamp-self.improve_since<1.5: return None
        if self.samples:
            previous,first=self.samples[-1],self.samples[0]
            if (stamp-previous['stamp']>self.max_gap or observation['label']!=first['label']
                    or math.dist(detection['origin'],first['origin'])>.04
                    or abs(math.atan2(math.sin(detection.get('capture_heading',0)-first['heading']),
                        math.cos(detection.get('capture_heading',0)-first['heading'])))>.05):
                self.reset()
        self.samples.append(dict(observation,origin=detection['origin'],
            bearing=detection.get('bearing',0.),heading=detection.get('capture_heading',0),
            ranging_mode=detection.get('ranging_mode','calibrated'),
            quality_flags=list(observation.get('quality_flags',[]))))
        while self.samples and stamp-self.samples[0]['stamp']>self.seconds: self.samples.popleft()
        if len(self.samples)<self.frames: return None
        points=np.array([[p['x'],p['y']] for p in self.samples])
        median=np.median(points,axis=0)
        residual=np.linalg.norm(points-median,axis=1)
        mad=1.4826*np.median(np.abs(residual-np.median(residual)))
        accepted=[p for p,r in zip(self.samples,residual) if r<=max(.04,3*mad)]
        if len(accepted)<self.frames: return None
        accepted=accepted[-self.frames:]
        # The newest exposure must agree: an older good batch cannot be
        # restamped by a new outlier or used to manufacture freshness.
        if accepted[-1]['stamp']!=stamp: return None
        points=np.array([[p['x'],p['y']] for p in accepted])
        mean=np.median(points,axis=0); delta=points-mean
        covariance=np.eye(2)*max(SIGMA_FLOOR,max(p['sigma'] for p in accepted))**2+delta.T@delta/len(points)
        flags=sorted({flag for p in accepted for flag in p['quality_flags']})
        near=any(p['ranging_mode']=='near_guess' for p in accepted)
        result=dict(stamp=stamp,source_frame=stamp,source_frames=[p['stamp'] for p in accepted],
            x=float(mean[0]),y=float(mean[1]),covariance=covariance.tolist(),
            sigma=math.sqrt(float(np.linalg.eigvalsh(covariance).max())),
            confidence=min(p['confidence'] for p in accepted),label=observation['label'],
            source='bottom_inverse_batch',method='bottom_inverse',sites=1,
            engineering_sigma_floor=.12 if near else SIGMA_FLOOR,distinct_frames=len(accepted),
            quality_flags=flags,observation_quality='approximate' if flags or near else 'calibrated')
        angle=math.atan2(np.mean([math.sin(p['bearing']) for p in accepted]),
            np.mean([math.cos(p['bearing']) for p in accepted]))
        result.update(near_guess=near,origin=np.mean([p['origin'] for p in accepted],axis=0).tolist(),
            bearing=angle,depth_bounds_m=[.12,.35] if near else [float(detection.get('camera_min_range',.35)),float(detection.get('camera_max_range',.95))])
        return result


def single_observation(observation, detection, now, max_age=2.):
    """One real exposure; no settling gate, fusion or manufactured confirmation."""
    if observation is None or detection is None:return None
    try:
        stamp=float(observation['stamp'])
        if (observation.get('method')!='bottom_inverse' or detection.get('stamp')!=stamp
                or detection.get('projection_valid') is not True or detection.get('bearing_valid') is not True
                or not 0<=now-stamp<=max_age or observation.get('confidence',0)<.60):return None
        x,y,sigma=map(float,(observation['x'],observation['y'],observation['sigma']))
        if not np.isfinite([stamp,x,y,sigma]).all() or sigma<=0:return None
        pose=detection.get('capture_pose') or dict(stamp=stamp,x=detection['origin'][0],
            y=detection['origin'][1],yaw=detection['capture_heading'],frame_id='map')
        if pose.get('stamp')!=stamp or not np.isfinite([pose['x'],pose['y'],pose['yaw']]).all():return None
        near=detection.get('ranging_mode')=='near_guess'
        sigma=max(.12 if near else SIGMA_FLOOR,sigma)
        flags=sorted(set(observation.get('quality_flags',[]))|{'single_frame_estimate'})
        return dict(observation,x=x,y=y,sigma=sigma,covariance=(np.eye(2)*sigma*sigma).tolist(),
            source='bottom_inverse_single',source_frames=[stamp],source_frame=stamp,
            frames=1,samples=1,single_frame=True,measurement_valid=True,reportable=True,
            centered=detection.get('centered') is True,origin=detection['origin'],
            capture_pose=pose,bearing=detection['bearing'],
            capture_heading=detection.get('capture_heading'),
            center_error_px=detection.get('center_error_px'),quality_flags=flags,
            engineering_sigma_floor=.12 if near else SIGMA_FLOOR,near_guess=near,
            depth_bounds_m=[.12,.35] if near else [detection.get('camera_min_range',.35),detection.get('camera_max_range',.95)],
            observation_quality='approximate',stable=False,sites=1)
    except (KeyError,TypeError,ValueError,OverflowError):return None
