"""Map-referenced bottle calibration. Motion never uses image-centre error."""
import math
from datetime import datetime, timezone

import numpy as np
from scipy.optimize import least_squares

from .calibration import (number, validate_setup, validate_sample, predicted_points,
                          observed_points, floor_point)


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def map_sample(setup, target, pose, box):
    dx, dy = target[0]-pose[0], target[1]-pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return dict(distance=c*dx+s*dy-setup['camera_x'],
                lateral=-s*dx+c*dy-setup['camera_y'], box=list(box))


def free_segment(grid, a, b, radius=.24):
    if not grid or grid['resolution'] <= 0:
        return False
    resolution = grid['resolution']
    steps = max(2, int(math.dist(a[:2], b[:2])/min(.025,resolution/2))+2)
    centres = np.linspace(a[:2], b[:2], steps)
    offsets = np.arange(-radius, radius+resolution/2, resolution/2)
    x,y = np.meshgrid(offsets, offsets)
    offsets = np.c_[x.ravel(),y.ravel()]
    offsets = offsets[np.linalg.norm(offsets,axis=1)<=radius+resolution/3]
    points = (centres[:,None,:]+offsets[None,:,:]).reshape(-1,2)-grid['origin'][:2]
    angle = grid['origin'][2]
    c,s = math.cos(angle),math.sin(angle)
    col = np.floor((c*points[:,0]+s*points[:,1])/resolution).astype(int)
    row = np.floor((-s*points[:,0]+c*points[:,1])/resolution).astype(int)
    if np.any(col<0) or np.any(row<0) or np.any(col>=grid['width']) or np.any(row>=grid['height']):
        return False
    return bool(np.all(np.asarray(grid['cells'])[row*grid['width']+col]==0))


def make_plan(grid, pose, target):
    if not isinstance(target, (list, tuple)) or len(target) != 2:
        raise ValueError('Select one bottle map point with x and y coordinates')
    target = [number(v,'Bottle map coordinate',-10000,10000) for v in target]
    radius = math.dist(pose[:2],target)
    if not .4<=radius<=1.5:
        raise ValueError('Mark a bottle position 0.40–1.50 m from the robot')
    if not free_segment(grid,target,target,radius=.025):
        raise ValueError('Select a free floor position inside the saved map')
    bearing = math.atan2(target[1]-pose[1],target[0]-pose[0])
    near = max(.65,radius)
    def goal(distance,offset=0.,holdout=False):
        return dict(pose=[target[0]-distance*math.cos(bearing),target[1]-distance*math.sin(bearing),
                          wrap(bearing+offset)],holdout=holdout)
    goals = [goal(near+d,a) for d in (0.,.2,.4,.6,.8) for a in (0.,-.18,.18)]
    goals += [goal(near+.5,holdout=True),goal(near+.3,holdout=True)]
    previous=pose
    for entry in goals:
        if not free_segment(grid,previous,entry['pose']):
            raise ValueError('The planned calibration route crosses occupied, unknown or unmapped floor; choose a clearer location')
        previous=entry['pose']
    return dict(target=target,goals=goals,clearance_m=.24,reverse_extent_m=near+.8-radius,
                range_m=[near,near+.8])


def fit_map_profile(setup,camera,target,views,pitch_hint_deg=14.):
    setup=validate_setup(setup)
    target=[number(v,'Bottle map coordinate',-10000,10000) for v in target]
    training=[v for v in views if not v.get('holdout')]
    checks=[v for v in views if v.get('holdout')]
    if len(training)<10 or len(checks)<2:
        raise ValueError('Collect the planned training views and both independent check views')
    samples=[]
    for view in views:
        pose=np.asarray(view['pose'],dtype=float)
        if pose.shape!=(3,) or not np.isfinite(pose).all():raise ValueError('Invalid timestamped map pose')
        sample=map_sample(setup,target,pose,view['box'])
        samples.append(validate_sample(sample,camera))
    train=[sample for view,sample in zip(views,samples) if not view.get('holdout')]
    holdouts=[sample for view,sample in zip(views,samples) if view.get('holdout')]
    distances=[s['distance'] for s in train]
    bearings=[math.atan2(s['lateral'],s['distance']) for s in train]
    if max(distances)-min(distances)<.6 or max(bearings)-min(bearings)<.2:
        raise ValueError('Insufficient actual distance or angle coverage; profile was not saved')
    w,h=camera['width'],camera['height']
    weights=np.array([[1.,1.],[1.,1.],[.35,0.],[.35,0.]])
    def residual(p):
        geometry={**setup,'camera_height':p[4]}
        return np.concatenate([((predicted_points(p[:4],geometry,camera,s)-observed_points(s['box']))*weights).ravel() for s in train])
    scale=np.array([w,h,1.,1.,.3])
    bounds=([w*.2,h*.2,-.5,-.4,.05],[w*4.,h*5.,1.,.4,1.])
    initial=np.array([camera['k'][0],camera['k'][4],math.radians(pitch_hint_deg),0.,setup['camera_height']])
    initial=np.clip(initial,np.array(bounds[0])+1e-5,np.array(bounds[1])-1e-5)
    result=least_squares(residual,initial,bounds=bounds,loss='soft_l1',f_scale=2.,x_scale=scale,max_nfev=1000)
    geometry={**setup,'camera_height':float(result.x[4])}
    errors=np.array([np.linalg.norm(predicted_points(result.x[:4],geometry,camera,s)[:2]-observed_points(s['box'])[:2],axis=1).max() for s in train])
    rms=float(np.sqrt(np.mean(errors**2)));condition=float(np.linalg.cond(result.jac*scale))
    if not result.success or np.any(result.active_mask) or not math.isfinite(condition) or condition>1e4 or rms>8 or errors.max()>16:
        raise ValueError(f'Map calibration rejected: {rms:.1f} px error; check bottle dimensions, marked position and localization')
    k=list(camera['k']);k[0],k[4]=map(float,result.x[:2])
    profile=dict(schema=1,kind='measured_object_floor',created_utc=datetime.now(timezone.utc).isoformat(),
        camera={**camera,'k':k},setup=geometry,pitch=float(result.x[2]),yaw=float(result.x[3]),
        base_frame='base_footprint',training=train,validation=[],validated=False,
        range_m=[min(distances),max(distances)],bearing_rad=[min(bearings),max(bearings)],
        quality=dict(reprojection_rms_px=rms,max_error_px=float(errors.max()),condition=condition),
        map_calibration=dict(method='known_map_point',target_xy=target,observations=views,
            validation_source='held-out map poses; not independent surveyed accuracy'))
    for sample in holdouts:
        x1,_,x2,y2=sample['box'];point=floor_point(profile,[(x1+x2)/2,y2])
        expected=np.array([sample['distance']+setup['camera_x'],sample['lateral']+setup['camera_y']])
        error=float(np.linalg.norm(point[:2]-expected))
        profile['validation'].append({**sample,'estimated_base_xy':point[:2].tolist(),'error_m':error,'passed':error<=.1})
    if not all(v['passed'] for v in profile['validation']):raise ValueError('Held-out floor checks failed; profile was not saved')
    return profile


class MapCalibrationRun:
    """Verify hardware yaw polarity, then visit fixed map poses. No image servo."""
    def __init__(self,now,pose,odom,plan):
        self.started=self.changed=now;self.phase='probe_positive';self.index=0
        self.plan=plan;self.start_pose=list(pose);self.probe_yaw=odom[2]
        self.probe_deltas=[];self.yaw_sign=None;self.stable_since=None
        self.reason='Verifying turn response: small bounded turn'
        self.goal_distance=None;self.goal_heading_error=None
        self.turn_watch=None

    def tick(self,now,pose,odom,capture_ready):
        if now-self.started>300:raise ValueError('Overall calibration deadline reached')
        if math.dist(pose[:2],self.start_pose[:2])>1.2:raise ValueError('Robot left the calibration area')
        if self.phase.startswith('probe_'):
            delta=wrap(odom[2]-self.probe_yaw)
            if abs(delta)>.18:raise ValueError('Turn-response verification exceeded its rotation limit')
            if self.phase in ('probe_positive','probe_negative'):
                if abs(delta)>=.045 and now-self.changed>=.3 or now-self.changed>=1.4:
                    self.probe_deltas.append(delta);self.phase='probe_settle';self.changed=now
                    return 0.,0.,False
                return 0.,(.10 if self.phase=='probe_positive' else -.10),False
            if now-self.changed<.4:return 0.,0.,False
            if len(self.probe_deltas)==1:
                self.phase='probe_negative';self.probe_yaw=odom[2];self.changed=now
                return 0.,0.,False
            a,b=self.probe_deltas
            if abs(a)<.025 or abs(b)<.025 or a*b>=0:
                raise ValueError('Could not verify consistent turn direction; no calibration motion continued')
            self.yaw_sign=1. if a>0 else -1.
            self.phase='position';self.changed=now
        if self.index>=len(self.plan['goals']):
            self.phase='done';return 0.,0.,False
        if now-self.changed>40:raise ValueError('Map positioning/capture timed out; inspect clearance, pose and bottle visibility')
        goal=self.plan['goals'][self.index]['pose']
        dx,dy=goal[0]-pose[0],goal[1]-pose[1]
        distance=math.hypot(dx,dy);angle=wrap(goal[2]-pose[2])
        self.goal_distance,self.goal_heading_error=distance,angle
        if self.phase=='capture':
            if distance>.075 or abs(angle)>.11:
                self.phase='position';self.stable_since=None
            else:
                self.reason=f'Capturing view {self.index+1}/{len(self.plan["goals"])}'
                return 0.,0.,capture_ready and now-self.stable_since>=.35
        self.reason=f'Positioning for view {self.index+1}/{len(self.plan["goals"])} using the map'
        if distance>.05:
            bearing=math.atan2(dy,dx);ahead=wrap(bearing-pose[2]);reverse=abs(ahead)>math.pi/2
            aim=wrap(bearing+(math.pi if reverse else 0));turn=wrap(aim-pose[2])
            if abs(turn)>.06:
                self.check_turn(now,turn)
                return 0.,self.yaw_sign*math.copysign(min(.12,max(.055,abs(turn)*.65)),turn),False
            self.turn_watch=None
            return (-.035 if reverse else .035),self.yaw_sign*turn*.5,False
        if abs(angle)>.05:
            self.check_turn(now,angle)
            return 0.,self.yaw_sign*math.copysign(min(.12,max(.055,abs(angle)*.65)),angle),False
        self.turn_watch=None
        self.phase='capture';self.stable_since=now
        return 0.,0.,False

    def check_turn(self,now,error):
        direction=math.copysign(1.,error)
        if self.turn_watch is None or self.turn_watch[0]!=self.index or self.turn_watch[1]!=direction:
            self.turn_watch=(self.index,direction,now,abs(error))
        elif now-self.turn_watch[2]>.6 and abs(error)>self.turn_watch[3]+.10:
            raise ValueError('Heading error is increasing; stopped before continuing in the wrong direction')

    def captured(self,now):
        self.turn_watch=None
        self.index+=1;self.phase='position';self.changed=now;self.stable_since=None


class ControlLease:
    """Browser-owned motion session; Stop invalidates every earlier request."""
    def __init__(self):
        self.mode='idle';self.reason='Stopped';self.generation=0;self.lease=-math.inf

    def stop(self,reason='Stopped'):
        self.mode='idle';self.reason=reason;self.generation+=1

    def renew(self,now,generation):
        if generation==self.generation and self.mode=='auto':self.lease=now


def complete_candidates(candidates,camera):
    """Reject clipped boxes; merge overlapping detector proposals for one object."""
    accepted=[]
    for candidate in sorted(candidates,key=lambda c:c.get('confidence') or 0.,reverse=True):
        x1,y1,x2,y2=candidate['box']
        if (not np.isfinite([x1,y1,x2,y2]).all() or x1<2 or y1<2
                or x2>camera['width']-2 or y2>camera['height']-2 or x2-x1<8 or y2-y1<20):continue
        duplicate=False
        for other in accepted:
            a,b,c,d=other['box'];intersection=max(0,min(x2,c)-max(x1,a))*max(0,min(y2,d)-max(y1,b))
            union=(x2-x1)*(y2-y1)+(c-a)*(d-b)-intersection
            if intersection/union>.65:duplicate=True;break
        if not duplicate:accepted.append(candidate)
    return accepted
