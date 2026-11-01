"""One authoritative map posterior for both the UI and mission acceptance."""
import json
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from rclpy.node import Node
from .core import quaternion_yaw
from .ros_utils import LATCHED, parameters, run
from .bottom_range import SettledBatch, single_observation
from .target_estimate import MapMMSE
from .triangulation import BearingIntersection
from .target_acquisition import SingleTarget
from .localization_diagnostics import DiagnosticHistory, mmse_rejection_reason


class TargetFilter(Node):
    def __init__(self, **kwargs):
        super().__init__('project_target_filter', **kwargs)
        self.p=parameters(self,dict(confirmation_frames=4,confidence=.60,max_frame_gap=1.2,
            max_observation_age=2.,map_frame='map',target_class='',posterior_window=8.,
            posterior_samples=30,common_error_fraction=.8,unknown_prior_weight=.25))
        self.tracker=MapMMSE(frames=self.p['confirmation_frames'],seconds=self.p['posterior_window'],
            max_samples=self.p['posterior_samples'],max_gap=self.p['max_frame_gap'],
            max_age=self.p['max_observation_age'],confidence=self.p['confidence'],
            common_fraction=self.p['common_error_fraction'],unknown_weight=self.p['unknown_prior_weight'])
        self.bottom_batch=SettledBatch()
        self.active=False
        self.target_locked=False
        self.association=SingleTarget()
        self.intersection=BearingIntersection()
        self.geometry_conflict=False
        self.armed_at=0.
        self.seed=None
        self.mission_stamp=None
        self.search_epoch=0
        self.epoch_started_stamp=0.
        self.last_frame_stamp=0.
        self.diagnostics=DiagnosticHistory()
        self.diagnostics_pub=self.create_publisher(String,'/project/localization_diagnostics',20)
        self.pose_pub=self.create_publisher(PoseWithCovarianceStamped,'/project/confirmed_target',10)
        self.details_pub=self.create_publisher(String,'/project/target_details',10)
        self.estimate_pub=self.create_publisher(String,'/project/target_estimate',LATCHED)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,LATCHED)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,10)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.on_initial_pose,10)
        self.create_subscription(Bool,'/project/search_active',self.on_active,LATCHED)
        self.create_subscription(String,'/project/target_observations',self.on_frame,10)
        self.create_subscription(String,'/project/mission_status',self.on_status,LATCHED)
        self.create_subscription(String,'/project/mission_result',self.on_result,LATCHED)
        self.create_timer(.5,self.publish)

    def now(self):return self.get_clock().now().nanoseconds*1e-9

    def on_map(self,msg):
        if msg.header.frame_id!=self.p['map_frame']:return
        try:
            grid=np.array(msg.data).reshape(msg.info.height,msg.info.width)
            pose=msg.info.origin
            previous_signature=self.tracker.signature
            self.tracker.set_map(grid,msg.info.resolution,[pose.position.x,pose.position.y,quaternion_yaw(pose.orientation)])
            if previous_signature != self.tracker.signature:
                self.armed_at=max(self.armed_at,self.now())
                self.intersection.reset();self.association.reset();self.bottom_batch.reset()
            if self.seed:self.tracker.set_seed(*self.seed)
            self.publish()
        except (ValueError,TypeError) as exc:self.get_logger().warning('MMSE map unavailable: '+str(exc))

    def on_pose(self,msg):
        if msg.header.frame_id!=self.p['map_frame']:return
        p=msg.pose.pose.position
        if math.isfinite(p.x) and math.isfinite(p.y):
            self.seed=(p.x,p.y)
            self.tracker.set_seed(*self.seed)

    def on_initial_pose(self,msg):
        self.armed_at=max(self.armed_at,self.now())
        self.tracker.reset()
        self.bottom_batch.reset()
        self.intersection.reset()
        self.association.reset()
        self.on_pose(msg)
        self.publish()

    def on_active(self,msg):
        self.active=msg.data
        self.publish()

    def on_frame(self,msg):
        now=self.now()
        event=dict(schema_version=1,stamp=None,published_stamp=now,mission_stamp=self.mission_stamp,search_epoch=getattr(self,'search_epoch',0),
            target_locked=self.target_locked,reason_codes=[],frame={},selected=None,
            association=dict(attempted=False,matched=False),intersection=dict(attempted=False),
            mmse=dict(attempted=False,updated=False,stable=False))
        codes=event['reason_codes']
        try:
            data=json.loads(msg.data);stamp=float(data['stamp'])
            event.update(stamp=stamp,frame_age=now-stamp)
            event['frame']=dict(data.get('diagnostics') or {})
            event['frame'].update(observation_count=len(data.get('observations',[])),
                bearing_count=len(data.get('bearings',[])),deferred=data.get('deferred') is True)
            codes.extend(event['frame'].get('reason_codes',[]))
            for detection in event['frame'].get('detections',[]):codes.extend(detection.get('reason_codes',[]))
            if not math.isfinite(stamp) or not 0<=now-stamp<=self.p['max_observation_age']:
                codes.append('frame_stale_future_or_invalid');return
            if stamp<self.armed_at:
                codes.append('frame_before_current_mission');return
            if stamp<=self.last_frame_stamp:
                codes.append('frame_duplicate_or_reordered');return
            self.last_frame_stamp=stamp
            bottle=event['frame'].get('localization_method')=='bottom_inverse'
            if getattr(self,'search_epoch',0)>0 and not self.target_locked and not bottle:
                codes.append('awaiting_current_epoch_target_lock')
                return
            candidates=[o for o in data['observations'] if o.get('stamp')==stamp and (not self.p['target_class'] or o.get('label')==self.p['target_class'])]
            candidates.sort(key=lambda o:o.get('confidence',0),reverse=True)
            bearings = [d for d in data.get('bearings', []) if d.get('stamp') == stamp
                        and (not self.p['target_class'] or d.get('label') == self.p['target_class'])]
            if event['frame'].get('localization_method') == 'bottom_inverse':
                self.on_bottom_frame(candidates,bearings,stamp,now,event)
                return
            updated=False
            selected=factor=None
            bearing_accepted=False
            if self.tracker.factor and now-self.tracker.factor['stamp'] > self.p['max_observation_age']:
                self.tracker.reset()
                codes.append('mmse_intersection_expired_recollecting')
            if self.target_locked and bearings:
                locked=self.association.update(bearings, now, self.armed_at)
                selected=self.association.candidate
                matched=bool(selected and selected['stamp']==stamp)
                event['association']=dict(attempted=True,matched=matched,locked=locked,
                    frames=self.association.frames,candidate_count=len(bearings))
                if not matched:codes.append('bearing_association_rejected')
                if selected and selected['stamp'] == stamp:
                    event['selected']=dict(selected)
                    candidates=[o for o in candidates if o.get('box')==selected.get('box')]
                    factor=self.intersection.add(selected)
                    bearing_accepted=self.intersection.frame_accepted
                    event['intersection']=dict(attempted=True,accepted=factor is not None,
                        frame_accepted=self.intersection.frame_accepted,reason=self.intersection.reason,
                        reason_code=self.intersection.reason_code)
                    codes.extend(self.intersection.last_events)
                    if factor:
                        event['intersection']['factor']=factor
                        self.geometry_conflict=False
                        if candidates:
                            floor=candidates[0]
                            distance=math.hypot(floor['x']-factor['x'], floor['y']-factor['y'])
                            self.geometry_conflict=distance>max(.20,3*math.hypot(floor['sigma'],factor['sigma']))
                        if self.geometry_conflict:codes.append('bearing_floor_geometry_conflict')
                # Floor and bearing factors must refer to the same detection,
                # not simply whichever same-class box has highest confidence.
                if selected and selected['stamp']==stamp and 'box' in selected:
                    candidates=[o for o in candidates if o.get('box')==selected['box']]
                elif selected is None or selected['stamp']!=stamp:
                    candidates=[]
            elif self.target_locked:
                codes.append('no_bearing_measurement')
            elif bearings:
                codes.append('bearing_waiting_for_target_lock')
            if candidates:
                event['mmse']['attempted']=True
                floor=candidates[0]
                updated=self.tracker.update(floor,now)
                event['mmse']['floor_updated']=updated
                if not updated:
                    codes.append(mmse_rejection_reason(self.tracker,floor,now))
                    event['mmse']['floor_rejection']=self.tracker.last_update_rejection
            else:codes.append('no_usable_floor_measurement')
            if factor and not self.geometry_conflict:
                event['mmse']['attempted']=True
                accepted=self.tracker.update_factor(factor,now)
                if accepted:updated=True
                else:
                    codes.append('mmse_intersection_factor_rejected')
                    event['mmse']['factor_rejection']=self.tracker.last_factor_rejection
            # Preserve the robot's bearing-only posterior and source deduplication.
            if not updated and self.tracker.factor is None and bearing_accepted:
                event['mmse']['attempted']=True
                updated=self.tracker.update_bearing(selected,now)
                if not updated:codes.append('mmse_bearing_likelihood_rejected')
            if not updated:self.tracker.miss(stamp,data.get('deferred') is True)
            self.publish()
            estimate=self.tracker.snapshot(now)
            event['mmse'].update(updated=updated,stable=bool(estimate and estimate['stable']),
                source=estimate.get('source','floor_projection') if estimate else None,
                prior_conflict=bool(estimate and estimate.get('prior_conflict')),
                samples=estimate.get('samples',0) if estimate else 0,confirmation_streak=self.tracker.streak)
            if estimate and estimate.get('prior_conflict'):codes.append('mmse_map_prior_conflict')
            elif updated and not estimate['stable']:codes.append('mmse_collecting_confirmation')
            elif updated:codes.append('mmse_stable_estimate')
            if self.active and updated and estimate and estimate['stable']:
                pose=PoseWithCovarianceStamped()
                pose.header.frame_id=self.p['map_frame']
                pose.header.stamp.sec=int(estimate['stamp'])
                pose.header.stamp.nanosec=int((estimate['stamp']-int(estimate['stamp']))*1e9)
                pose.pose.pose.position.x,pose.pose.pose.position.y=estimate['x'],estimate['y']
                pose.pose.pose.orientation.w=1.
                c=estimate['covariance']
                pose.pose.covariance[0],pose.pose.covariance[1]=c[0]
                pose.pose.covariance[6],pose.pose.covariance[7]=c[1]
                self.pose_pub.publish(pose)
                self.details_pub.publish(String(data=json.dumps(dict(estimate,mission_stamp=self.mission_stamp,search_epoch=getattr(self,'search_epoch',0)))))
        except (ValueError,TypeError,KeyError,AttributeError,IndexError,OverflowError,np.linalg.LinAlgError) as exc:
            codes.append('localization_frame_processing_error')
            event['error']=str(exc)
        finally:
            event['progress']=self.intersection.progress()
            event['geometry_conflict']=self.geometry_conflict
            event=self.diagnostics.add(event)
            self.diagnostics_pub.publish(String(data=json.dumps(event,allow_nan=False)))

    def on_bottom_frame(self, candidates, bearings, stamp, now, event):
        # Bearings retain identity and steering. They do not bypass calibrated
        # range confirmation, nor become a second factor from the same images.
        bearings=[d for d in bearings if d.get('search_eligible',True) and
            not any(math.dist(d.get('origin',(math.inf,math.inf)),p)<.20 for p in getattr(self,'failed_positions',[]))]
        # Both owners prefer a usable projection over an unusable high-confidence box.
        projected=[d for d in bearings if d.get('projection_valid') is True]
        if projected:bearings=projected
        self.association.update(bearings,now,self.armed_at)
        selected=self.association.candidate
        if selected and selected.get('stamp')==stamp:
            selected=next((d for d in bearings if d.get('stamp')==stamp and d.get('box')==selected.get('box')),selected)
            self.association.candidate=selected
        matched=bool(selected and selected['stamp']==stamp)
        event['association']=dict(attempted=True,matched=matched,locked=self.target_locked,
            frames=self.association.frames,candidate_count=len(bearings))
        event['selected']=selected if matched else None
        floor=next((o for o in candidates if matched and o.get('box')==selected.get('box')
            and o.get('method')=='bottom_inverse'),None)
        measurement=single_observation(floor,selected if matched else None,now,self.p.get('max_observation_age',2.))
        updated=bool(measurement and self.tracker.replace_visual_observation(measurement,now))
        codes=event['reason_codes']
        codes.append('bottom_single_observation_accepted' if updated else 'bottom_single_observation_unavailable')
        event['mmse'].update(attempted=measurement is not None,updated=updated,
            batch_frames=1 if updated else 0,required_frames=1,measurement_valid=updated)
        self.publish()
        estimate=self.tracker.snapshot(now)
        event['mmse']['stable']=bool(estimate and estimate['stable'])
        if self.active and updated and estimate and estimate.get('reportable'):
            pose=PoseWithCovarianceStamped()
            pose.header.frame_id=self.p['map_frame']
            pose.header.stamp.sec=int(estimate['stamp'])
            pose.header.stamp.nanosec=int((estimate['stamp']%1)*1e9)
            pose.pose.pose.position.x,pose.pose.pose.position.y=estimate['x'],estimate['y']
            pose.pose.pose.orientation.w=1.
            c=estimate['covariance']
            pose.pose.covariance[0],pose.pose.covariance[1]=c[0]
            pose.pose.covariance[6],pose.pose.covariance[7]=c[1]
            self.pose_pub.publish(pose)
            self.details_pub.publish(String(data=json.dumps(dict(estimate,mission_stamp=self.mission_stamp,search_epoch=getattr(self,'search_epoch',0)))))

    def on_status(self,msg):
        try:
            data=json.loads(msg.data);stamp=data.get('started_stamp')
            epoch=data.get('search_epoch',0)
            current_epoch=getattr(self,'search_epoch',0)
            if not stamp: return
            if self.mission_stamp is not None and (stamp<self.mission_stamp or
                    (stamp==self.mission_stamp and epoch<current_epoch)):
                return
            self.failed_positions=data.get('failed_localization_positions',[])
            changed=stamp!=self.mission_stamp or epoch!=current_epoch
            was_locked=self.target_locked
            self.target_locked=bool(data.get('target_detected'))
            if changed:
                self.mission_stamp=stamp
                self.search_epoch=epoch
                self.epoch_started_stamp=float(data.get('epoch_started_stamp',stamp))
                self.armed_at=self.epoch_started_stamp
                self.last_frame_stamp=0.
                self.tracker.reset()
                self.bottom_batch.reset()
                self.intersection.reset()
                self.association.reset()
                self.geometry_conflict=False
                self.diagnostics.reset()
            if self.target_locked and not was_locked:
                self.association.reset()
                self.bottom_batch.reset()
                self.intersection.reset()
                self.armed_at=max(self.armed_at,float(data.get('detection_started_stamp',self.armed_at)))
                seed=data.get('target_lock')
                if isinstance(seed,dict) and seed.get('stamp',0.)>=self.armed_at:
                    self.association.candidate=dict(seed)
                    self.association.frames=2
                    self.association.locked=True
            if self.target_locked:self.association.started=1.
            if changed:self.publish()
        except (ValueError,TypeError):pass

    def on_result(self,msg):
        try:
            data=json.loads(msg.data);target=data.get('target')
            if (data.get('search_epoch',0)!=getattr(self,'search_epoch',0)
                    or data.get('started_stamp',self.mission_stamp)!=self.mission_stamp): return
            if data.get('target_reported') is False or data.get('event')=='target_accepted':return
            if target and target.get('estimator')=='map-constrained MMSE':
                frozen=dict(target,label=target['class'],stamp=target['observation_stamp'],single_frame=target.get('single_frame',False),
                            method=target['estimator'],samples=1 if target.get('single_frame') else target.get('confirmation_frames',4),
                            frames=1 if target.get('single_frame') else target.get('confirmation_frames',4),prior_conflict=False)
                self.tracker.freeze(frozen)
                self.publish()
        except (ValueError,TypeError,KeyError):pass

    def publish(self):
        result=self.tracker.snapshot(self.now())
        if result:
            result['frame_id']=self.p['map_frame']
            result['mission_stamp']=self.mission_stamp
            result['geometry_conflict']=self.geometry_conflict
            result['intersection_reason']=self.intersection.reason
        result=result or dict(available=False,map_signature=self.tracker.signature)
        result['mission_stamp']=self.mission_stamp
        result['search_epoch']=getattr(self,'search_epoch',0)
        result['epoch_started_stamp']=getattr(self,'epoch_started_stamp',self.armed_at)
        result['bearing_progress']=self.intersection.progress()
        result['target_locked']=self.target_locked
        result['geometry_conflict']=self.geometry_conflict
        result['intersection_reason']=self.intersection.reason
        result['diagnostics_summary']=self.diagnostics.summary()
        self.estimate_pub.publish(String(data=json.dumps(result)))


def main():run(TargetFilter)
