"""Read-only detection telemetry and provisional map display; no fusion writes."""
import math
from .localization_diagnostics import finite_json


def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)


class VisualTelemetry:
    def __init__(self): self.reset()

    def reset(self,minimum_stamp=0.):
        self.minimum_stamp=minimum_stamp
        self.frame=None
        self.last_detection=None
        self.preview=None
        self.diagnostic=None

    def ingest(self,data,now,target_class):
        stamp=data.get('stamp')
        if (not finite(stamp) or not 0<=now-stamp<=3 or stamp<self.minimum_stamp
                or (self.frame and stamp<=self.frame['stamp'])):return False
        detections=[]
        for item in data.get('detections',[])[:16]:
            if not isinstance(item,dict) or item.get('label')!=target_class:continue
            confidence=item.get('confidence')
            if not finite(confidence) or not .60<=confidence<=1:continue
            box=item.get('box',[])
            if len(box)!=4 or not all(finite(v) for v in box) or not box[0]<box[2] or not box[1]<box[3]:continue
            detection={k:v for k,v in item.items() if k in (
                'label','confidence','box','image_width','image_height','range_m','ranging_mode','range_state',
                'search_eligible','center_error_px','centered','projection_valid','bearing_valid',
                'settled','motion_check','attitude','reason','reason_codes','quality_flags','frame_age',
                'projection_error','bearing_error','camera_min_range','camera_max_range',
                'x','y','sigma','origin','bearing','position_sigma','bearing_sigma')}
            detection['stamp']=stamp
            # A capture-time map ray and an in-range pixel distance can still
            # provide a display-only point when covariance validation blocks
            # the mission measurement. Never substitute the latest robot pose.
            origin=detection.get('origin',[])
            if (not all(finite(detection.get(k)) for k in ('x','y','sigma'))
                    and detection.get('search_eligible') is not False
                    and detection.get('bearing_valid') is True
                    and detection.get('ranging_mode') in ('calibrated','near_guess')
                    and len(origin)==2 and all(finite(v) for v in origin)
                    and finite(detection.get('bearing')) and finite(detection.get('range_m'))
                    and 0<detection['range_m']<=float(detection.get('camera_max_range',.95))):
                distance,angle=detection['range_m'],detection['bearing']
                detection.update(x=origin[0]+distance*math.cos(angle),y=origin[1]+distance*math.sin(angle),
                    sigma=max(.15,detection.get('position_sigma') or .15),
                    quality_flags=[*detection.get('quality_flags',[]),'unverified_position_preview'])
            detections.append(detection)
        # Prefer a mappable observation; an in-range identity stays visible
        # even before the mission locks it or eight-frame confirmation passes.
        def mappable(d):return all(finite(d.get(k)) for k in ('x','y','sigma')) and d['sigma']>0 and d.get('search_eligible') is not False
        choices=[d for d in detections if mappable(d)]
        if self.diagnostic and self.diagnostic.get('target_locked') and self.preview:
            reference=self.diagnostic.get('selected') or self.preview
            if all(finite(reference.get(k)) for k in ('x','y')):
                choices=[d for d in choices if math.hypot(d['x']-reference['x'],d['y']-reference['y'])<.4]
        selected=max(choices or detections,key=lambda d:d['confidence']) if detections else None
        self.frame=finite_json(dict(stamp=stamp,detections=detections,selected=selected,
            raw_count=len(detections),in_range_count=sum(d.get('search_eligible') is not False for d in detections)))
        if selected:
            self.last_detection=finite_json(selected)
        if selected and selected in choices and mappable(selected):
            sigma=max(.02,float(selected['sigma']))
            if not selected.get('projection_valid'):sigma=max(.10,sigma)
            self.preview=dict(stamp=stamp,x=float(selected['x']),y=float(selected['y']),sigma=sigma,
                covariance=[[sigma*sigma,0.],[0.,sigma*sigma]],label=selected['label'],
                confidence=selected['confidence'],samples=1,frames=1,frame_id='map',
                method='single-frame visual preview',source='visual_preview',display_only=True,
                stable=False,confirmed=False,quality_flags=selected.get('quality_flags',[]),
                ranging_mode=selected.get('ranging_mode'),projection_valid=selected.get('projection_valid',False),
                settled=selected.get('settled',False),estimate_id='preview:'+format(stamp,'.9f'))
        return True

    def ingest_diagnostic(self,data,now):
        stamp=data.get('stamp')
        if not finite(stamp) or not 0<=now-stamp<=3 or stamp<self.minimum_stamp:return
        self.diagnostic=finite_json(dict(stamp=stamp,mmse=data.get('mmse') or {},
            reason_codes=data.get('reason_codes',[]),association=data.get('association') or {},
            target_locked=data.get('target_locked',False),selected=data.get('selected')))

    def displayed_preview(self,now):
        if not self.preview:return None
        age=now-self.preview['stamp']
        if age<0:return None
        current=bool(self.frame and self.frame.get('selected') and self.frame['stamp']==self.preview['stamp'])
        return dict(self.preview,age=age,stale=age>2 or not current,currently_visible=current and age<=2)

    def snapshot(self,now):
        frame=self.frame or {}
        age=now-frame['stamp'] if frame else None
        selected=frame.get('selected') if age is not None and 0<=age<=3 else None
        diagnostic=self.diagnostic if self.diagnostic and 0<=now-self.diagnostic['stamp']<=3 else None
        return dict(stamp=frame.get('stamp'),age=age,stale=age is None or age>3,
            detections=frame.get('detections',[]) if selected else [],selected=selected,
            raw_count=frame.get('raw_count',0) if age is not None and 0<=age<=3 else None,
            in_range_count=frame.get('in_range_count',0) if age is not None and 0<=age<=3 else None,
            last_detection=self.last_detection,
            last_seen_age=now-self.last_detection['stamp'] if self.last_detection else None,
            diagnostic=diagnostic)
