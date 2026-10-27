#!/usr/bin/env python3
"""Drive only the explicitly simulated console through its real HTTP workflow.

The world, detector, SLAM, MMSE, Mission and Nav2 supply every result. No target
coordinates, navigation outcomes or mission transitions are injected.
"""
import argparse
import json
from pathlib import Path
import time
import urllib.error
import urllib.parse
import urllib.request


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--url',default='http://127.0.0.1:8792')
    parser.add_argument('--reuse-map',action='store_true',help='Unfinished trial only; simulated world must have been reset to its assigned origin')
    parser.add_argument('--output',default='/tmp/ee3033-preview-validation.json')
    args=parser.parse_args()
    parsed=urllib.parse.urlparse(args.url)
    if parsed.scheme!='http' or parsed.hostname not in ('localhost','127.0.0.1'):
        parser.error('Only a localhost simulation URL is permitted')
    base=args.url.rstrip('/')+'/api/'
    def get(path='state'):
        with urllib.request.urlopen(base+path,timeout=30) as response:return json.load(response)
    def simulated(state):
        if state.get('simulated') is not True or state.get('preview_backend')!='main-ros':
            raise RuntimeError('Refusing to control a non-simulation console')
    def post(path,body=None):
        state=get();simulated(state)
        request=urllib.request.Request(base+path,data=json.dumps(body or {}).encode(),headers={
            'Content-Type':'application/json','X-Project-Token':state['token'],
            'X-Control-Generation':str(state['control_generation'])})
        try:
            with urllib.request.urlopen(request,timeout=60) as response:result=json.load(response)
        except urllib.error.HTTPError as error:raise RuntimeError(error.read().decode()) from error
        if result.get('success') is False:raise RuntimeError(str(result))
        print('POST',path,result.get('message','OK'),flush=True)
        return result
    def wait(predicate,timeout):
        deadline=time.monotonic()+timeout;last=None
        while time.monotonic()<deadline:
            state=get();simulated(state);mission=state.get('mission',{})
            key=state['phase'],mission.get('state'),mission.get('reason')
            if key!=last:print('STATE',*key,flush=True);last=key
            if predicate(state):return state
            if mission.get('state')=='FAILED':raise RuntimeError(mission.get('reason'))
            time.sleep(.25)
        raise TimeoutError('Simulation workflow timed out')
    initial=get();simulated(initial)
    try:
        if args.reuse_map:
            if initial.get('assessment',{}).get('finished') is not None:
                raise RuntimeError('This trial is finished; run without --reuse-map to build a fresh trial map')
            entry=get('maps')['maps'][0];pose=[0.,0.,0.]
        else:
            if initial.get('assessment',{}).get('finished') is not None:
                post('new_trial')
            else:
                post('stage',{'phase':'mapping'})
            wait(lambda s:s.get('mission',{}).get('ready') is True,60)
            post('start')
            state=wait(lambda s:s.get('mission',{}).get('state') in ('MAPPED','MAPPED_PARTIAL'),300)
            pose=state['pose']
            if not pose:raise RuntimeError('Mapping-to-mission pose handoff is unavailable')
            entry=next(m for m in get('maps')['maps'] if m['path']==state['saved_map'])
            post('review_map',{'map_id':entry['id']})
        labels=get('labels')
        if not labels.get('ready') or 'cup' not in labels['labels']:raise RuntimeError(str(labels))
        post('stage',dict(phase='mission',map_id=entry['id'],target_class='cup',localization_method='hybrid',
                          return_tolerance=.2,preferred_target_sigma=.1))
        wait(lambda s:s['phase']=='mission' and s.get('mission',{}).get('state')=='WAITING'
             and s.get('health',{}).get('Mission',{}).get('ok'),60)
        post('initial_pose',dict(zip(('x','y','yaw'),pose)))
        wait(lambda s:s.get('mission',{}).get('ready') is True,60)
        post('start')
        state=wait(lambda s:s.get('mission',{}).get('state') in
                   ('COMPLETE','FOUND_UNLOCALIZED','SEARCH_INCOMPLETE','NOT_FOUND','STOPPED'),600)
        result=get('result');Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
        mission=state['mission']
        print(json.dumps({k:mission.get(k) for k in ('state','elapsed_seconds','target','return_verified','return_distance')},indent=2),flush=True)
        if mission['state']!='COMPLETE' or mission.get('return_verified') is not True:
            raise RuntimeError('Simulation did not complete target report and verified return')
    finally:
        post('stop')


if __name__=='__main__':main()
