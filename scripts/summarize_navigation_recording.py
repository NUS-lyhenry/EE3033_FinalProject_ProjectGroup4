#!/usr/bin/env python3
"""Summarize recorded evidence; never sends navigation or motor commands."""
import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path


def evaluation_comparison(message):
    candidates=message.get('twists',[]);index=message.get('best_index',-1)
    if not 0<=index<len(candidates):return None
    best=candidates[index]
    forwards=[c for c in candidates if c['traj']['velocity']['x']>.01 and c['total']>=0]
    forward=min(forwards,key=lambda c:c['total']) if forwards else None
    def concise(c):
        return dict(velocity=c['traj']['velocity'],total=c['total'],scores=c['scores']) if c else None
    result=dict(best=concise(best),best_valid_forward=concise(forward),
        invalid_critics=dict(Counter(c['scores'][-1]['name'] for c in candidates if c['total']<0 and c.get('scores'))))
    if forward:
        b={s['name']:s['raw_score']*s['scale'] for s in best['scores']}
        f={s['name']:s['raw_score']*s['scale'] for s in forward['scores']}
        result['forward_minus_best_total']=forward['total']-best['total']
        result['forward_minus_best_critic_contributions']={name:f.get(name,0)-b.get(name,0) for name in b.keys()|f.keys()}
    return result


def summarize(path):
    path=Path(path);topics=Counter();gate=Counter();stages=Counter();scoring=0;all_forward_invalid=0;eval_count=0;examples=[];last_example=-1e99
    last_sim=None;max_moving_step=0.;stale_intercepts=Counter();failures=Counter()
    with gzip.open(path,'rt') as stream:
        for line in stream:
            row=json.loads(line);topic=row['topic'];m=row['message'];topics[topic]+=1
            if topic=='/project/velocity_gate_status':
                gate[m.get('reason')]+=1
                if m.get('reason') in ('scan_stale','command_stale') and any(abs(v or 0)>.001 for v in m.get('requested',[])):
                    stale_intercepts[m['reason']]+=1
            if topic=='/project/navigation_status':
                stages[m.get('stage')]+=1
                if m.get('stage')=='FAILED':failures[m.get('failure_reason') or m.get('reason')]+=1
            if topic=='/project/simulation_motion':
                last_sim=m
                if any(abs(v)>.001 for v in m.get('command',[])):
                    max_moving_step=max(max_moving_step,m.get('elapsed_step',0.))
            if topic!='/evaluation':continue
            eval_count+=1;c=evaluation_comparison(m)
            if not c:continue
            b=c['best'];f=c['best_valid_forward']
            if abs(b['velocity']['x'])<1e-5:
                if f and f['total']>b['total']:
                    scoring+=1
                    if row['monotonic']-last_example>=1. and len(examples)<12:
                        examples.append(c);last_example=row['monotonic']
                elif not f:all_forward_invalid+=1
    return dict(source=str(path),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        topic_counts=dict(topics),gate_reason_counts=dict(gate),supervisor_stage_counts=dict(stages),
        dwb_evaluation_frames=eval_count,rotation_preferred_over_valid_forward_frames=scoring,
        rotation_with_no_valid_forward_frames=all_forward_invalid,scoring_examples=examples,
        final_simulation_motion=last_sim,
        resource_evidence=dict(max_wall_step_while_motion_requested=max_moving_step,
            stale_interceptions_while_motion_requested=dict(stale_intercepts),navigation_failure_reasons=dict(failures)),
        interpretation='Rotation-scoring evidence is distinct from collision rejection and gate interception; counts may coexist in one run')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('recording');p.add_argument('--output');a=p.parse_args()
    result=summarize(a.recording);text=json.dumps(result,indent=2)+'\n'
    if a.output:Path(a.output).write_text(text)
    else:print(text)


if __name__=='__main__':main()
