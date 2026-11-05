"""Installed Humble dispatches message-only; readiness needs real fresh data."""
from pathlib import Path
import inspect
import runpy
helpers=runpy.run_path(str(Path(__file__).parents[1]/'scripts/passive_simulation_timing.py'))


def test_all_callback_adapters_accept_exactly_one_message_and_bind_streams():
    calls=[]
    callback=helpers['message_callback'](lambda message,info,static:calls.append((message,info,static)),True)
    assert len(inspect.signature(callback).parameters)==1
    callback('tf')
    a=helpers['string_callback']('target_detections',lambda *args:calls.append(args))
    b=helpers['string_callback']('mission_status',lambda *args:calls.append(args))
    a('frame');b('status')
    assert calls==[('tf',None,True),('target_detections','frame',None),('mission_status','status',None)]


def test_empty_subscriptions_are_not_ready_and_fresh_packets_are_required():
    ready=helpers['stream_readiness']
    assert ready({},dict(),100.,10.)['ready'] is False
    stats={}
    for key in ('odom','tf:odom_combined->base_footprint','scan_fixed','mission_status','target_detections','cmd_vel'):
        item=helpers['StreamStats']();item.add(100.,None if key=='cmd_vel' else 99.9,9.9);stats[key]=item
    lookups=dict(latest_available=True,latest_age_seconds=.02,latest_receipt_monotonic=9.9)
    assert ready(stats,lookups,100.,10.)['ready'] is True
    assert ready(stats,lookups,102.,12.)['ready'] is False


def test_split_scan_readiness_rejects_missing_or_stale_navigation_feed_even_when_mapping_is_fresh():
    ready=helpers['stream_readiness'];stats={}
    for key in ('odom','tf:odom_combined->base_footprint','scan_fixed','mission_status','target_detections','cmd_vel'):
        item=helpers['StreamStats']();item.add(100.,None if key=='cmd_vel' else 99.9,9.9);stats[key]=item
    lookups=dict(latest_available=True,latest_age_seconds=.02,latest_receipt_monotonic=9.9)
    assert ready(stats,lookups,100.,10.,True)['ready'] is False
    nav=helpers['StreamStats']();nav.add(100.,98.,9.9);stats['scan_navigation']=nav
    assert ready(stats,lookups,100.,10.,True)['ready'] is False
    nav.add(100.,99.9,9.9)
    assert ready(stats,lookups,100.,10.,True)['ready'] is True
    stats['scan_fixed'].last_stamp=98.
    assert ready(stats,lookups,100.,10.,True)['ready'] is False
