"""HTTP -> stage process -> ROS -> map save -> mission -> recorded result.

Real mapper, mission, filter, gate, ROS graph and Nav2 map saver. Sensors and
NavigateToPose server are synthetic. No hardware drivers are launched.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request,urlopen

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from ee3033_project.console import Console
from ee3033_project.assessment import StageProcess


def wait(predicate,timeout=20):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():return True
        time.sleep(.05)
    return False


class WorkflowIntegration(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.root=tempfile.TemporaryDirectory(prefix='group4_workflow_')
        self.map_root=Path(self.root.name)/'maps'
        self.console=Console(parameter_overrides=[Parameter(k,value=v) for k,v in dict(
            port=0,phase='mapping',allow_motion=True,manage_stages=True,
            map_directory=str(self.map_root),session_directory=self.root.name+'/records').items()])
        # The synthetic detector fixture has these labels; real deployments
        # obtain them from the configured weights through /api/labels.
        from ee3033_project.model_labels import ModelLabels
        weights=Path(self.root.name)/'fixture.pt';weights.write_bytes(b'synthetic detector')
        self.console.model_labels=ModelLabels(weights)
        self.console.model_labels.signature=self.console.model_labels.identity()
        self.console.model_labels.cache=self.console.model_labels.response(labels=['bottle','cup'])
        self.executor=MultiThreadedExecutor(num_threads=4);self.executor.add_node(self.console)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        self.base=f'http://127.0.0.1:{self.console.p["port"]}'
        self.launched=[]
        def fixture(args,**kwargs):
            self.launched.append(args)
            phase='mapping' if 'mapping.launch.py' in args else 'mission'
            return subprocess.Popen([sys.executable,str(Path(__file__).parent/'fixtures/workflow_stage.py'),
                                     phase,str(self.map_root)],**kwargs)
        self.console.stage.popen=fixture

    def tearDown(self):
        try:self.console.stage.stop()
        finally:
            self.executor.shutdown(timeout_sec=3);self.thread.join(timeout=3)
            self.console.destroy_node();rclpy.shutdown();self.root.cleanup()

    def get(self,path='state'):
        return json.load(urlopen(self.base+'/api/'+path,timeout=10))

    def post(self,path,data=None,generation=None):
        headers={'Content-Type':'application/json','X-Project-Token':self.console.token,
                 'X-Control-Generation':str(self.get()['control_generation'] if generation is None else generation)}
        return json.load(urlopen(Request(self.base+'/api/'+path,json.dumps(data or {}).encode(),
                                        headers,method='POST'),timeout=25))

    def test_basic_two_stage_trial_and_evidence(self):
        self.post('stage',{'phase':'mapping'})
        self.assertTrue(wait(lambda:self.get()['mission'].get('ready')),self.get())
        self.assertTrue(wait(lambda:self.get()['health'].get('Odometry',{}).get('ok')))
        first_generation=self.get()['control_generation']
        self.post('start')
        self.assertTrue(wait(lambda:self.get()['mission']['state'] in ('MAPPED','MAPPED_PARTIAL','FAILED')))
        self.assertEqual(self.get()['mission']['state'],'MAPPED',self.get())
        maps=self.get('maps')['maps'];self.assertEqual(len(maps),1)
        self.assertEqual(maps[0]['quality'],'mapped');self.assertFalse(maps[0]['reviewed'])
        settings=dict(phase='mission',map_id=maps[0]['id'],target_class='bottle',localization_method='lidar',return_tolerance=.2)
        with self.assertRaises(HTTPError):self.post('stage',settings)
        self.post('review_map',{'map_id':maps[0]['id']})
        with self.assertRaises(HTTPError):self.post('stage',{**settings,'target_class':'pepsi'})
        self.post('stage',settings)
        self.assertTrue(wait(lambda:self.get()['mission'].get('ready')))
        self.assertEqual(self.console.count_publishers('/cmd_vel'),1)
        self.assertIn('map:='+maps[0]['path'],self.launched[-1])
        with self.assertRaises(HTTPError):self.post('teleop',{'linear':.1})
        with self.assertRaises(HTTPError):self.post('start')
        with self.assertRaises(HTTPError):self.post('initial_pose',{'x':0.,'y':0.,'yaw':0.},generation=first_generation)
        self.post('initial_pose',{'x':0.,'y':0.,'yaw':0.})
        self.post('start')
        self.assertTrue(wait(lambda:self.get()['mission']['state']=='COMPLETE'),self.get())
        result=self.get('result');self.assertNotIn('token',result)
        self.assertTrue(result['mission']['return_verified'])
        self.assertAlmostEqual(result['mission']['target']['x'],.5)
        self.assertTrue(result['assessment']['timing_valid'])
        self.assertIsNotNone(result['assessment']['finished'])
        self.assertGreater(result['assessment']['elapsed_seconds'],result['mission']['elapsed_seconds'])
        persisted=Path(self.root.name)/'records'/result['assessment']['id']/'assessment.json'
        self.assertEqual(json.loads(persisted.read_text())['result']['state'],'COMPLETE')
        self.post('new_trial')
        self.assertTrue(wait(lambda:self.get()['mission'].get('phase')=='mapping' and self.get()['mission'].get('ready')))
        with self.assertRaises(HTTPError):self.post('stage',settings) # prior trial's map cannot replace on-site construction
        self.assertEqual(self.console.count_publishers('/cmd_vel'),1)
        print('PASS: native two-stage process transition, unique map/save/review, pose lockout, autonomous report/verified return, continuous timing and persisted export')

    def test_stop_during_loading_prevents_late_stage(self):
        entered=threading.Event();release=threading.Event();factory=self.console.stage.popen
        def delayed(args,**kwargs):
            entered.set();release.wait(timeout=10)
            return factory(args,**kwargs)
        self.console.stage.popen=delayed
        errors=[]
        def load():
            try:self.post('stage',{'phase':'mapping'})
            except Exception as exc:errors.append(exc)
        thread=threading.Thread(target=load);thread.start()
        try:
            self.assertTrue(entered.wait(timeout=3))
            self.assertTrue(self.post('stop')['success'])
        finally:release.set();thread.join(timeout=15)
        self.assertFalse(thread.is_alive());self.assertEqual(errors,[])
        self.assertIsNone(self.console.stage.process)
        self.assertTrue(wait(lambda:self.console.count_publishers('/cmd_vel')==0,5))
        print('PASS: Stop bypasses the stage transaction and cancels a late process start')


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
        raise SystemExit('Use an isolated ROS_DOMAIN_ID, e.g. 93. Never domain 6.')
    unittest.main(verbosity=2)
