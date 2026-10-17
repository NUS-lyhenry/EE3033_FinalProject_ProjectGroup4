"""HTTP-to-ROS console checks on a separate ROS domain, without hardware."""
import json
import os
import threading
import time
import unittest
from urllib.request import Request,urlopen
from urllib.error import HTTPError
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from ee3033_project.console import Console
from ee3033_project.ros_utils import LATCHED
from ee3033_project.mapper import Mapper
from ee3033_project.velocity_gate import VelocityGate
from ee3033_project.trial import parameter_plan


class ConsoleTests(unittest.TestCase):
    def test_live_trial_sliders_and_voltage(self):
        rclpy.init()
        console=Console(parameter_overrides=[Parameter('port',value=0),
                        Parameter('phase',value='mapping'),Parameter('allow_motion',value=True)])
        gate=VelocityGate(parameter_overrides=[Parameter('mapping_mode',value=True),
             Parameter('max_linear',value=.12),Parameter('max_angular',value=.4)])
        mapper=Mapper()
        controller=Node('controller_server')
        initial=dict(max_linear=.12,max_angular=.4,goal_timeout=60.)
        for key,value in parameter_plan(initial)['controller_server'].items():
            controller.declare_parameter(key,value)
        reject=[False]
        def accept(changes):
            if reject[0]:
                reject[0]=False
                return SetParametersResult(successful=False,reason='Simulated rejection')
            return SetParametersResult(successful=True)
        controller.add_on_set_parameters_callback(accept)
        voltage=controller.create_publisher(Float32,'/power_voltage',10)
        executor=MultiThreadedExecutor(num_threads=4)
        for node in (console,gate,mapper,controller):
            executor.add_node(node)
        thread=threading.Thread(target=executor.spin,daemon=True)
        thread.start()
        base=f'http://127.0.0.1:{console.p["port"]}'
        token=console.token
        def post(data):
            request=Request(base+'/api/trial',json.dumps(data).encode(),
                            {'Content-Type':'application/json','X-Project-Token':token},method='POST')
            return json.load(urlopen(request,timeout=15))
        try:
            deadline=time.monotonic()+4
            while 'Mission' not in console.freshness and time.monotonic()<deadline:
                time.sleep(.05)
            state=json.load(urlopen(base+'/api/trial',timeout=12))
            self.assertEqual(state['values'],initial)
            changed=dict(max_linear=.18,max_angular=.6,goal_timeout=90.)
            response=post(changed)
            self.assertTrue(response['synchronized'])
            self.assertEqual(response['values'],changed)
            time.sleep(.2)
            self.assertEqual(gate.p['max_linear'],.18)
            self.assertEqual(mapper.p['goal_timeout'],90.)
            self.assertEqual(gate.p['rotation_clearance'],.16)
            with self.assertRaises(HTTPError):
                post({**changed,'max_linear':1.})
            # A rejection restores all nodes already updated, not just UI values.
            reject[0]=True
            with self.assertRaises(HTTPError):
                post(initial)
            self.assertEqual(json.load(urlopen(base+'/api/trial',timeout=12))['values'],changed)
            self.assertIsNone(console.trial_fault)
            for timer in mapper.timers:
                timer.cancel()
            mapper.state='MAPPING'
            console.on_status(String(data=json.dumps({'state':'MAPPING','reason':'Synthetic active mapping'})))
            with self.assertRaises(HTTPError):
                post(initial)
            self.assertFalse(mapper.set_parameters_atomically([Parameter('goal_timeout',value=60.)]).successful)
            mapper.state='STOPPED'
            for _ in range(4):
                voltage.publish(Float32(data=11.87))
                time.sleep(.1)
            battery=json.load(urlopen(base+'/api/state'))['battery']
            self.assertTrue(battery['fresh'])
            self.assertAlmostEqual(battery['voltage'],11.87,places=2)
            with console.lock:
                console.freshness['Battery']=time.monotonic()-6
            self.assertFalse(json.load(urlopen(base+'/api/state'))['battery']['fresh'])
            print('PASS: live navigation trial parameters, limits, readback, rollback, active lockout, battery freshness')
        finally:
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            for node in (console,gate,mapper,controller):
                node.destroy_node()
            rclpy.shutdown()

    def test_http_controls_and_autonomous_lockout(self):
        rclpy.init()
        console=Console(parameter_overrides=[Parameter('port',value=0),
                                             Parameter('allow_motion',value=True)])
        io=Node('console_test_peer')
        status=io.create_publisher(String,'/project/mission_status',LATCHED)
        calls=[]
        manual=[]
        io.create_subscription(Twist,'/project/manual_cmd_vel',lambda m:manual.append(m),10)
        def trigger(request,response):
            del request
            calls.append('called')
            response.success,response.message=True,'Mock mission accepted'
            return response
        io.create_service(Trigger,'/project/start',trigger)
        io.create_service(Trigger,'/project/stop',trigger)
        executor=MultiThreadedExecutor(num_threads=3)
        executor.add_node(io)
        executor.add_node(console)
        thread=threading.Thread(target=executor.spin,daemon=True)
        thread.start()
        base=f'http://127.0.0.1:{console.p["port"]}'
        try:
            html=urlopen(base).read().decode()
            self.assertIn('Mission console',html)
            self.assertIn('Set initial pose',html)
            self.assertIn('Live object detections',html)
            state=json.load(urlopen(base+'/api/state'))
            token=state['token']
            def post(endpoint,data=None,token_value=token,origin=None):
                headers={'Content-Type':'application/json','X-Project-Token':token_value}
                if origin:
                    headers['Origin']=origin
                request=Request(base+'/api/'+endpoint,json.dumps(data or {}).encode(),headers,method='POST')
                return json.load(urlopen(request,timeout=7))
            with self.assertRaises(HTTPError) as missing:
                post('start',token_value='')
            self.assertEqual(missing.exception.code,403)
            with self.assertRaises(HTTPError) as cross:
                post('start',origin='https://example.invalid')
            self.assertEqual(cross.exception.code,403)
            self.assertTrue(post('start')['success'])
            self.assertTrue(post('stop')['success'])
            self.assertEqual(len(calls),2)
            with self.assertRaises(HTTPError) as manual_error:
                post('teleop',{'linear':0.1})
            self.assertEqual(manual_error.exception.code,400)
            status.publish(String(data=json.dumps({'state':'SEARCHING','reason':'Active',
                                                   'target':None,'home':[0,0,0]})))
            deadline=time.monotonic()+3
            while console.state['state']!='SEARCHING' and time.monotonic()<deadline:
                time.sleep(0.02)
            with self.assertRaises(HTTPError):
                post('initial_pose',{'x':0,'y':0,'yaw':0})
            with self.assertRaises(HTTPError):
                post('save_map')
            # Backend enforces the motion-disabled state even if a caller bypasses UI.
            console.p['allow_motion']=False
            with self.assertRaises(HTTPError):
                post('start')
            print('PASS: console HTML, start/stop ROS services, token/origin checks, '
                  'autonomous manual-control lockout, and disabled-motion enforcement')
        finally:
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            console.destroy_node()
            io.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':
    if os.environ.get('ROS_DOMAIN_ID') in (None,'0','6'):
        raise SystemExit('Use an isolated ROS_DOMAIN_ID, e.g. 90')
    unittest.main(verbosity=2)
