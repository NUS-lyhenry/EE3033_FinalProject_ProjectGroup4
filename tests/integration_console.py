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
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ee3033_project.console import Console
from ee3033_project.ros_utils import LATCHED


class ConsoleTests(unittest.TestCase):
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
