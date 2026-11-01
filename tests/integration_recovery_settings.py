"""Native HTTP/ROS recovery switch transactions on an isolated domain; no hardware."""
import json,tempfile,threading,unittest
from pathlib import Path
from unittest.mock import patch
from urllib.request import Request,urlopen
from urllib.error import HTTPError
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from ee3033_project.console import Console

class RecoverySettingsTests(unittest.TestCase):
    def setUp(self):
        rclpy.init();self.root=tempfile.TemporaryDirectory()
        self.supervisor=Node('project_navigation_supervisor')
        self.supervisor.declare_parameter('allow_reverse_recovery',True)
        self.console=Console(parameter_overrides=[Parameter('port',value=0),Parameter('phase',value='mapping'),Parameter('manage_stages',value=True),Parameter('session_directory',value=self.root.name)])
        self.executor=MultiThreadedExecutor(num_threads=4)
        for n in (self.console,self.supervisor):self.executor.add_node(n)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        self.base=f'http://127.0.0.1:{self.console.p["port"]}'
    def tearDown(self):
        self.executor.shutdown(timeout_sec=3);self.thread.join(timeout=3)
        self.console.destroy_node();self.supervisor.destroy_node();rclpy.shutdown();self.root.cleanup()
    def call(self,body=None):
        request=self.base+'/api/recovery'
        if body is not None:request=Request(request,json.dumps(body).encode(),{'Content-Type':'application/json','Origin':self.base,'X-Project-Token':self.console.token,'X-Control-Generation':str(self.console.control_generation)},method='POST')
        return json.load(urlopen(request,timeout=6))
    def test_saved_and_live_boolean_round_trip(self):
        self.assertTrue(self.call()['live_value'])
        off=self.call({'enabled':False})
        self.assertFalse(off['value']);self.assertFalse(off['live_value']);self.assertTrue(off['synchronized'])
        self.assertFalse(self.supervisor.get_parameter('allow_reverse_recovery').value)
        self.assertFalse(json.loads(Path(self.root.name,'recovery.json').read_text())['enabled'])
        self.assertFalse(self.console.stage.allow_reverse_recovery)
        self.assertTrue(self.call({'enabled':True})['live_value'])
    def test_active_or_non_boolean_requests_cannot_change_setting(self):
        for body in ({'enabled':1},{'enabled':'false'},{'enabled':None},{}):
            with self.assertRaises(HTTPError):self.call(body)
        self.console.state['state']='MAPPING'
        with self.assertRaises(HTTPError):self.call({'enabled':False})
        self.assertTrue(self.supervisor.get_parameter('allow_reverse_recovery').value)
    def test_failed_persistence_rolls_live_parameter_back(self):
        with patch('ee3033_project.console_recovery.atomic_json',side_effect=OSError('fixture disk error')):
            with self.assertRaises(HTTPError):self.call({'enabled':False})
        self.assertTrue(self.supervisor.get_parameter('allow_reverse_recovery').value)
        self.assertTrue(self.console.recovery_value)
    def test_saved_off_survives_settings_reload(self):
        self.call({'enabled':False});self.console.recovery_value=True;self.console.stage.allow_reverse_recovery=True
        self.console.init_recovery()
        self.assertFalse(self.console.recovery_value);self.assertFalse(self.console.stage.allow_reverse_recovery)

if __name__=='__main__':unittest.main()
