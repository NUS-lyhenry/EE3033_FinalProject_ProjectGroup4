"""HTTP → owner cancellation → home action, with synthetic ROS sensors only."""
import json,os,tempfile,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace
from urllib.request import Request,urlopen
from urllib.error import HTTPError
import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from integration_ros import FakeRobot,overrides,wait
from ee3033_project.mapper import Mapper
from ee3033_project.mission import Mission
from ee3033_project.console import Console

class ReturnRobot(FakeRobot):
    def __init__(self):
        self.release_cancel=threading.Event();self.release_cancel.set();self.late_accept=False
        super().__init__('absent')
        self.command=self.create_publisher(Twist,'/cmd_vel',10)
        self.create_timer(.03,lambda:self.command.publish(Twist()))
    def accept(self,request):
        if self.late_accept:time.sleep(.35)
        return super().accept(request)
    async def execute(self,handle):
        p=handle.request.pose.pose.position;home=abs(p.x)<.01 and abs(p.y)<.01
        self.active_goals+=1;self.max_active_goals=max(self.max_active_goals,self.active_goals)
        if home:self.home_goals+=1
        else:self.search_goals+=1
        try:
            if home:
                self.xy_yaw=[p.x,p.y,0.];time.sleep(.15);handle.succeed()
            else:
                deadline=time.monotonic()+10
                while not handle.is_cancel_requested and time.monotonic()<deadline:time.sleep(.01)
                if handle.is_cancel_requested:
                    self.release_cancel.wait(timeout=4);self.canceled+=1;handle.canceled()
                else:handle.abort()
            return NavigateToPose.Result()
        finally:self.active_goals-=1

class ReturnHomeIntegration(unittest.TestCase):
    def run_case(self,phase,mode):
        rclpy.init();root=tempfile.TemporaryDirectory();robot=ReturnRobot()
        owner=(Mapper(parameter_overrides=overrides(allow_motion=True,map_directory=root.name)) if phase=='mapping'
            else Mission(parameter_overrides=overrides(allow_motion=True,target_class='bottle',results_directory=root.name,verify_seconds=.15,verify_timeout=2.)))
        console=Console(parameter_overrides=overrides(port=0,phase=phase,allow_motion=True,manage_stages=True,session_directory=str(Path(root.name)/'console')))
        # The fixture owns the nodes instead of launching hardware processes.
        console.stage.process=SimpleNamespace(poll=lambda:None)
        executor=MultiThreadedExecutor(num_threads=8)
        for n in (robot,owner,console):executor.add_node(n)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        base=f'http://127.0.0.1:{console.p["port"]}'
        def call(name):
            request=Request(base+'/api/'+name,b'{}',{'Content-Type':'application/json','Origin':base,'X-Project-Token':console.token,'X-Control-Generation':str(console.control_generation)},method='POST')
            return json.load(urlopen(request,timeout=7))
        try:
            self.assertTrue(wait(lambda:owner.nav.server_is_ready() and owner.navigation_map.fresh() and (owner.error() if phase=='mapping' else owner.readiness_error(False)) is None))
            if mode=='no_home':
                with self.assertRaises(HTTPError):call('return_home')
                self.assertEqual(robot.home_goals,0);return
            if phase=='mission':
                answer=owner.set_parameters_atomically(overrides(return_home_seed='[0.0,0.0,0.0]'))
                self.assertTrue(answer.successful,answer.reason)
                self.assertTrue(wait(lambda:owner.home is not None))
            else:owner.home=(0.,0.,0.)
            if mode=='idle':
                owner.state='STOPPED' if phase=='mapping' else 'WAITING';robot.xy_yaw=[.4,0.,0.]
            else:
                if mode in ('stop','research_return','research_stop'):robot.release_cancel.clear()
                if mode=='late':robot.late_accept=True
                owner.started=time.monotonic()
                if phase=='mapping':owner.state='MAPPING';owner.send((.4,0.,0.))
                else:owner.send_goal(owner.make_pose(.4,0.,0.),'search')
                self.assertTrue(wait(lambda:owner.pending if phase=='mapping' and mode=='late' else owner.goal_pending if phase=='mission' and mode=='late' else robot.active_goals==1))
            if mode in ('research_return','research_stop'):
                owner.target_detected=True;owner.target_ever_detected=True
                owner.acquisition.started=time.monotonic()
                owner.request_research('Synthetic target loss')
                self.assertTrue(wait(lambda:owner.state=='CANCELING' and owner.cancel_next=='RESEARCH'))
            self.assertTrue(wait(lambda:console.state.get('home') is not None))
            if mode in ('research_return','research_stop'):
                self.assertTrue(wait(lambda:console.state.get('pending_action')=='RESEARCH'))
            if phase=='mission' and mode=='idle':
                request=Request(base+'/api/initial_pose',json.dumps(dict(x=0.,y=0.,yaw=0.)).encode(),{'Content-Type':'application/json','Origin':base,'X-Project-Token':console.token,'X-Control-Generation':str(console.control_generation)},method='POST')
                try:self.assertTrue(json.load(urlopen(request,timeout=6))['success'])
                except HTTPError as exc:self.fail(exc.read().decode())
            answer=call('return_home');self.assertTrue(answer['success'],answer)
            if mode in ('stop','research_stop'):
                self.assertTrue(wait(lambda:owner.state=='CANCELING'))
                self.assertTrue(call('stop')['success']);robot.release_cancel.set()
                self.assertTrue(wait(lambda:owner.state=='STOPPED' and robot.active_goals==0))
                self.assertEqual(robot.home_goals,0)
            else:
                robot.release_cancel.set()
                self.assertTrue(wait(lambda:robot.home_goals==1),owner.reason)
                self.assertEqual(robot.max_active_goals,1)
                self.assertTrue(wait(lambda:owner.return_verified,8),owner.reason)
                self.assertEqual(tuple(owner.home),(0.,0.,0.))
                if phase=='mission':
                    expected='FOUND_UNLOCALIZED' if mode=='research_return' else 'SEARCH_INCOMPLETE'
                    self.assertEqual(owner.state,expected,owner.reason)
            if mode in ('research_return','research_stop'):
                self.assertEqual(owner.search_restarts,0)
            print('PASS',phase,mode,'home goals',robot.home_goals,'max overlapping',robot.max_active_goals)
        finally:
            robot.release_cancel.set();executor.shutdown(timeout_sec=4);thread.join(timeout=4)
            console.stage.process=None
            console.destroy_node();owner.destroy_node();robot.destroy_node();rclpy.shutdown();root.cleanup()
    def test_mapping_active_return(self):self.run_case('mapping','active')
    def test_mapping_stop_supersedes_return(self):self.run_case('mapping','stop')
    def test_mapping_late_acceptance_is_canceled_before_home(self):self.run_case('mapping','late')
    def test_mapping_failed_task_can_return(self):self.run_case('mapping','idle')
    def test_mission_active_return(self):self.run_case('mission','active')
    def test_mission_stop_supersedes_return(self):self.run_case('mission','stop')
    def test_mission_retained_home_can_return_without_search(self):self.run_case('mission','idle')
    def test_missing_home_refuses_motion(self):self.run_case('mission','no_home')
    def test_return_supersedes_pending_research_cancellation(self):self.run_case('mission','research_return')
    def test_stop_supersedes_return_queued_during_research_cancellation(self):self.run_case('mission','research_stop')

if __name__=='__main__':
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6')
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    unittest.main()
