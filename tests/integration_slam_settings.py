"""Real SLAM readback, pending persistence and process-restart regression. No motion."""
import argparse,json,os,tempfile,time,threading,subprocess,signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.srv import GetParameters
from ee3033_project.console_slam_settings import SlamSettings
assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6','121')
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
parser=argparse.ArgumentParser()
parser.add_argument('config',type=Path)
parser.add_argument('--output',type=Path)
args=parser.parse_args()
class Fixture(SlamSettings,Node):
 def __init__(self,directory):
  Node.__init__(self,'slam_settings_native_fixture')
  self.p=dict(phase='mapping',session_directory=directory);self.stage=None
  self.control_generation=0;self.lock=threading.Lock();self.state={'state':'WAITING'}
  self.scan_filter_editable=lambda: self.state['state']=='WAITING'
  self.parameter_clients={('slam_toolbox','get_parameters'):self.create_client(GetParameters,'/slam_toolbox/get_parameters')}
  self.init_slam_settings()
 def parameter_request(self,name,operation,request):
  f=self.parameter_clients[(name,operation)].call_async(request);deadline=time.monotonic()+5
  while not f.done() and time.monotonic()<deadline:time.sleep(.01)
  if not f.done():raise RuntimeError('Native parameter query timed out')
  return f.result()
def start(threshold,passes,log):
 return subprocess.Popen(['/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node','--ros-args','--params-file',str(args.config.resolve()),'-p','resolution:=0.05','-p',f'occupancy_threshold:={threshold}','-p',f'min_pass_through:={passes}'],stdout=log,stderr=subprocess.STDOUT)
def stop(process):
 if process is not None and process.poll() is None:
  process.send_signal(signal.SIGINT)
  try:process.wait(timeout=5)
  except subprocess.TimeoutExpired:process.kill();process.wait(timeout=3)
def read_until(node,expected):
 deadline=time.monotonic()+10;last=None
 while time.monotonic()<deadline:
  last=node.slam_settings()
  if last.get('live')==expected:return last
  time.sleep(.2)
 raise AssertionError(last)
rclpy.init();process=None
with tempfile.TemporaryDirectory(prefix='slam-settings-native-') as directory,tempfile.TemporaryFile(mode='w+') as log:
 node=Fixture(directory);executor=SingleThreadedExecutor();executor.add_node(node)
 thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
 try:
  old=dict(occupancy_threshold=.1,minimum_pass_through=2);new=dict(occupancy_threshold=.2,minimum_pass_through=4)
  process=start(.1,2,log);before=read_until(node,old)
  pending=node.slam_settings(new,0);assert pending['selected']==new and pending['live']==old and pending['requires_new_map']
  unchanged=read_until(node,old);assert unchanged['selected']==new and unchanged['requires_new_map']
  stop(process);process=None;node.control_generation+=1
  assert not node.slam_settings_snapshot()['available']
  process=start(.2,4,log);after=read_until(node,new)
  assert after['synchronized'] and not after['requires_new_map']
  result=dict(passed=True,simulated=True,physical_motion=False,backend='Real native slam_toolbox parameter services',before=before,pending=pending,unchanged_active_readback=unchanged,after_new_process=after,persisted=json.loads(node.slam_settings_path.read_text()))
  if args.output:args.output.write_text(json.dumps(result,indent=2)+'\n')
  print(json.dumps(result))
 finally:
  stop(process);executor.shutdown(timeout_sec=2);node.destroy_node();rclpy.shutdown();thread.join(timeout=2)
