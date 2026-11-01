"""Native ROS detections -> console HTTP preview, without a mission or motors."""
import json,os,tempfile,threading,time,urllib.request
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String
from ee3033_project.console import Console
from integration_ros import wait


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','6')
    rclpy.init()
    with tempfile.TemporaryDirectory(prefix='visual_telemetry_check_') as directory:
        console=Console(parameter_overrides=[Parameter('port',value=0),Parameter('phase',value='mission'),
            Parameter('session_directory',value=directory),Parameter('allow_motion',value=False)])
        console.mission_options['target_class']='bottle'
        fixture=Node('visual_preview_fixture')
        publisher=fixture.create_publisher(String,'/project/target_detections',10)
        diagnostics=fixture.create_publisher(String,'/project/localization_diagnostics',10)
        mode=['coordinate']
        def publish():
            stamp=fixture.get_clock().now().nanoseconds*1e-9
            data=dict(stamp=stamp,detections=[])
            if mode[0]!='missing':
                d=dict(stamp=stamp,label='bottle',confidence=.85,box=[300.,100.,340.,300.],
                    image_width=640,image_height=480,range_m=.7,settled=False,projection_valid=False,
                    center_error_px=0.,motion_check=dict(verified=False,linear_mean=.3,reason='Mean linear speed is too high'))
                if mode[0]=='coordinate':d.update(x=.7,y=.2,sigma=.25,search_eligible=True)
                else:d.update(origin=[0.,0.],bearing=.1,search_eligible=False,range_state='too_far')
                data['detections']=[d]
            publisher.publish(String(data=json.dumps(data)))
            diagnostics.publish(String(data=json.dumps(dict(stamp=stamp,mmse=dict(batch_frames=2,required_frames=8)))))
        fixture.create_timer(.1,publish)
        executor=MultiThreadedExecutor(num_threads=4)
        for node in (console,fixture):executor.add_node(node)
        thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
        def state():return json.load(urllib.request.urlopen('http://127.0.0.1:%d/api/state'%console.p['port'],timeout=3))
        try:
            assert wait(lambda:state().get('target_preview') is not None,timeout=10)
            s=state();assert s['target_preview']['display_only'] and not s['target_preview']['confirmed']
            assert s['mission']['target'] is None and s['target_estimate'] is None
            assert s['target_preview']['sigma']==.25
            assert wait(lambda:(state()['perception_details'].get('diagnostic') or {}).get('mmse',{}).get('batch_frames')==2,timeout=5)
            mode[0]='missing'
            assert wait(lambda:state()['perception_details']['raw_count']==0,timeout=5)
            assert state()['target_preview']['stale']
            mode[0]='far'
            with console.lock:console.visual.reset(time.time())
            assert wait(lambda:state()['perception_details']['in_range_count']==0 and state()['perception_details']['raw_count']==1,timeout=5)
            s=state();assert s['target_preview'] is None and s['perception_details']['selected']['range_state']=='too_far'
            assert s['mission']['target'] is None
            print(json.dumps(dict(native_visual_preview_passed=True,first_frame_preview=True,
                uncertain_visible=True,stale_retained=True,far_position_not_invented=True,mission_unchanged=True)),flush=True)
        finally:
            executor.shutdown(timeout_sec=3);thread.join(timeout=3)
            console.destroy_node();fixture.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
