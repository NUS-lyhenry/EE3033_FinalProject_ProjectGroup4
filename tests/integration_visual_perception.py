"""Actual image/CameraInfo/TF pipeline; only neural inference is substituted."""
import json
import math
import os
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from cv_bridge import CvBridge
from ee3033_project.perception import Perception
from ee3033_project.calibration import mounting_rotation
from integration_ros import overrides, wait


class Array:
    def __init__(self, value): self.value = np.array(value)
    def cpu(self): return self
    def numpy(self): return self.value


class Detector:
    names = {0: 'bottle'}
    box = [308., 266., 332., 316.]
    def predict(self, frame, **kwargs):
        # Sensor callbacks must remain fresh during a slow CPU inference.
        time.sleep(.30)
        box = SimpleNamespace(cls=SimpleNamespace(item=lambda: 0), conf=SimpleNamespace(item=lambda: .9), xyxy=[Array(self.box)])
        return [SimpleNamespace(boxes=[box], names=self.names, plot=lambda: frame.copy())]


def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None, '0', '6'):
        raise SystemExit('Use an isolated ROS domain')
    rclpy.init()
    fixture = Node('visual_sensor_fixture')
    detector = Detector()
    with patch.object(Perception, 'load_detector', return_value=detector):
        perception = Perception(parameter_overrides=overrides(target_class='bottle',
            image_topic='/test/image', camera_info_topic='/test/camera', inference_period=.08))
    broadcaster = StaticTransformBroadcaster(fixture)
    transforms = []
    for parent, child, xyz, rotation in [
        ('map', 'base_footprint', [0., 0., 0.], np.eye(3)),
        ('base_footprint', 'project_camera_optical', [0., 0., .22], mounting_rotation(math.radians(10), 0))]:
        tr = TransformStamped(); tr.header.stamp = fixture.get_clock().now().to_msg()
        tr.header.frame_id = parent; tr.child_frame_id = child
        tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = xyz
        q = Rotation.from_matrix(rotation).as_quat()
        tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w = map(float, q)
        transforms.append(tr)
    broadcaster.sendTransform(transforms)
    image_pub = fixture.create_publisher(Image, '/test/image', 10)
    info_pub = fixture.create_publisher(CameraInfo, '/test/camera', 10)
    odom_pub = fixture.create_publisher(Odometry, '/odom', 10)
    pose_pub = fixture.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
    observations, detections, statuses = [], [], []
    for topic, output in [('/project/target_observations', observations), ('/project/target_detections', detections), ('/project/perception_status', statuses)]:
        fixture.create_subscription(String, topic, lambda msg, output=output: output.append(json.loads(msg.data)), 10)
    bridge = CvBridge()
    bad_frame = [False]
    def publish():
        image = bridge.cv2_to_imgmsg(np.zeros((480, 640, 3), np.uint8), encoding='bgr8')
        image.header.stamp = fixture.get_clock().now().to_msg(); image.header.frame_id = 'project_camera_optical'
        info = CameraInfo(); info.header.stamp = image.header.stamp
        info.header.frame_id = 'wrong_camera' if bad_frame[0] else image.header.frame_id
        info.width = 640; info.height = 480
        info.k = [450., 0., 320., 0., 450., 240., 0., 0., 1.]; info.d = [0.]*5; info.distortion_model = 'plumb_bob'
        info_pub.publish(info); image_pub.publish(image)
        odom = Odometry(); odom.header = image.header
        odom_pub.publish(odom)
        pose = PoseWithCovarianceStamped(); pose.header.stamp = image.header.stamp
        pose.header.frame_id = 'map'; pose.pose.pose.orientation.w = 1.
        pose.pose.covariance[0] = pose.pose.covariance[7] = pose.pose.covariance[35] = .0001
        pose_pub.publish(pose)
    fixture.create_timer(.1, publish)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (fixture, perception): executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    try:
        # Separate ROS topics may be delivered in either order.
        assert wait(lambda: observations and observations[-1]['observations']
                    and statuses and statuses[-1].get('geometry_ready')), (statuses[-1:], detections[-1:])
        assert fixture.count_subscribers('/scan_fixed') == 0
        assert statuses[-1]['geometry_ready'] and statuses[-1]['camera_model']
        assert observations[-1]['observations'][0]['method'] == 'ground_plane'
        assert observations[-1]['observations'][0]['sigma'] <= .10
        # A far contact remains a coarse candidate but cannot become a report.
        detector.box = [308., 150., 332., 220.]
        assert wait(lambda: detections and detections[-1]['detections'] and detections[-1]['detections'][0].get('sigma', 0) > .10)
        assert wait(lambda: observations and not observations[-1]['observations'])
        detector.box = [0., 200., 20., 316.]
        assert wait(lambda: detections[-1]['detections'][0].get('reason', '').startswith('Target is too small'))
        assert not observations[-1]['observations']
        bad_frame[0] = True
        assert wait(lambda: statuses[-1].get('geometry_ready') is False)
        assert 'frame' in statuses[-1]['geometry_error']
        assert not observations[-1]['observations']
        print('PASS native visual perception: no lidar, no fitted profile, geometry and uncertainty checks', flush=True)
    finally:
        executor.shutdown(timeout_sec=3); thread.join(timeout=3)
        perception.destroy_node(); fixture.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
