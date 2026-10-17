import math
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from .core import rotation_matrix


LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def parameters(node, defaults):
    for name, value in defaults.items():
        node.declare_parameter(name, value)
    return {name: node.get_parameter(name).value for name in defaults}


def transform_arrays(transform):
    import numpy as np
    t, q = transform.transform.translation, transform.transform.rotation
    return rotation_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])


def set_yaw(quaternion, yaw):
    quaternion.z = math.sin(yaw/2)
    quaternion.w = math.cos(yaw/2)


def run(node_class):
    import rclpy
    rclpy.init()
    node = node_class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
