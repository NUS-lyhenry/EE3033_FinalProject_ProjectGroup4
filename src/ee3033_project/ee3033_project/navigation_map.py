"""Supply Nav2 with the same conservative subcells used by path search."""
from copy import deepcopy

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from .path_validation import path_search_grid
from .ros_utils import LATCHED


def refine_map(message):
    """Preserve the source frame, stamp, origin and every occupied/unknown square."""
    if (message.header.frame_id != 'map' or message.info.width <= 0
            or message.info.height <= 0
            or len(message.data) != message.info.width*message.info.height):
        raise ValueError('Invalid navigation map')
    grid = np.asarray(message.data).reshape(message.info.height, message.info.width)
    if np.any((grid < -1) | (grid > 100)):
        raise ValueError('Invalid occupancy labels')
    refined, resolution, _ = path_search_grid(grid, message.info.resolution)
    result = OccupancyGrid()
    result.header = deepcopy(message.header)
    result.info = deepcopy(message.info)
    result.info.resolution = resolution
    result.info.height, result.info.width = refined.shape
    result.data = refined.ravel().tolist()
    return result


class NavigationMap(Node):
    def __init__(self):
        super().__init__('project_navigation_map')
        self.publisher = self.create_publisher(OccupancyGrid, '/project/navigation_map', LATCHED)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)

    def on_map(self, message):
        try:
            self.publisher.publish(refine_map(message))
        except (ValueError, TypeError, OverflowError) as exc:
            self.get_logger().error('Navigation map rejected: '+str(exc))


def main():
    rclpy.init()
    node = NavigationMap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
