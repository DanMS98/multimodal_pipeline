#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import numpy as np
from loguru import logger
import ros2_numpy as rnp




# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarReconstructor()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
