#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_sync.msg import SyncedSensors
from loguru import logger

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        logger.info("Calibration Node Initialized")
        self.subscription = self.create_subscription(
            SyncedSensors,
            '/synced_sensors',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        logger.info(f"Received Synced Message:")
        logger.info(f"Sync Time: {msg.sync_time}")
        logger.info(f"Camera Data: {msg.camera.header.stamp.sec}.{msg.camera.header.stamp.nanosec}")
        logger.info(f"Lidar Data: {msg.lidar.header.stamp.sec}.{msg.lidar.header.stamp.nanosec}")
        logger.info(f"Radar Data: {msg.radar.header.stamp.sec}.{msg.radar.header.stamp.nanosec}")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
