#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.time import Time
from collections import deque
import copy
from loguru import logger

class RadarLidarSyncNode(Node):
    def __init__(self):
        super().__init__('radar_lidar_sync_node')

        self.radar_buffer = deque(maxlen=50)

        lidar_topic = '/ouster/points'
        radar_topic = '/radar_data/point_cloud'

        self.create_subscription(PointCloud2, lidar_topic, self.lidar_callback, 10)
        self.create_subscription(PointCloud2, radar_topic, self.radar_callback, 10)

        lidar_publish_topic = '/ouster/synced_pointcloud'
        radar_publish_topic = '/radar_data/synced_pointcloud'

        self.lidar_pub = self.create_publisher(PointCloud2, lidar_publish_topic, 10)
        self.radar_pub = self.create_publisher(PointCloud2, radar_publish_topic, 10)
        
        logger.info("Radar-LiDAR sync node initialized with buffering and /synced outputs.")

    def radar_callback(self, msg):
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.radar_buffer.append((stamp_ns, msg))

    def lidar_callback(self, lidar_msg):
        lidar_stamp_ns = Time.from_msg(lidar_msg.header.stamp).nanoseconds

        matched_radar_msg = None
        for stamp_ns, radar_msg in reversed(self.radar_buffer):
            if stamp_ns <= lidar_stamp_ns:
                matched_radar_msg = radar_msg
                break

        if matched_radar_msg is None:
            logger.warning("No matching radar message found for current LiDAR frame.")
            return

        synced_radar = copy.deepcopy(matched_radar_msg)
        synced_radar.header.stamp = lidar_msg.header.stamp

        self.lidar_pub.publish(lidar_msg)
        self.radar_pub.publish(synced_radar)

        logger.info(
            f"Published synced LiDAR and Radar @ t={lidar_msg.header.stamp.sec}.{lidar_msg.header.stamp.nanosec}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RadarLidarSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
