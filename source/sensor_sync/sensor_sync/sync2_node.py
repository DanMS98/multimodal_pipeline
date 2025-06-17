#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from rclpy.time import Time
from collections import deque
import copy
from loguru import logger

class RadarLidarSyncNode(Node):
    def __init__(self):
        super().__init__('radar_lidar_sync_node')

        self.radar_buffer = deque(maxlen=50)
        self.camera_buffer = deque(maxlen=50)

        lidar_topic = '/ouster/points'
        radar_topic = '/radar_data/point_cloud'
        camera_topic = '/camera/camera/color/image_raw'

        self.create_subscription(PointCloud2, lidar_topic, self.lidar_callback, 10)
        self.create_subscription(PointCloud2, radar_topic, self.radar_callback, 10)
        self.create_subscription(Image, camera_topic, self.camera_callback, 10)

        lidar_publish_topic = '/ouster/synced_pointcloud'
        radar_publish_topic = '/radar_data/synced_pointcloud'
        camera_publish_topic = '/camera/synced_image'

        self.lidar_pub = self.create_publisher(PointCloud2, lidar_publish_topic, 10)
        self.radar_pub = self.create_publisher(PointCloud2, radar_publish_topic, 10)
        self.camera_pub = self.create_publisher(Image, camera_publish_topic, 10)

        logger.info("Radar-LiDAR sync node initialized with buffering and /synced outputs.")


    def camera_callback(self, msg):
        camera_stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.camera_buffer.append((camera_stamp_ns, msg))


    def radar_callback(self, msg):
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        self.radar_buffer.append((stamp_ns, msg))

    def lidar_callback(self, lidar_msg):
        """
            Camera timestamps are wrong (57 years, it's UNIX time while
            Lidar is ROS time),so it is being ignored for now. it should be
            fixed in the future.
        """
        lidar_stamp_ns = Time.from_msg(lidar_msg.header.stamp).nanoseconds

        matched_radar_msg = None
        matched_camera_msg = None


        # matched_camera_msg = self.find_closest_before(self.camera_buffer,
        #                                                 lidar_stamp_ns, debug=True)
        # 
        # if matched_camera_msg is None:
        #     logger.warning("No matching camera message found for current LiDAR frame. Skipping sync.")  
        #     return
         
        matched_radar_msg = self.find_closest_before(self.radar_buffer,
                                                      lidar_stamp_ns)
        if matched_radar_msg is None:
            logger.warning("No matching radar message found for current LiDAR frame. Skipping sync.")
            return
        
        if not self.camera_buffer:
            logger.warning("No camera messages received yet. Skipping sync.")
            return
        
        _, matched_camera_msg = self.camera_buffer[-1]

        synced_radar = copy.deepcopy(matched_radar_msg)
        synced_camera = copy.deepcopy(matched_camera_msg)
        
        synced_radar.header.stamp = lidar_msg.header.stamp
        synced_camera.header.stamp = lidar_msg.header.stamp

        self.lidar_pub.publish(lidar_msg)
        self.radar_pub.publish(synced_radar)
        self.camera_pub.publish(synced_camera)

        logger.info(
            f"Published synced LiDAR and Radar and Camera @ t={lidar_msg.header.stamp.sec}.{lidar_msg.header.stamp.nanosec}"
        )

    def find_closest_before(self, buffer, target_stamp_ns, debug=False):
        for stamp_ns, msg in reversed(buffer):
            logger.debug(f"Checking message @ t={stamp_ns} against target {target_stamp_ns}")
            if stamp_ns <= target_stamp_ns:
                if debug:
                    logger.debug(f"Found matching message @ t={stamp_ns}")
                return msg
        return None


def main(args=None):
    rclpy.init(args=args)
    node = RadarLidarSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
