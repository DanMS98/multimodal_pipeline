#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_sync.msg import SyncedSensors
from collections import deque
from loguru import logger
import cv2
from cv_bridge import CvBridge
from rclpy.parameter import Parameter




class SyncNode(Node):
    def __init__(self):
        
        super().__init__('sync_node')
        logger.info("Intializing SyncNode...")
        # self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.camera_buffer = deque(maxlen=100)
        self.lidar_buffer = deque(maxlen=100)
        self.radar_buffer = deque(maxlen=100)

        self.create_subscription(Image, '/camera/camera/color/image_raw', self.camera_callback, 10)
        self.create_subscription(PointCloud2, '/ouster/points', self.lidar_callback, 10)
        self.create_subscription(PointCloud2, '/radar_data/point_cloud', self.radar_callback, 10)

        self.sync_pub = self.create_publisher(SyncedSensors, '/synced_sensors', 10)

        # self.sync_rate = 10.0  # Hz
        # self.sync_threshold = 0.05  # seconds
        # self.timer = self.create_timer(1.0 / self.sync_rate, self.sync_callback)
        self.timer = self.create_timer(0.5, self.timer_callback)

        logger.info("SyncNode Intialized")


    def camera_callback(self, msg):
        # bridge = CvBridge()
        # frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # frame = cv2.resize(frame, (960, 540)) 
        # cv2.imshow('Camera Feed', frame)
        # cv2.waitKey(1)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # logger.info(f'Camera timestamp: {ts}')
        self.camera_buffer.append((ts, msg))

    def lidar_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # logger.info(f'Lidar timestamp: {ts}')
        self.lidar_buffer.append((ts, msg))

    def radar_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # logger.info(f'Radar timestamp: {ts}')
        self.radar_buffer.append((ts, msg))

    def find_closest(self, buffer, target_time):
        if not buffer:
            logger.error("BUFFER EMPTY")
            return None
        closest = min(buffer, key=lambda x: abs(x[0] - target_time))
        if abs(closest[0] - target_time) < self.sync_threshold:
            return closest[1]
        else:
            return None
        
    def find_latest(self, buffer):
        if not buffer:
            logger.error("BUFFER EMPTY")
            return None
        latest = buffer.pop()
        return latest[1]

    def timer_callback(self):
        logger.debug("Sync Callback started...")

        now = self.get_clock().now().seconds_nanoseconds()
        sync_time = now[0] + now[1] * 1e-9
        ros_time = self.get_clock().now().to_msg()

        # camera_msg = self.find_closest(self.camera_buffer, sync_time)
        # lidar_msg = self.find_closest(self.lidar_buffer, sync_time)
        # radar_msg = self.find_closest(self.radar_buffer, sync_time)
        camera_msg = self.find_latest(self.camera_buffer)
        lidar_msg = self.find_latest(self.lidar_buffer)
        radar_msg = self.find_latest(self.radar_buffer)


        if camera_msg and lidar_msg and radar_msg:
            synced_msg = SyncedSensors()
            synced_msg.sync_time = ros_time
            synced_msg.camera = camera_msg
            synced_msg.lidar = lidar_msg
            synced_msg.radar = radar_msg
            logger.info(f"sync_time: {synced_msg.sync_time}")
            logger.info(f"camera frame_id: {synced_msg.camera.header.frame_id}")
            logger.info(f"lidar frame_id: {synced_msg.lidar.header.frame_id}")
            logger.info(f"radar frame_id: {synced_msg.radar.header.frame_id}")

            logger.info(f"camera size: {synced_msg.camera.width}x{synced_msg.camera.height}")
            logger.info(f"lidar point cloud height: {synced_msg.lidar.height}, width: {synced_msg.lidar.width}")
            logger.info(f"radar point cloud height: {synced_msg.radar.height}, width: {synced_msg.radar.width}")

            logger.info("Publish synced message...")

            self.sync_pub.publish(synced_msg)
        else:
            logger.warning("Couldn't publish the message")
            

def main(args=None):
    rclpy.init(args=args)
    node = SyncNode()
    # node.use_sim_time = True
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
