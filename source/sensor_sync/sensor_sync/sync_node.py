#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_sync.msg import SyncedSensors
from collections import deque

class SyncNode(Node):
    def __init__(self):
        super().__init__('sync_node')
        self.camera_buffer = deque(maxlen=100)
        self.lidar_buffer = deque(maxlen=100)
        self.radar_buffer = deque(maxlen=100)

        self.create_subscription(Image, '/camera/camera/color/image_raw', self.camera_callback, 10)
        self.create_subscription(PointCloud2, '/ouster/points', self.lidar_callback, 10)
        self.create_subscription(PointCloud2, '/radar_data/point_cloud', self.radar_callback, 10)

        self.sync_pub = self.create_publisher(SyncedSensors, '/synced_sensors', 10)

        self.sync_rate = 10.0  # Hz
        self.sync_threshold = 0.05  # seconds
        self.timer = self.create_timer(1.0 / self.sync_rate, self.sync_callback)

    def camera_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.camera_buffer.append((ts, msg))

    def lidar_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.lidar_buffer.append((ts, msg))

    def radar_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.radar_buffer.append((ts, msg))

    def find_closest(self, buffer, target_time):
        if not buffer:
            return None
        closest = min(buffer, key=lambda x: abs(x[0] - target_time))
        if abs(closest[0] - target_time) < self.sync_threshold:
            return closest[1]
        else:
            return None

    def sync_callback(self):
        now = self.get_clock().now().seconds_nanoseconds()
        sync_time = now[0] + now[1] * 1e-9
        ros_time = self.get_clock().now().to_msg()

        camera_msg = self.find_closest(self.camera_buffer, sync_time)
        lidar_msg = self.find_closest(self.lidar_buffer, sync_time)
        radar_msg = self.find_closest(self.radar_buffer, sync_time)

        if camera_msg and lidar_msg and radar_msg:
            synced_msg = SyncedSensors()
            synced_msg.sync_time = ros_time
            synced_msg.camera = camera_msg
            synced_msg.lidar = lidar_msg
            synced_msg.radar = radar_msg

            self.sync_pub.publish(synced_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
