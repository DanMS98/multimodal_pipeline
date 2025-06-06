#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import os

class PointCloudSaver(Node):
    def __init__(self, output_folder='output_pcds'):
        super().__init__('pointcloud_saver')
        self.output_folder = output_folder
        os.makedirs(self.output_folder, exist_ok=True)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',  
            self.listener_callback,
            10)
        self.frame_id = 0
        self.get_logger().info(f"Ready to save PointCloud2 frames to folder: {self.output_folder}")

    def listener_callback(self, msg):
        points_list = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points_list.append([p[0], p[1], p[2], p[3]])  # XYZ + intensity

        if not points_list:
            self.get_logger().warn("Received empty point cloud!")
            return

        np_points = np.array(points_list, dtype=np.float32)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points[:, :3])

        intensity = np_points[:, 3]
        intensity_normalized = (intensity - np.min(intensity)) / (np.max(intensity) - np.min(intensity) + 1e-8)
        colors = np.tile(intensity_normalized.reshape(-1, 1), (1, 3))
        pcd.colors = o3d.utility.Vector3dVector(colors)

        filename = os.path.join(self.output_folder, f'frame_{self.frame_id:04d}.pcd')
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f"Saved: {filename}")
        self.frame_id += 1

def main(args=None):
    rclpy.init(args=args)
    output_folder = 'output_pcds'  
    saver = PointCloudSaver(output_folder=output_folder)
    rclpy.spin(saver)
    saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
