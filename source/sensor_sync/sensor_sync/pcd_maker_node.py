#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import loguru
import os

class MultiPointCloudSaver(Node):
    def __init__(self):
        super().__init__('multi_pointcloud_saver')

        self.ouster_output_folder = 'lidar_pcds'
        self.radar_output_folder = 'radar_pcds'
        os.makedirs(self.ouster_output_folder, exist_ok=True)
        os.makedirs(self.radar_output_folder, exist_ok=True)

        self.ouster_frame_id = 0
        self.radar_frame_id = 0

        self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.ouster_callback,
            10
        )

        self.create_subscription(
            PointCloud2,
            '/radar_data/point_cloud',
            self.radar_callback,
            10
        )

        self.get_logger().info("Subscribed to /ouster/points and /radar_data/point_cloud")

    def process_pointcloud(self, msg, output_folder, frame_id, sensor_name):
        loguru.logger.info(f"Processing point cloud from {sensor_name}...")
        points_list = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points_list.append([p[0], p[1], p[2], p[3]])  # XYZ + intensity

        if not points_list:
            loguru.logger.error(f"{sensor_name}: Received empty point cloud!")
            return frame_id

        np_points = np.array(points_list, dtype=np.float32)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points[:, :3])

        intensity = np_points[:, 3]
        intensity_normalized = (intensity - np.min(intensity)) / (np.max(intensity) - np.min(intensity) + 1e-8)
        colors = np.tile(intensity_normalized.reshape(-1, 1), (1, 3))
        pcd.colors = o3d.utility.Vector3dVector(colors)

        filename = os.path.join(output_folder, f'frame_{frame_id:04d}.pcd')
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f"{sensor_name}: Saved {filename}")

        return frame_id + 1

    def ouster_callback(self, msg):
        pass
        # self.ouster_frame_id = self.process_pointcloud(
        #     msg,
        #     self.ouster_output_folder,
        #     self.ouster_frame_id,
        #     "Ouster"
        # )

    def radar_callback(self, msg):
        self.radar_frame_id = self.process_pointcloud(
            msg,
            self.radar_output_folder,
            self.radar_frame_id,
            "Radar"
        )

def main(args=None):
    rclpy.init(args=args)
    saver = MultiPointCloudSaver()
    rclpy.spin(saver)
    saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
