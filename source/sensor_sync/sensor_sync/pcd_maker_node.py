#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from loguru import logger
import os

class MultiPointCloudSaver(Node):
    def __init__(self, merge_lidar_radar=False):
        super().__init__('multi_pointcloud_saver')

        self.merge_mode = merge_lidar_radar

        if self.merge_mode:
            self.merged_output_folder = os.path.join('pcds', 'merged')
            os.makedirs(self.merged_output_folder, exist_ok=True)

            logger.info("Merging LiDAR and Radar point clouds into a single folder.")
        else:
            self.ouster_output_folder = os.path.join('pcds', 'lidar')
            self.radar_output_folder = os.path.join('pcds', 'radar')
            os.makedirs(self.ouster_output_folder, exist_ok=True)
            os.makedirs(self.radar_output_folder, exist_ok=True)
            logger.info("Saving LiDAR and Radar point clouds into separate folders.")


        self.ouster_frame_id = 0
        self.radar_frame_id = 0
        self.merged_frame_id = 0

        self.latest_radar_msg = None

        # radar_topic = '/radar_data/point_cloud'
        # ouster_topic = '/ouster/points'
        radar_topic = '/synced/radar_pointcloud'
        ouster_topic = '/synced/ouster_pointcloud'

        self.create_subscription(
            PointCloud2,
            ouster_topic,
            self.ouster_callback,
            10
        )

        self.create_subscription(
            PointCloud2,
            radar_topic,
            self.radar_callback,
            10
        )

        logger.info("Subscribed to /synced/ouster_pointcloud and /synced/radar_pointcloud")

    def process_pointcloud(self, msg, output_folder, frame_id, sensor_name):
        logger.info(f"Processing point cloud from {sensor_name}...")
        points_list = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points_list.append([p[0], p[1], p[2], p[3]])  # XYZ + intensity

        if not points_list:
            logger.error(f"{sensor_name}: Received empty point cloud!")
            return frame_id

        np_points = np.array(points_list, dtype=np.float32)

        pcd = o3d.t.geometry.PointCloud()
        pcd.point["positions"] = o3d.core.Tensor(np_points[:, :3], dtype=o3d.core.Dtype.Float32)
        pcd.point["intensity"] = o3d.core.Tensor(np_points[:, 3:], dtype=o3d.core.Dtype.Float32)

        filename = os.path.join(output_folder, f'frame_{frame_id:04d}.pcd')
        o3d.t.io.write_point_cloud(filename, pcd)
        logger.info(f"{sensor_name}: Saved {filename}")

        return frame_id + 1
    
    def extract_points(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points.append([p[0], p[1], p[2], p[3]])
        return np.array(points, dtype=np.float32) if points else np.empty((0, 4), dtype=np.float32)


    def merge_and_save(self, lidar_msg, radar_msg):
        logger.info("Merging LiDAR and Radar point clouds...")

        if radar_msg is None:
            logger.warning("Waiting for Radar message to merge with LiDAR," \
            " Skipping merge.")
            return
        
        lidar_pts = self.extract_points(lidar_msg)
        radar_pts = self.extract_points(radar_msg)

        if lidar_pts.size == 0 and radar_pts.size == 0:
            logger.error("Both point clouds are empty!")
            return
        
        all_pts = np.vstack([lidar_pts, radar_pts]) if radar_pts.size else lidar_pts

        pcd = o3d.t.geometry.PointCloud()
        pcd.point["positions"] = o3d.core.Tensor(all_pts[:, :3], dtype=o3d.core.Dtype.Float32)
        pcd.point["intensity"] = o3d.core.Tensor(all_pts[:, 3:], dtype=o3d.core.Dtype.Float32)

        filename = os.path.join(self.merged_output_folder, f'frame_{self.merged_frame_id:04d}.pcd')
        o3d.t.io.write_point_cloud(filename, pcd)
        logger.info(f"Merged: Saved {filename}")
        self.merged_frame_id += 1

        return self.merged_frame_id 

    
    def ouster_callback(self, msg):

        if self.merge_mode:
            self.merge_and_save(msg, self.latest_radar_msg)
        else:  
            self.ouster_frame_id = self.process_pointcloud(
                msg,
                self.ouster_output_folder,
                self.ouster_frame_id,
                "Ouster"
            )

    def radar_callback(self, msg):
        for f in msg.fields:
            f.count = 1

        if self.merge_mode:
            self.latest_radar_msg = msg
        else:
            # field_names = [f.name for f in msg.fields]
            # logger.info(f"Radar fields: {field_names}")
            
            self.radar_frame_id = self.process_pointcloud(
                msg,
                self.radar_output_folder,
                self.radar_frame_id,
                "Radar"
            )

def main(args=None):
    rclpy.init(args=args)
    saver = MultiPointCloudSaver(merge_lidar_radar=True)  # Set to True for merging, False for separate folders
    rclpy.spin(saver)
    saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
