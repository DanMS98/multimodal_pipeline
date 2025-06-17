#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from loguru import logger
import os
import cv2
from cv_bridge import CvBridge
import yaml
import json

class MultiPointCloudSaver(Node):
    def __init__(self, merge_lidar_radar=False):
        super().__init__('multi_pointcloud_saver')

        self.merge_mode = merge_lidar_radar

        if self.merge_mode:
            self.merged_output_folder = os.path.join('Dataset', 'merged')
            os.makedirs(self.merged_output_folder, exist_ok=True)

            logger.info("Merging LiDAR and Radar point clouds into a single folder.")
        else:
            self.ouster_output_folder = os.path.join('Dataset', 'lidar')
            self.radar_output_folder = os.path.join('Dataset', 'radar')
            os.makedirs(self.ouster_output_folder, exist_ok=True)
            os.makedirs(self.radar_output_folder, exist_ok=True)
            logger.info("Saving LiDAR and Radar point clouds into separate folders.")

        self.image_output_folder = os.path.join('Dataset', 'images')
        # self.extrinsics_output_folder = os.path.join('Dataset', 'extrinsics')

        os.makedirs(self.image_output_folder, exist_ok=True)
        # os.makedirs(self.extrinsics_output_folder, exist_ok=True)


        self.ouster_frame_id = 0
        self.radar_frame_id = 0
        self.merged_frame_id = 0

        self.latest_radar_msg = None
        self.latest_camera_msg = None
        self.camera_info = None
        self.camera_info_saved = False

        # radar_topic = '/radar_data/point_cloud'
        # ouster_topic = '/ouster/points'
        radar_topic = '/radar_data/calib_pointcloud'
        ouster_topic = '/ouster/calib_pointcloud'
        camera_topic = '/camera/camera/color/synced_image'
        camera_info_topic = '/camera/camera/color/camera_info'

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

        self.create_subscription(
            Image,
            camera_topic,
            self.camera_callback,
            10
        )
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )

        self.bridge = CvBridge()

        logger.info("Subscribed to /synced/ouster_pointcloud and \
                     /synced/radar_pointcloud")
        

    def camera_info_callback(self, msg):
        if self.camera_info_saved:
            return

        intrinsics = {
            'width': int(msg.width),
            'height': int(msg.height),
            'K': [float(v) for v in msg.k],
            'distortion_model': msg.distortion_model,
            'D': [float(v) for v in msg.d],
        }

        os.makedirs('Dataset/camera_info', exist_ok=True)
        with open('Dataset/camera_info/intrinsics.yaml', 'w') as f:
            yaml.dump(intrinsics, f)

        logger.debug("Saved camera intrinsics to Dataset/camera_info/intrinsics.yaml")
        self.camera_info_saved = True

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
        file, img = self.rosimg_to_opencv(self.latest_camera_msg)


        if lidar_pts.size == 0 and radar_pts.size == 0:
            logger.error("Both point clouds are empty!")
            return
        
        all_pts = np.vstack([lidar_pts, radar_pts]) if radar_pts.size else lidar_pts

        pcd = o3d.t.geometry.PointCloud()
        pcd.point["positions"] = o3d.core.Tensor(all_pts[:, :3], dtype=o3d.core.Dtype.Float32)
        pcd.point["intensity"] = o3d.core.Tensor(all_pts[:, 3:], dtype=o3d.core.Dtype.Float32)

        filename = os.path.join(self.merged_output_folder,
                                 f'frame_{self.merged_frame_id:04d}.pcd')
        
        o3d.t.io.write_point_cloud(filename, pcd)
        cv2.imwrite(file, img)
        logger.info(f"Merged: Saved {filename} and camera image {file}")
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

    def rosimg_to_opencv(self, ros_image):
        try:
            img = self.bridge.imgmsg_to_cv2(ros_image,
                                             desired_encoding='bgr8')
            img_filename = os.path.join(self.image_output_folder,
                                         f'frame_{self.merged_frame_id:04d}.png')
            return img_filename, img
        except Exception as e:
            logger.error(f"Failed to save camera image: {e}")

    def camera_callback(self, msg):
        if self.merge_mode:
            self.latest_camera_msg = msg
            return
        file, img = self.rosimg_to_opencv(msg)
        cv2.imwrite(file, img)
            


def main(args=None):
    rclpy.init(args=args)
    saver = MultiPointCloudSaver(merge_lidar_radar=True)  # Set to True for merging, False for separate folders
    rclpy.spin(saver)
    saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
