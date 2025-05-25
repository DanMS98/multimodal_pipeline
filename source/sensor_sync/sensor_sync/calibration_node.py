#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_sync.msg import SyncedSensors
from loguru import logger
from cv_bridge import CvBridge
import cv2
import numpy as np
import open3d as o3d
import struct
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import ros2_numpy as rnp
import struct
import matplotlib.pyplot as plt
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_ros
from geometry_msgs.msg import TransformStamped
import time
from scipy.spatial.transform import Rotation as R


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,
                                              self,
                                                spin_thread=True)
        # self.timer = self.create_timer(2.0, self.lookup_all_transforms)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.listener_callback,
            10
        )
        self.lidar_pub = self.create_publisher(PointCloud2, '/ouster/points_base', 10)

        logger.info("Calibration Node Initialized")

        # self.subscription = self.create_subscription(
        #     SyncedSensors,
        #     '/synced_sensors',
        #     self.listener_callback,
        #     10
        # )

        

        # self.l_utils = LidarUtils()
        # # self.sync_pub = self.create_publisher(SyncedSensors, '/synced_sensors', 10)
        # # self.timer = self.create_timer(0.5, self.timer_callback)

        # self.camera = None
        # self.lidar = None
        # self.radar = None
        # self.visualize = False
        # self.bridge = CvBridge()

    def lookup_all_transforms(self):
        self.lookup_transform('os_lidar', 'base_link')
        self.lookup_transform('camera_color_optical_frame', 'base_link')
        self.lookup_transform('laser_frame', 'base_link')

    def lookup_transform(self, source_frame, target_frame, print_tfs=True):
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=rclpy.time.Time()
            )
            if print_tfs:
                logger.debug(f'Transform {source_frame} -> {target_frame}:')
                logger.info(f'Translation: {transform.transform.translation}')
                logger.info(f'Rotation: {transform.transform.rotation}')
        except Exception as e:
            logger.error(f'Failed to get transform: {e}')
        return self.transform_to_matrix(transform)

    def transform_to_matrix(self, transform):
        t = transform.transform.translation
        q = transform.transform.rotation

        translation = np.array([t.x, t.y, t.z])
        rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = translation
        return T


    def listener_callback(self, msg: PointCloud2):
        # pc_np = rnp.numpify(msg)
        # pc_xyz = rnp.get_xyz_points(pc_np, remove_nans=True)
        # points = list(pc2.read_points(msg,
        #                                skip_nans=True,
        #                                  field_names=None))
        points = pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True)
        if len(points) == 0:
            logger.error("No valid LiDAR points to transform.")
            return
        
        T = self.lookup_transform(source_frame=msg.header.frame_id,
                              target_frame='base_link', print_tfs=True)
        if T is None or T.shape != (4, 4):
            logger.error(f"Invalid transformation matrix: {T}")
            return
        
        xyz = np.array([[p[0], p[1], p[2]] for p in points])
        num_points = xyz.shape[0]
        homo_xyz = np.hstack((xyz, np.ones((num_points, 1))))
        transformed_xyz = (T @ homo_xyz.T).T[:, :3]
        logger.debug(f'transformed_xyz:{transformed_xyz}')
        # reconstruct 
        field_names = [f.name for f in msg.fields]
        full_dtype = pc2.PointCloud2(msg).fields
        fields = msg.fields

        transformed_points = []
        for i, p in enumerate(points):
            p = list(p)
            p[0], p[1], p[2] = transformed_xyz[i]
            transformed_points.append(tuple(p))

        # newPointCloud2
        transformed_msg = pc2.create_cloud(
            header=msg.header,
            fields=fields,
            points=transformed_points
        )
        transformed_msg.header.frame_id = 'base_link'
        self.lidar_pub.publish(transformed_msg)
        logger.debug("Published transformed LiDAR point cloud")



    



def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
