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
from sensor_msgs_py.point_cloud2 import create_cloud, read_points, create_cloud_xyz32
from geometry_msgs.msg import TransformStamped
import time
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point
import PyKDL
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud



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
        self.publisher = self.create_publisher(PointCloud2, 'transformed_cloud', 10)

        logger.info("Calibration Node Initialized")


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
        return transform
    

    # def transform_to_matrix(self, transform):
    #     t = transform.transform.translation
    #     q = transform.transform.rotation

    #     translation = np.array([t.x, t.y, t.z])
    #     rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    #     T = np.eye(4)
    #     T[:3, :3] = rotation
    #     T[:3, 3] = translation
    #     return T

    def transform_to_kdl(self, t):
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(
                            t.transform.rotation.x, t.transform.rotation.y,
                            t.transform.rotation.z, t.transform.rotation.w),
                        PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))
    
    def do_transform_cloud(self, cloud, transform):
        t_kdl = self.transform_to_kdl(transform)
        points_out = []
        for p_in in read_points(cloud, field_names=("x", "y", "z")):
            p_out = t_kdl * PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            points_out.append([p_out[0], p_out[1], p_out[2]])
        res = create_cloud_xyz32(transform.header, points_out)
        return res
            


    def listener_callback(self, msg: PointCloud2):

        
        tf_msg = self.lookup_transform(source_frame=msg.header.frame_id,
                              target_frame='base_link', print_tfs=True)
        # T = self.transform_to_matrix(tf_msg)
        transformed_cloud = self.do_transform_cloud(msg, tf_msg)
        logger.debug('transformed succedfully')
        logger.info(transformed_cloud.header)
        self.publisher.publish(transformed_cloud)

            
        # if T is None or T.shape != (4, 4):
        #     logger.error(f"Invalid transformation matrix: {T}")
        #     return
        ################
        # transformed_cloud = do_transform_cloud(msg, tf_msg)
        # transformed_cloud.header.frame_id = 'base_link'
        ################
     
       



    



def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
