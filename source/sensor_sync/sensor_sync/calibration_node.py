#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_sync.msg import SyncedSensors
from loguru import logger
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, Image
import struct
import matplotlib.pyplot as plt
from tf2_ros import Buffer, TransformListener
from sensor_msgs_py.point_cloud2 import create_cloud, read_points, create_cloud_xyz32
from geometry_msgs.msg import TransformStamped
import time
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point
import PyKDL



class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,
                                              self,
                                                spin_thread=True)
        # self.timer = self.create_timer(2.0, self.lookup_all_transforms)


        self.lidar_topic = '/ouster/points'
        self.radar_topic = '/radar_data/point_cloud'
        self.camera_topic = '/camera/color/image_raw'

        self.lidar_topic = '/synced/ouster_pointcloud'
        self.radar_topic = '/synced/radar_pointcloud'

        self.bridge = CvBridge()

        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.ouster_callback,
            10
        )
        self.radar_subscription = self.create_subscription(
            PointCloud2,
            self.radar_topic,
            self.radar_callback,
            10
        )

        # self.create_subscription(Image, '/camera/camera/color/image_raw', self.camera_callback, 10)

        self.camera_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # self.lidar_pub = self.create_publisher(PointCloud2, '/ouster/points_base', 10)
        # self.publisher = self.create_publisher(PointCloud2, 'transformed_cloud', 10)

        self.calib_ouster_pub = self.create_publisher(PointCloud2, '/calib/ouster_pointcloud', 10)
        self.calib_radar_pub = self.create_publisher(PointCloud2, '/calib/radar_pointcloud', 10)


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
    

    def image_callback(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            logger.error(f"Failed to convert image: {e}")
            return

        tf_msg = self.lookup_transform(source_frame=msg.header.frame_id,
                                    target_frame='base_link',
                                    print_tfs=False)

        # if tf_msg:
        #     cam_to_base = self.transform_to_matrix(tf_msg)
        #     logger.info(f"Got camera → base_link transform:\n{cam_to_base}")
        

    def transform_to_matrix(self, transform):
        t = transform.transform.translation
        q = transform.transform.rotation

        translation = np.array([t.x, t.y, t.z])
        rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = translation
        return T

    def transform_to_kdl(self, t):
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(
                            t.transform.rotation.x, t.transform.rotation.y,
                            t.transform.rotation.z, t.transform.rotation.w),
                        PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))
    
    def do_transform_cloud(self, cloud, transform):
        t_kdl = self.transform_to_kdl(transform)
        # T = self.transform_to_matrix(transform)
        fields = cloud.fields
        header = cloud.header
        points_out = []

        has_intensity = any(f.name == 'intensity' for f in fields)

        for p in read_points(cloud, field_names=[f.name for f in fields], skip_nans=True):
            xyz = PyKDL.Vector(p[0], p[1], p[2])
            p_out = t_kdl * xyz
            intensity = p[3] if has_intensity else 0.0
            points_out.append((p_out[0], p_out[1], p_out[2], intensity))

        
        fields_out = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        return create_cloud(header, fields_out, points_out)


    def ouster_callback(self, msg: PointCloud2):
        tf_msg = self.lookup_transform(source_frame=msg.header.frame_id,
                                       target_frame='base_link',
                                       print_tfs=False)
        if tf_msg:
            transformed = self.do_transform_cloud(msg, tf_msg)
            self.calib_ouster_pub.publish(transformed)
            logger.info("Published transformed LiDAR to /calib/ouster_pointcloud")

    def radar_callback(self, msg: PointCloud2):
        for f in msg.fields:
            f.count = 1
        tf_msg = self.lookup_transform(source_frame=msg.header.frame_id,
                                       target_frame='base_link',
                                       print_tfs=False)
        if tf_msg:
            transformed = self.do_transform_cloud(msg, tf_msg)
            self.calib_radar_pub.publish(transformed)
            logger.info("Published transformed Radar to calib/radar_pointcloud")

     
       





def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
