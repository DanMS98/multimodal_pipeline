#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from loguru import logger
from scipy.spatial.transform import Rotation as R



class LidarToCameraVisualizer(Node):
    def __init__(self, use_synced_topics=False):
        super().__init__('lidar_to_camera_visualizer')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.intrinsics = None
        self.cam_frame = 'camera_color_optical_frame'

        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                                  self.camera_info_callback, 10)
        
        if use_synced_topics:
            self.create_subscription(Image, '/camera/camera/color/synced_image',
                                      self.image_callback, 10)
            self.create_subscription(PointCloud2, '/ouster/synced_pointcloud',
                                      self.lidar_callback, 10)
            self.create_subscription(PointCloud2, '/radar_data/synced_pointcloud',
                                      self.radar_callback, 10)
        else:
            self.create_subscription(Image, '/camera/camera/color/image_raw',
                                    self.image_callback, 10)
            self.create_subscription(PointCloud2, '/ouster/points',
                                    self.lidar_callback, 10)
            self.create_subscription(PointCloud2, '/radar_data/point_cloud',
                                    self.radar_callback, 10)

        self.latest_image = None
        self.latest_radar_frame = None
        logger.info(f"Lidar to Camera Visualizer Node Initialized. Using {'synced' if use_synced_topics else 'default'} topics.")


    def camera_info_callback(self, msg):
        K = np.array(msg.k).reshape(3, 3)
        self.intrinsics = K

    def transform_to_matrix(self, transform: TransformStamped):
        t = transform.transform.translation
        q = transform.transform.rotation
        translation = np.array([t.x, t.y, t.z])
        rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = translation
        return T

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def radar_callback(self, msg):
        for f in msg.fields:
            f.count = 1
        self.latest_radar_frame = msg

    def lidar_callback(self, msg):
        if self.latest_image is None or self.intrinsics is None:
            logger.warning("Waiting for camera image or intrinsics.returning")
            return
        if self.latest_radar_frame is None:
            logger.warning("Waiting for radar frame. Returning")
            return
        
        vis_img = self.latest_image.copy()
        vis_img = self.project_and_draw(msg, vis_img, sensor_name='LiDAR')
        vis_img = self.project_and_draw(self.latest_radar_frame, vis_img, sensor_name='Radar')

        vis_img = cv2.resize(vis_img, (640, 480))
        cv2.imshow("Projected LiDAR on Camera", vis_img)
        cv2.waitKey(1)

    def project_and_draw(self, msg, image, sensor_name='Sensor'):
        try:
            tf_msg: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame=self.cam_frame,
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time())
        except Exception as e:
            logger.warning(f"[{sensor_name}] Transform lookup failed: {e}")
            return image

        T = self.transform_to_matrix(tf_msg)

        points_list = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points_list.append([p[0], p[1], p[2], p[3]])

        if not points_list:
            logger.warning(f"[{sensor_name}] Empty point cloud.")
            return image

        points = np.array([p[:3] for p in points_list])
        intensities = np.array([p[3] for p in points_list])
        points_hom = np.hstack((points, np.ones((points.shape[0], 1))))
        cam_points_all = (T @ points_hom.T).T[:, :3]

        valid_mask = cam_points_all[:, 2] > 0
        cam_points = cam_points_all[valid_mask]
        cam_intensities = intensities[valid_mask]

        img_pts = (self.intrinsics @ cam_points.T).T
        img_pts = img_pts[:, :2] / img_pts[:, 2:]

        for (u, v), pt, intensity in zip(img_pts.astype(np.int32), cam_points, cam_intensities):
            if 0 <= u < image.shape[1] and 0 <= v < image.shape[0]:
                # color_val = int(np.clip(intensity / 255.0, 0.0, 1.0) * 255) * 10
                range_m = np.linalg.norm(pt)
                brightness = int(np.clip(1.0 - (range_m / 30.0), 0.0, 1.0) * 255)
                if sensor_name == 'LiDAR':
                    color = (0, brightness, 0)  
                    size = 4
                else:
                    color = (0, 0, brightness)
                    size = 6
                cv2.circle(image, (u, v), size, color, -1)

        return image


def main(args=None):

    rclpy.init(args=args)
    node = LidarToCameraVisualizer(use_synced_topics=True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
