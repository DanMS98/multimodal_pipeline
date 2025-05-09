#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_sync.msg import SyncedSensors
from loguru import logger
from cv_bridge import CvBridge
import cv2
import numpy as np
import open3d as o3d

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        logger.info("Calibration Node Initialized")
        self.subscription = self.create_subscription(
            SyncedSensors,
            '/synced_sensors',
            self.listener_callback,
            10
        )
        self.camera = None
        self.lidar = None
        self.radar = None
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.camera = msg.camera
        self.lidar = msg.lidar
        self.radar = msg.radar
        logger.info(f"Received Synced Message:")
        logger.info(f"Sync Time: {msg.sync_time}")
        self.process_camera()
        self.process_lidar()
        self.process_radar()


    
    def process_camera(self, ):
        if self.camera is None:
            logger.error("CAMERA EMPTY.")
            return
        logger.info(f"Camera Time: {self.camera.header.stamp.sec}.{self.camera.header.stamp.nanosec}")
        frame = self.bridge.imgmsg_to_cv2(self.camera, desired_encoding='bgr8')
        frame = cv2.resize(frame, (960, 540)) 
        cv2.imshow('Camera Feed', frame)
        cv2.waitKey(1)
        

    def process_lidar(self, ):
        if self.lidar is None:
            logger.error("LIDAR EMPTY.")
            return
        
        logger.debug("Visualizing Lidar Data")
        lidar_data = self.pointcloud2_to_array(self.lidar)
        if lidar_data is not None:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(lidar_data[:, :3])  # Using only x, y, z coordinates
            o3d.visualization.draw_geometries([pcd])
        else:
            logger.warning("Lidar data is empty")

        logger.info(f"Lidar Time: {self.lidar.header.stamp.sec}.{self.lidar.header.stamp.nanosec}")

    def process_radar(self, ):
        if self.radar is None:
            logger.error("RADAR EMPTY.")
            return
        logger.info(f"Radar Time: {self.radar.header.stamp.sec}.{self.radar.header.stamp.nanosec}")


    def pointcloud2_to_array(self, pointcloud2_msg):
        try:
            # Extract field names and data type
            field_names = [field.name for field in pointcloud2_msg.fields]
            dtype_list = []
            seen_fields = set()

            for field in pointcloud2_msg.fields:
                # Ensure unique field names
                if field.name in seen_fields:
                    logger.warning(f"Duplicate field '{field.name}' encountered. Skipping.")
                    continue
                seen_fields.add(field.name)

                if field.datatype == 7:  # FLOAT32
                    dtype_list.append((field.name, np.float32))
                elif field.datatype == 8:  # FLOAT64
                    dtype_list.append((field.name, np.float64))
                elif field.datatype == 6:  # INT32
                    dtype_list.append((field.name, np.int32))
                elif field.datatype == 5:  # INT16
                    dtype_list.append((field.name, np.int16))
                elif field.datatype == 4:  # INT8
                    dtype_list.append((field.name, np.int8))
                elif field.datatype == 2:  # UINT16
                    dtype_list.append((field.name, np.uint16))
                elif field.datatype == 1:  # UINT8
                    dtype_list.append((field.name, np.uint8))
                else:
                    logger.warning(f"Unsupported datatype: {field.datatype}")
                    continue

            # Create numpy dtype
            dtype = np.dtype(dtype_list)

            # Calculate the expected size per point
            point_step = pointcloud2_msg.point_step

            # Check alignment
            if len(pointcloud2_msg.data) % point_step != 0:
                raise ValueError(f"Buffer size {len(pointcloud2_msg.data)} is not a multiple of point step {point_step}")

            # Convert to numpy array (raw data, byte-wise)
            raw_data = np.frombuffer(pointcloud2_msg.data, dtype=np.uint8).reshape((-1, point_step))
            logger.debug(f"Raw Data Shape: {raw_data.shape}, C-Contiguous: {raw_data.flags['C_CONTIGUOUS']}")

            # Initialize structured array as C-contiguous
            structured_data = np.zeros((len(raw_data),), dtype=dtype)
            structured_data = np.ascontiguousarray(structured_data)
            logger.debug(f"Structured Data Shape: {structured_data.shape}, C-Contiguous: {structured_data.flags['C_CONTIGUOUS']}")

            # Fill structured array
            for i, field in enumerate(pointcloud2_msg.fields):
                name = field.name
                offset = field.offset
                field_size = np.dtype(dtype_list[i][1]).itemsize

                # Extract data slice per point
                # Each slice should be a single value, not a range
                field_data = raw_data[:, offset:offset + field_size]

                # Reshape to 1D and cast
                field_data = field_data.view(dtype_list[i][1]).reshape(-1)
                field_data = np.ascontiguousarray(field_data)

                # Assign data to structured array
                structured_data[name] = field_data

                logger.debug(f"Field '{name}' - Shape: {field_data.shape}, Expected: {structured_data[name].shape}, C-Contiguous: {field_data.flags['C_CONTIGUOUS']}")

            # Ensure that the required fields are present
            required_fields = ['x', 'y', 'z']
            if not all(field in field_names for field in required_fields):
                raise ValueError(f"Expected fields {required_fields} not found in PointCloud2")

            # Extract x, y, z points and enforce C-contiguity
            points = np.stack([structured_data['x'], structured_data['y'], structured_data['z']], axis=-1)
            points = np.ascontiguousarray(points)

            logger.debug(f"Final Points - Shape: {points.shape}, C-Contiguous: {points.flags['C_CONTIGUOUS']}")

            return points

        except Exception as e:
            logger.error(f"Error converting PointCloud2: {e}")
            return None





def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
