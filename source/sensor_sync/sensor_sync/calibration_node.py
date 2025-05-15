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
import ros2_numpy as rnp
import struct
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class LidarUtils(Node):
    def __init__(self):
        pass

    def lidar_reconstructor(self, msg):
        try:
            point_step = msg.point_step
            data = msg.data
            width = msg.width
            height = msg.height
            nan_count = 0
            total_points = width * height

            valid_count = 0
            
            reconstructed_points = []
            for i in range(total_points):
                start_index = i * point_step
                end_index = start_index + point_step
                point_data = data[start_index:end_index]
                try:
                    x = struct.unpack_from('f', point_data, 0)[0]
                    y = struct.unpack_from('f', point_data, 4)[0]
                    z = struct.unpack_from('f', point_data, 8)[0]
                    range_val = struct.unpack_from('f', point_data, offset=32)[0]
                    intensity = struct.unpack_from('f', point_data, offset=16)[0]
                    ring = struct.unpack_from('H', point_data, offset=26)[0]
                    if any([x != x, y != y, z != z]):  # Check for nan using `!=` comparison
                        nan_count += 1
                    else:
                        valid_count += 1
                    # logger.info(f"Point {i}: X: {x}, Y: {y}, Z: {z}, Intensity: {intensity}, Range: {range_val}")
                    # if not np.isnan(range_val) and range_val > 0.001:
                        # logger.error(f"Range: {range_val}, Intensity: {intensity}, Ring: {ring}")
                        # exit()
                    if not np.isnan(range_val) and range_val > 0:
                        #  (assumes 64-channel LiDAR)
                        vertical_angle = np.deg2rad(-30 + 60 * (ring / 63))
                        #  (assumes 360° coverage)
                        horizontal_angle = np.deg2rad((i % width) * 360 / width)

                        # Convert to Cartesian coordinates
                        x = range_val * np.cos(vertical_angle) * np.cos(horizontal_angle)
                        y = range_val * np.cos(vertical_angle) * np.sin(horizontal_angle)
                        z = range_val * np.sin(vertical_angle)

                        reconstructed_points.append([x, y, z, intensity])
                    else:
                        # logger.warning(f'range_val: {range_val}')
                        reconstructed_points.append([np.nan, np.nan, np.nan, intensity])

                except struct.error as e:
                    logger.error(f"Data unpacking error at point {i}: {e}")
            logger.info(f"Total Points: {total_points}")
            logger.info(f"Valid Points: {valid_count}")
            logger.info(f"NaN Points: {nan_count}")
            logger.info(f"Percentage of Valid Points: {(valid_count / total_points) * 100:.2f}%")
            logger.info(f"Percentage of NaN Points: {(nan_count / total_points) * 100:.2f}%\n")

            # Convert to numpy array
            points_array = np.array(reconstructed_points, dtype=np.float32)
            logger.info(f"Reconstructed PointCloud2 with shape: {points_array.shape}")
            if points_array.shape[0] > 0:
                self.visualize_points(points_array)
            else:
                logger.warning("No valid points to visualize.")

        except Exception as e:
            logger.error(f"Error in lidar_callback: {e}")
        return points_array
    


    def visualize_points(self, points):
        """ Live 3D visualization using Matplotlib. """
        try:
            # Extract X, Y, Z, Intensity
            x_vals = points[:, 0]
            y_vals = points[:, 1]
            z_vals = points[:, 2]
            intensity = points[:, 3]

            # Plot with Matplotlib
            fig = plt.figure(figsize=(10, 7))
            ax = fig.add_subplot(111, projection='3d')

            # Scatter plot with intensity-based coloring
            scatter = ax.scatter(x_vals, y_vals, z_vals, c=intensity, cmap='viridis', s=1)
            plt.colorbar(scatter, ax=ax, label='Intensity')

            ax.set_title('3D PointCloud Visualization')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            plt.show()

        except Exception as e:
            logger.error(f"Visualization Error: {e}")   


    def lidar_test(self, msg):
        try:
            point_step = msg.point_step
            data = msg.data

            for i in range(3):
                start_index = i * point_step
                end_index = start_index + point_step
                point_data = data[start_index:end_index]

                try:
                    x, y, z = struct.unpack_from('fff', point_data, offset=0)
                    intensity = struct.unpack_from('f', point_data, offset=16)[0]
                    timestamp = struct.unpack_from('I', point_data, offset=20)[0]
                    reflectivity = struct.unpack_from('H', point_data, offset=24)[0]
                    ring = struct.unpack_from('H', point_data, offset=26)[0]
                    ambient = struct.unpack_from('H', point_data, offset=28)[0]
                    point_range = struct.unpack_from('f', point_data, offset=32)[0]

                    logger.info(f"Point {i}: XYZ: ({x}, {y}, {z}), Intensity: {intensity}, "
                                f"Timestamp: {timestamp}, Reflectivity: {reflectivity}, "
                                f"Ring: {ring}, Ambient: {ambient}, Range: {point_range}")

                except struct.error as e:
                    logger.error(f"Data unpacking error for point {i}: {e}")

        except Exception as e:
            logger.error(f"Error during manual extraction: {e}")


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.subscription = self.create_subscription(
            SyncedSensors,
            '/synced_sensors',
            self.listener_callback,
            10
        )

        self.l_utils = LidarUtils()
        # self.sync_pub = self.create_publisher(SyncedSensors, '/synced_sensors', 10)
        # self.timer = self.create_timer(0.5, self.timer_callback)

        self.camera = None
        self.lidar = None
        self.radar = None
        self.visualize = False
        self.bridge = CvBridge()
        logger.info("Calibration Node Initialized")


    def listener_callback(self, msg):
        self.camera = msg.camera
        self.lidar = msg.lidar
        self.radar = msg.radar
        logger.info(f"Received Synced Message:")
        logger.info(f"Sync Time: {msg.sync_time}")
        
        logger.debug(f'lidar_points:{self.lidar.header}')

        lidar_points = self.l_utils.lidar_reconstructor(self.lidar)
        # lidar_points = self.pc2_to_array(self.lidar)

        # logger.debug(f"First 5 X Points: {lidar_points['xyz'][:5]}")
        # logger.debug(f"First 5 Y Points: {lidar_points['xyz'][:5]}")
        # logger.debug(f"First 5 Z Points: {lidar_points['xyz'][:5]}")
        # logger.debug(f"First 5 Intensity: {lidar_points['intensity'][:5]}")

        # self.process_camera()
        # self.process_lidar()
        # self.process_radar()

        # radar_points = self.parse_pointcloud2(self.radar)



    def timer_callback(self, ):
        pass

    
    def process_camera(self, ):
        if self.camera is None:
            logger.error("CAMERA EMPTY.")
            return
        logger.info(f"Camera Time: {self.camera.header.stamp.sec}.{self.camera.header.stamp.nanosec}")
        if self.visualize:
            frame = self.bridge.imgmsg_to_cv2(self.camera, desired_encoding='bgr8')
            frame = cv2.resize(frame, (960, 540)) 
            cv2.imshow('Camera Feed', frame)
            cv2.waitKey(1)
        

    def process_lidar(self, ):
        if self.lidar is None:
            logger.error("LIDAR EMPTY.")
            return
        
        if self.visualize:
            logger.debug("Visualizing Lidar Data")
            lidar_data = self.pointcloud2_to_array_GUI(self.lidar)
            if lidar_data is not None:
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(lidar_data[:, :3])
                o3d.visualization.draw_geometries([pcd])
            else:
                logger.warning("Lidar data is empty")

        logger.info(f"Lidar Time: {self.lidar.header.stamp.sec}.{self.lidar.header.stamp.nanosec}")

    def process_radar(self, ):
        if self.radar is None:
            logger.error("RADAR EMPTY.")
            return
        logger.info(f"Radar Time: {self.radar.header.stamp.sec}.{self.radar.header.stamp.nanosec}")

    def pc2_to_array(self, pointcloud2_msg):
        return rnp.numpify(pointcloud2_msg)
    
    
    def pointcloud2_to_array_GUI(self, pointcloud2_msg):
        try:
            field_names = [field.name for field in pointcloud2_msg.fields]
            dtype_list = []
            seen_fields = set()

            for field in pointcloud2_msg.fields:
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

            dtype = np.dtype(dtype_list)

            point_step = pointcloud2_msg.point_step

            if len(pointcloud2_msg.data) % point_step != 0:
                raise ValueError(f"Buffer size {len(pointcloud2_msg.data)} is not a multiple of point step {point_step}")

            raw_data = np.frombuffer(pointcloud2_msg.data, dtype=np.uint8).reshape((-1, point_step))
            # logger.debug(f"Raw Data Shape: {raw_data.shape}, C-Contiguous: {raw_data.flags['C_CONTIGUOUS']}")

            structured_data = np.zeros((len(raw_data),), dtype=dtype)
            structured_data = np.ascontiguousarray(structured_data)
            # logger.debug(f"Structured Data Shape: {structured_data.shape}, C-Contiguous: {structured_data.flags['C_CONTIGUOUS']}")

            for i, field in enumerate(pointcloud2_msg.fields):
                name = field.name
                offset = field.offset
                field_size = np.dtype(dtype_list[i][1]).itemsize

                field_data = raw_data[:, offset:offset + field_size]

                field_data = field_data.view(dtype_list[i][1]).reshape(-1)
                field_data = np.ascontiguousarray(field_data)

                structured_data[name] = field_data

                # logger.debug(f"Field '{name}' - Shape: {field_data.shape}, Expected: {structured_data[name].shape}, C-Contiguous: {field_data.flags['C_CONTIGUOUS']}")

            required_fields = ['x', 'y', 'z']
            if not all(field in field_names for field in required_fields):
                raise ValueError(f"Expected fields {required_fields} not found in PointCloud2")

            points = np.stack([structured_data['x'], structured_data['y'], structured_data['z']], axis=-1)
            points = np.ascontiguousarray(points)

            # logger.debug(f"Final Points - Shape: {points.shape}, C-Contiguous: {points.flags['C_CONTIGUOUS']}")

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
