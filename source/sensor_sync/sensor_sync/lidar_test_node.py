#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_sync.msg import SyncedSensors
from loguru import logger
# from sensor_msgs.msg import PointCloud2
# from ouster.sdk import client
# import numpy as np
# import matplotlib.pyplot as plt
# import struct
# from ouster_sensor_msgs.msg import PacketMsg
# from ouster_sensor_msgs.libouster_sensor_msgs__rosidl_generator_py import point_cloud_from_packets
from mpl_toolkits.mplot3d import Axes3D
from ouster.sdk.client import LidarScan, ChanField, XYZLut
# np.set_printoptions(threshold=np.inf, linewidth=200, precision=3, suppress=True)

# class LidarTest(Node):
#     def __init__(self):
#         super().__init__('lidar_test_node')
#         self.create_subscription(PointCloud2,
#                                   '/ouster/points',
#                                     self.points_callback, 10)
        
#         self.create_subscription(PointCloud2,
#                                   '/ouster/metadata',
#                                     self.meta_callback, 10)
#         self.sensor_info = None
#         plt.ion()  
#         self.fig = plt.figure()
#         self.ax = self.fig.add_subplot(111, projection='3d')
#         self.sc = self.ax.scatter([], [], [], s=1, c=[], cmap='viridis')
#         self.ax.set_xlabel('X (meters)')
#         self.ax.set_ylabel('Y (meters)')
#         self.ax.set_zlabel('Z (meters)')
#         self.ax.set_title('Live Ouster PointCloud Visualization - 3D')
#         logger.info(f'Lidar tester initialzied.')

        
        
        
#     def points_callback(self, msg):
#         if self.sensor_info is None:
#             self.sensor_info = client.SensorInfo()
#             # self.lut = xyz_lut(self.sensor_info)  # Removed as xyz_lut is not available

#         point_data = self.pointcloud2_to_numpy(msg)
#         if point_data is not None:
#             self.visualize_points(point_data)

#     def meta_callback(self, msg):
#         logger.debug('Metadata callback...')
#         logger.info(f'METADATA:{msg}')

#     # def pointcloud2_to_numpy(self, msg):
#     #     data = np.frombuffer(msg.data, dtype=np.float32)
#     #     # logger.info(f'the data is :{data}')
#     #     data = data.reshape(-1, 4)  # Assuming x, y, z, intensity
#     #     return data[:, :3]
#     # 
#     def pointcloud2_to_numpy(self, msg):
#         import sensor_msgs_py.point_cloud2 as pc2
#         points = []
#         for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
#             points.append([point[0], point[1], point[2], point[3]])

#         return np.array(points)  

#     def visualize_points(self, points):
#         self.ax.cla()
#         self.ax.set_xlabel('X (meters)')
#         self.ax.set_ylabel('Y (meters)')
#         self.ax.set_zlabel('Z (meters)')
#         self.ax.set_title('Live Ouster PointCloud Visualization - 3D')
#         self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, c=points[:, 2], cmap='viridis')
#         plt.pause(0.01)


# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarTest()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud2
# import numpy as np
# import struct


# def pointcloud2_to_array(pointcloud):
#     dtype_list = [
#         ('x', np.float32), ('y', np.float32), ('z', np.float32),
#         ('intensity', np.uint16), ('t', np.uint32), 
#         ('reflectivity', np.uint8), ('ring', np.uint8), ('ambient', np.uint16)
#     ]
    
#     data = np.frombuffer(pointcloud.data, dtype=np.uint8)

#     num_points = pointcloud.width * pointcloud.height
#     point_step = pointcloud.point_step

#     data = data.reshape((-1, point_step))
    
#     points = np.zeros((num_points,), dtype=dtype_list)
#     for i, (name, dtype) in enumerate(dtype_list):
#         offset = pointcloud.fields[i].offset
#         length = np.dtype(dtype).itemsize
#         points[name] = data[:, offset:offset + length].view(dtype).flatten()

#     return points

# class LidarSubscriber(Node):
#     def __init__(self):
#         super().__init__('lidar_listener')
#         self.subscription = self.create_subscription(
#             PointCloud2,
#             '/ouster/points',
#             self.listener_callback,
#             10
#         )
#         logger.info(f'Lidar tester initialzied.')

    # def manual_lidar_reconstructor(self, msg):
    #         try:
    #             point_step = msg.point_step
    #             data = msg.data
    #             width = msg.width
    #             height = msg.height
    #             nan_count = 0
    #             total_points = width * height

    #             valid_count = 0
                
    #             reconstructed_points = []
    #             for i in range(total_points):
    #                 start_index = i * point_step
    #                 end_index = start_index + point_step
    #                 point_data = data[start_index:end_index]
    #                 try:
    #                     x = struct.unpack_from('f', point_data, 0)[0]
    #                     y = struct.unpack_from('f', point_data, 4)[0]
    #                     z = struct.unpack_from('f', point_data, 8)[0]
    #                     range_val = struct.unpack_from('f', point_data, offset=32)[0]
    #                     intensity = struct.unpack_from('f', point_data, offset=16)[0]
    #                     ring = struct.unpack_from('H', point_data, offset=26)[0]
    #                     if any([x != x, y != y, z != z]):  # Check for nan using `!=` comparison
    #                         nan_count += 1
    #                     else:
    #                         valid_count += 1
    #                     # logger.info(f"Point {i}: X: {x}, Y: {y}, Z: {z}, Intensity: {intensity}, Range: {range_val}")
    #                     # if not np.isnan(range_val) and range_val > 0.001:
    #                         # logger.error(f"Range: {range_val}, Intensity: {intensity}, Ring: {ring}")
    #                         # exit()
    #                     if not np.isnan(range_val) and range_val > 0:
    #                         #  (assumes 64-channel LiDAR)
    #                         vertical_angle = np.deg2rad(-30 + 60 * (ring / 63))
    #                         #  (assumes 360° coverage)
    #                         horizontal_angle = np.deg2rad((i % width) * 360 / width)

    #                         # Convert to Cartesian coordinates
    #                         x = range_val * np.cos(vertical_angle) * np.cos(horizontal_angle)
    #                         y = range_val * np.cos(vertical_angle) * np.sin(horizontal_angle)
    #                         z = range_val * np.sin(vertical_angle)

    #                         reconstructed_points.append([x, y, z, intensity])
    #                     else:
    #                         # logger.warning(f'range_val: {range_val}')
    #                         reconstructed_points.append([np.nan, np.nan, np.nan, intensity])

    #                 except struct.error as e:
    #                     logger.error(f"Data unpacking error at point {i}: {e}")
    #             logger.info(f"Total Points: {total_points}")
    #             logger.info(f"Valid Points: {valid_count}")
    #             logger.info(f"NaN Points: {nan_count}")
    #             logger.info(f"Percentage of Valid Points: {(valid_count / total_points) * 100:.2f}%")
    #             logger.info(f"Percentage of NaN Points: {(nan_count / total_points) * 100:.2f}%\n")

    #             # Convert to numpy array
    #             points_array = np.array(reconstructed_points, dtype=np.float32)
    #             logger.info(f"Reconstructed PointCloud2 with shape: {points_array.shape}")
    #             if points_array.shape[0] > 0:
    #                 self.update_plot(points_array)
    #             else:
    #                 logger.warning("No valid points to visualize.")

    #         except Exception as e:
    #             logger.error(f"Error in lidar_callback: {e}")
    #         return points_array


#     def listener_callback(self, msg):
#         points_array = pointcloud2_to_array(msg)
#         mask = ~np.isnan(points_array['x'])
#         valid_points = points_array[mask]

#         logger.info(f"Received {len(points_array)} points")
#         logger.info(points_array)
#         logger.info(f"Number of valid points: {len(valid_points)}")
#         logger.info(valid_points)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarSubscriber()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# class LidarUtils(Node):
#     def __init__(self):
        
#         self.init_plot()


#     def init_plot(self):
#         self.fig = plt.figure()
#         self.ax = self.fig.add_subplot(111, projection='3d')
#         plt.ion()
#         plt.show()
#         logger.info("Lidar Visualizer Node Initialized")

#         # Metadata and LUT initialization
#         self.metadata = None
#         self.xyz_lut = None


#     def update_plot(self, points):
#         x_vals = points[:, 0]
#         y_vals = points[:, 1]
#         z_vals = points[:, 2]
#         intensity = points[:, 3]

#         self.ax.cla()
#         self.scatter = self.ax.scatter(x_vals, y_vals, z_vals, c=intensity, cmap='viridis', s=1)
#         self.ax.set_title("Live 3D PointCloud Visualization")
#         self.ax.set_xlabel("X")
#         self.ax.set_ylabel("Y")
#         self.ax.set_zlabel("Z")
#         plt.draw()
#         plt.pause(0.001)
    # def pointcloud2_to_array_GUI(self, pointcloud2_msg):
    #         try:
    #             field_names = [field.name for field in pointcloud2_msg.fields]
    #             dtype_list = []
    #             seen_fields = set()

    #             for field in pointcloud2_msg.fields:
    #                 if field.name in seen_fields:
    #                     logger.warning(f"Duplicate field '{field.name}' encountered. Skipping.")
    #                     continue
    #                 seen_fields.add(field.name)

    #                 if field.datatype == 7:  # FLOAT32
    #                     dtype_list.append((field.name, np.float32))
    #                 elif field.datatype == 8:  # FLOAT64
    #                     dtype_list.append((field.name, np.float64))
    #                 elif field.datatype == 6:  # INT32
    #                     dtype_list.append((field.name, np.int32))
    #                 elif field.datatype == 5:  # INT16
    #                     dtype_list.append((field.name, np.int16))
    #                 elif field.datatype == 4:  # INT8
    #                     dtype_list.append((field.name, np.int8))
    #                 elif field.datatype == 2:  # UINT16
    #                     dtype_list.append((field.name, np.uint16))
    #                 elif field.datatype == 1:  # UINT8
    #                     dtype_list.append((field.name, np.uint8))
    #                 else:
    #                     logger.warning(f"Unsupported datatype: {field.datatype}")
    #                     continue

    #             dtype = np.dtype(dtype_list)

    #             point_step = pointcloud2_msg.point_step

    #             if len(pointcloud2_msg.data) % point_step != 0:
    #                 raise ValueError(f"Buffer size {len(pointcloud2_msg.data)} is not a multiple of point step {point_step}")

    #             raw_data = np.frombuffer(pointcloud2_msg.data, dtype=np.uint8).reshape((-1, point_step))
    #             # logger.debug(f"Raw Data Shape: {raw_data.shape}, C-Contiguous: {raw_data.flags['C_CONTIGUOUS']}")

    #             structured_data = np.zeros((len(raw_data),), dtype=dtype)
    #             structured_data = np.ascontiguousarray(structured_data)
    #             # logger.debug(f"Structured Data Shape: {structured_data.shape}, C-Contiguous: {structured_data.flags['C_CONTIGUOUS']}")

    #             for i, field in enumerate(pointcloud2_msg.fields):
    #                 name = field.name
    #                 offset = field.offset
    #                 field_size = np.dtype(dtype_list[i][1]).itemsize

    #                 field_data = raw_data[:, offset:offset + field_size]

    #                 field_data = field_data.view(dtype_list[i][1]).reshape(-1)
    #                 field_data = np.ascontiguousarray(field_data)

    #                 structured_data[name] = field_data

    #                 # logger.debug(f"Field '{name}' - Shape: {field_data.shape}, Expected: {structured_data[name].shape}, C-Contiguous: {field_data.flags['C_CONTIGUOUS']}")

    #             required_fields = ['x', 'y', 'z']
    #             if not all(field in field_names for field in required_fields):
    #                 raise ValueError(f"Expected fields {required_fields} not found in PointCloud2")

    #             points = np.stack([structured_data['x'], structured_data['y'], structured_data['z']], axis=-1)
    #             points = np.ascontiguousarray(points)

    #             # logger.debug(f"Final Points - Shape: {points.shape}, C-Contiguous: {points.flags['C_CONTIGUOUS']}")

    #             return points

    #         except Exception as e:
    #             logger.error(f"Error converting PointCloud2: {e}")
    #             return None
        



#     def convert_to_numpy(self, msg):
#         try:
#             scan = LidarScan(msg.height, msg.width)
#             scan_field = scan.field(ChanField.RANGE)
#             xyz_lut = XYZLut(self.metadata)
#             xyz = xyz_lut(scan_field)

#             intensity = scan.field(ChanField.REFLECTIVITY)

#             x = xyz[:, :, 0].flatten()
#             y = xyz[:, :, 1].flatten()
#             z = xyz[:, :, 2].flatten()
#             intensity = intensity.flatten()

#             points = np.stack((x, y, z, intensity), axis=-1)
#             return points

#         except Exception as e:
#             logger.error(f"Error in convert_to_numpy: {e}")
#             return np.array([])


#     def lidar_reconstructor(self, packet_msg):
#         try:
#             points = self.convert_to_numpy(packet_msg)
#             if points.size > 0:
#                 self.update_plot(points)
#         except Exception as e:
#             logger.error(f"Error in lidar_callback: {e}")


#     def manual_lidar_reconstructor(self, msg):
#         try:
#             point_step = msg.point_step
#             data = msg.data
#             width = msg.width
#             height = msg.height
#             nan_count = 0
#             total_points = width * height

#             valid_count = 0
            
#             reconstructed_points = []
#             for i in range(total_points):
#                 start_index = i * point_step
#                 end_index = start_index + point_step
#                 point_data = data[start_index:end_index]
#                 try:
#                     x = struct.unpack_from('f', point_data, 0)[0]
#                     y = struct.unpack_from('f', point_data, 4)[0]
#                     z = struct.unpack_from('f', point_data, 8)[0]
#                     range_val = struct.unpack_from('f', point_data, offset=32)[0]
#                     intensity = struct.unpack_from('f', point_data, offset=16)[0]
#                     ring = struct.unpack_from('H', point_data, offset=26)[0]
#                     if any([x != x, y != y, z != z]):  # Check for nan using `!=` comparison
#                         nan_count += 1
#                     else:
#                         valid_count += 1
#                     # logger.info(f"Point {i}: X: {x}, Y: {y}, Z: {z}, Intensity: {intensity}, Range: {range_val}")
#                     # if not np.isnan(range_val) and range_val > 0.001:
#                         # logger.error(f"Range: {range_val}, Intensity: {intensity}, Ring: {ring}")
#                         # exit()
#                     if not np.isnan(range_val) and range_val > 0:
#                         #  (assumes 64-channel LiDAR)
#                         vertical_angle = np.deg2rad(-30 + 60 * (ring / 63))
#                         #  (assumes 360° coverage)
#                         horizontal_angle = np.deg2rad((i % width) * 360 / width)

#                         # Convert to Cartesian coordinates
#                         x = range_val * np.cos(vertical_angle) * np.cos(horizontal_angle)
#                         y = range_val * np.cos(vertical_angle) * np.sin(horizontal_angle)
#                         z = range_val * np.sin(vertical_angle)

#                         reconstructed_points.append([x, y, z, intensity])
#                     else:
#                         # logger.warning(f'range_val: {range_val}')
#                         reconstructed_points.append([np.nan, np.nan, np.nan, intensity])

#                 except struct.error as e:
#                     logger.error(f"Data unpacking error at point {i}: {e}")
#             logger.info(f"Total Points: {total_points}")
#             logger.info(f"Valid Points: {valid_count}")
#             logger.info(f"NaN Points: {nan_count}")
#             logger.info(f"Percentage of Valid Points: {(valid_count / total_points) * 100:.2f}%")
#             logger.info(f"Percentage of NaN Points: {(nan_count / total_points) * 100:.2f}%\n")

#             # Convert to numpy array
#             points_array = np.array(reconstructed_points, dtype=np.float32)
#             logger.info(f"Reconstructed PointCloud2 with shape: {points_array.shape}")
#             if points_array.shape[0] > 0:
#                 self.update_plot(points_array)
#             else:
#                 logger.warning("No valid points to visualize.")

#         except Exception as e:
#             logger.error(f"Error in lidar_callback: {e}")
#         return points_array

#     def visualize_points(self, points):
#         """ Live 3D visualization using Matplotlib. """
#         try:
#             # Extract X, Y, Z, Intensity
#             x_vals = points[:, 0]
#             y_vals = points[:, 1]
#             z_vals = points[:, 2]
#             intensity = points[:, 3]

#             # Plot with Matplotlib
#             fig = plt.figure(figsize=(10, 7))
#             ax = fig.add_subplot(111, projection='3d')

#             # Scatter plot with intensity-based coloring
#             scatter = ax.scatter(x_vals, y_vals, z_vals, c=intensity, cmap='viridis', s=1)
#             plt.colorbar(scatter, ax=ax, label='Intensity')

#             ax.set_title('3D PointCloud Visualization')
#             ax.set_xlabel('X')
#             ax.set_ylabel('Y')
#             ax.set_zlabel('Z')

#             plt.show()

#         except Exception as e:
#             logger.error(f"Visualization Error: {e}")   


#     def lidar_test(self, msg):
#         try:
#             point_step = msg.point_step
#             data = msg.data

#             for i in range(3):
#                 start_index = i * point_step
#                 end_index = start_index + point_step
#                 point_data = data[start_index:end_index]

#                 try:
#                     x, y, z = struct.unpack_from('fff', point_data, offset=0)
#                     intensity = struct.unpack_from('f', point_data, offset=16)[0]
#                     timestamp = struct.unpack_from('I', point_data, offset=20)[0]
#                     reflectivity = struct.unpack_from('H', point_data, offset=24)[0]
#                     ring = struct.unpack_from('H', point_data, offset=26)[0]
#                     ambient = struct.unpack_from('H', point_data, offset=28)[0]
#                     point_range = struct.unpack_from('f', point_data, offset=32)[0]

#                     logger.info(f"Point {i}: XYZ: ({x}, {y}, {z}), Intensity: {intensity}, "
#                                 f"Timestamp: {timestamp}, Reflectivity: {reflectivity}, "
#                                 f"Ring: {ring}, Ambient: {ambient}, Range: {point_range}")

#                 except struct.error as e:
#                     logger.error(f"Data unpacking error for point {i}: {e}")

#         except Exception as e:
#             logger.error(f"Error during manual extraction: {e}")




from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import open3d as o3d
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import sensor_msgs_py.numpy_compat as nc
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np


class MySubscriber(Node):
    def __init__(self):
        super().__init__('lidar_listener')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.listener_callback,
            10
        )
        logger.info(f'Lidar tester initialzied.')
        

    def listener_callback(self, msg: PointCloud2):
        logger.debug('Received PointCloud2 message')
        self.point_cloud_data = msg
        points = self.convert_point_cloud_msg_to_numpy(self.point_cloud_data)
        logger.info(f'points are: {points}')
        
        
    def convert_point_cloud_msg_to_numpy(self, data: PointCloud2):
        if self.point_cloud_data is not None:
            gen = pc2.read_points(data, field_names=("x", "y", "z", "intensity"), skip_nans=False)
            points_list = list(gen)

            if len(points_list) == 0:
                return np.array([])

            points_numpy = np.array([
                [p[0], p[1], p[2], p[3]] for p in points_list
            ], dtype=np.float32)

            mask = ~np.isnan(points_numpy[:, 0]) & ~np.isnan(points_numpy[:, 1]) & ~np.isnan(points_numpy[:, 2])
            points_numpy = points_numpy[mask]

            return points_numpy
        

def main():
    
    rclpy.init(args=None)
    subscriber = MySubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

