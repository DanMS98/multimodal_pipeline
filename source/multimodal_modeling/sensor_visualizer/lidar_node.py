import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class LidarViewer(Node):
    def __init__(self):
        super().__init__('lidar_viewer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.listener_callback,
            10)
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='LiDAR Viewer', width=800, height=600)
        self.pcd = o3d.geometry.PointCloud()
        self.first_frame = True

    def listener_callback(self, msg):
        cloud_points = np.array([
            [p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])
        if len(cloud_points) == 0:
            return

        self.pcd.points = o3d.utility.Vector3dVector(cloud_points)

        if self.first_frame:
            self.vis.add_geometry(self.pcd)
            self.first_frame = False
        else:
            self.vis.update_geometry(self.pcd)

        self.vis.poll_events()
        self.vis.update_renderer()

    def destroy_node(self):
        self.vis.destroy_window()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
