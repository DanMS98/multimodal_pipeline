import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import message_filters
import math
from visualization_msgs.msg import Marker, MarkerArray


class RadarVisualizer(Node):
    def __init__(self):
        super().__init__('radar_visualizer')
        self.sub_radar = self.create_subscription(
            PointCloud2,
            '/radar_data/point_cloud',
            self.radar_callback,
            10
        )
        self.get_logger().info('Radar Visualizer Node Started')

    def radar_callback(self, msg: PointCloud2):
        marker_array = MarkerArray()
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        for i, (x, y, z) in enumerate(points):
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue
            if x == 0.0 and y == 0.0 and z == 0.0:
                continue

            marker = Marker()
            marker.header = msg.header
            marker.ns = "radar"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.1
            marker.lifetime.sec = 1 
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} radar points')

def main(args=None):
    rclpy.init(args=args)
    node = RadarVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
