from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_sync',
            executable='sync2_node.py',
            name='sensor_sync',
            output='screen'
        ),
        Node(
            package='sensor_sync',
            executable='calibration_node.py',
            name='calib',
            output='screen'
        ),
        Node(
            package='sensor_sync',
            executable='pcd_maker_node.py',
            name='pcd_maker',
            output='screen'
        ),
    ])
