from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_terminals = LaunchConfiguration('use_terminals')
    setup = 'source /opt/ros/humble/setup.bash && source ~/rosws/install/setup.bash &&'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_terminals',
            default_value='false',
            description='Launch each node in a separate terminal'
        ),

        ExecuteProcess(
            condition=IfCondition(use_terminals),
            cmd=['gnome-terminal', '--', 'bash', '-c', f'{setup} ros2 run sensor_sync sync2_node.py'],
            name='sync_terminal'
        ),
        Node(
            condition=UnlessCondition(use_terminals),
            package='sensor_sync',
            executable='sync2_node.py',
            name='sync_node',
            output='screen'
        ),

        ExecuteProcess(
            condition=IfCondition(use_terminals),
            cmd=['gnome-terminal', '--', 'bash', '-c', f'{setup} ros2 run sensor_sync calibration_node.py'],
            name='calib_terminal'
        ),
        Node(
            condition=UnlessCondition(use_terminals),
            package='sensor_sync',
            executable='calibration_node.py',
            name='calibration_node',
            output='screen'
        ),

        ExecuteProcess(
            condition=IfCondition(use_terminals),
            cmd=['gnome-terminal', '--', 'bash', '-c', f'{setup} ros2 run sensor_sync pcd_maker_node.py'],
            name='pcd_terminal'
        ),
        Node(
            condition=UnlessCondition(use_terminals),
            package='sensor_sync',
            executable='pcd_maker_node.py',
            name='pcd_maker_node',
            output='screen'
        )
    ])
