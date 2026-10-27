"""Quest commissioning with NO robot output: TCP endpoint only. Pair with quest_teleop.monitor."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('endpoint_ip', default_value='0.0.0.0'),
        Node(package='ros_tcp_endpoint', executable='default_server_endpoint', output='screen',
             parameters=[{'ROS_IP': LaunchConfiguration('endpoint_ip'), 'ROS_TCP_PORT': 10000}]),
    ])
