from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='rotate',
            name='move_left_turtlebot3.py',
            namespace='SR6T2',
            output='screen',)
    ])


