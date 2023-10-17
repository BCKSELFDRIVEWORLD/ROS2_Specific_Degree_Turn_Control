from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
            DeclareLaunchArgument(
        'namespace',
        default_value='SR6T2',  
        description='rotate',
        
        Node(
            package='my_package',
            executable='rotate',
            output='screen'),
    ])

