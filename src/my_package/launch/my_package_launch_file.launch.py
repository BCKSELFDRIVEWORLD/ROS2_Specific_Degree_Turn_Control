from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='my_package',
            executable='rotate',
            output='screen'),
            remappings=[('/SR6T1/', '/SR6T2/')]  
    ])

