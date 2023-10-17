from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='my_package',
            executable='move_turtlebot3_left',
            output='screen'),
            remappings=[('/SR6T1/', '/SR6T2/')]  
    ])

