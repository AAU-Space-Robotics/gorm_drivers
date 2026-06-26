from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gorm_base_control',
            executable='ackermann_nav2',
            name='ackermann_nav2'
        ),
    ])
