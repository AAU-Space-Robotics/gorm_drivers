
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    ld = LaunchDescription()

    # Delayed zed_tracking launch
    zed_tracking = TimerAction(
        period=20.0,  # Delay in seconds
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('gorm_sensors'), '/launch/zed_camera.launch.py'
                ]),
                launch_arguments={
                    'camera_model': 'zed2i',
                    'serial_number': '37915676',
                    'camera_id': '0',
                    'node_name': 'zed_tracking',
                    'grab_resolution': 'HD1080',
                    'gnss_fusion_enabled': 'false',
                    'namespace': 'zed_tracking',
                    'initial_base_pose': '[0.28, 0.0, 0.225, 0.0, 0.0, 0.0]',
                    'pos_tracking': 'true',
                    'publish_tf': 'true',
                    'publish_map_tf': 'true',
                }.items()
            )
        ]
    )

    zed_front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gorm_sensors'), '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'serial_number': '35803121',
            'camera_id': '1',
            'node_name': 'zed_front',
            'grab_resolution': 'HD1080', # 'VGA',  # The native camera grab resolution. 'HD2K', 'HD1080', 'HD720', 'VGA', 'AUTO'
            'pos_tracking': 'false',  # Enable positional tracking
            'publish_tf': 'true',  # Publish TF for the camera
            'publish_imu_tf': 'true'
            'publish_map_tf': 'false',  # Publish map TF for the camera
            'namespace': 'zed_front',  # Namespace for the camera node
        }.items()
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_zed_base',
        arguments=['-0.28', '0.0', '-0.225', '0.0', '0.0', '0.0', 'zed_camera_link', 'base_link'],
    )

    camera_pose = Node(
        package='gorm_sensors',
        executable='pose_transform',
        name='camera_pose',
    )

    ld.add_action(static_tf_node)
    ld.add_action(zed_tracking)
    ld.add_action(zed_front)

    return ld
