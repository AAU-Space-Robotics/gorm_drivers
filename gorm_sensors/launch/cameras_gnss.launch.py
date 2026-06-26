from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    ld = LaunchDescription()

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
                    'grab_resolution': 'HD720',
                    'gnss_fusion_enabled': 'true',
                    'namespace': 'zed_tracking',
                    'initial_base_pose': '[0.28, 0.0, 0.225, 0.0, 0.0, 0.0]',
                    'pos_tracking': 'true',
                    'publish_tf': 'true',
                    'publish_imu_tf': 'true',
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
            'gnss_fusion_enabled': 'true',
            'pos_tracking': 'false',  # Enable positional tracking
            'publish_tf': 'false',  # Publish TF for the camera
            'publish_imu_tf': 'true',
            'publish_map_tf': 'false',  # Publish map TF for the camera
            'namespace': 'zed_front',  # Namespace for the camera node
        }.items()
    )
    static_tf_node_f = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_zed_base_f',
        arguments=['0.280', '-0.264', '0.472', '0.1222', '0.2618', '0.0', 'chassis_link', 'zed_camera_link'],
        # arguments=['0.0', '0.0', '0.0', '0.0', '0.2618', '0.0', 'base_link', 'zed_camera_link'],
        # Arguments: x y z yaw pitch roll parent_frame child_frame
        # Note: yaw, pitch, roll are in radians!
    )

    static_tf_node_b = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_zed_base_b',
        arguments=['-0.282', '0.264', '0.340', '3.2637', '0.2618', '0.0', 'chassis_link', 'zed_camera_link_b'],
        # Arguments: x y z yaw pitch roll parent_frame child_frame
        # Note: yaw, pitch, roll are in radians!
    )

    static_tf_node_c = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_zed_base_c',
        arguments=['0.0', '0.0', '0.302', '0.0', '0.0', '0.0', 'base_link', 'chassis_link'],
        # Arguments: x y z yaw pitch roll parent_frame child_frame
        # Note: yaw, pitch, roll are in radians!
    )

    camera_pose = Node(
        package='gorm_sensors',
        executable='pose_transform',
        name='camera_pose',
    )

    params_file = os.path.join(
        get_package_share_directory('gorm_sensors'),
        'config',
        'zed_f9p.yaml'
    )

    ublox_gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[params_file],
    )

    ld.add_action(static_tf_node_f)
    ld.add_action(static_tf_node_b)
    ld.add_action(static_tf_node_c)
    ld.add_action(zed_tracking)
    ld.add_action(zed_front)
    ld.add_action(ublox_gps_node)
    return ld