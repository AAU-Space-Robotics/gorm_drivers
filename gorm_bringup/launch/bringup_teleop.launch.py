
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    urdf_folder = os.path.join(get_package_share_directory("gorm_bringup"), "urdf")
    urdf_path = os.path.join(urdf_folder, "gorm_simple.urdf.xml")
    with open(urdf_path, 'r') as infp:
        urdf = infp.read()

    # URDF publisher
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False, 'robot_description': urdf}],
        arguments=[urdf]
    )

    # cmd_vel to motor commands converter
    ackermann_nav2 = Node(
        package='gorm_base_control',
        executable='ackermann_nav2',
        name='ackermann_nav2',
        output='screen'
    )

    motor_driver_node = Node(
        package='gorm_base_control',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='screen'
    )

    # Include control switching (twist_mux) system
    control_switch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gorm_bringup'), 'launch', 'control_switch.launch.py')
        ])
    )

    # Include teleop system  
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gorm_teleop'), 'launch', 'teleop.launch.py')
        ])
    )

    # Web interface (includes rosbridge and web server)
    web_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gorm_web_interface'), 'launch', 'web_interface.launch.py')
        ])
    )

    ld.add_action(state_publisher)
    ld.add_action(control_switch)
    ld.add_action(teleop_launch) 
    ld.add_action(ackermann_nav2)
    ld.add_action(motor_driver_node)
    ld.add_action(web_interface)

    return ld
