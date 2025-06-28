from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    pkg_share = get_package_share_directory("roboracer_description")
    sensors_config = os.path.join(pkg_share, 'config', 'sensors.yaml')

    joy_teleop_config = os.path.join(
        get_package_share_directory('roboracer_description'),
        'config',
        'joy_teleop.yaml'
    )
    
    vesc_config = os.path.join(
        get_package_share_directory('roboracer_description'),
        'config',
        'vesc.yaml'
    )

    mux_config = os.path.join(
        get_package_share_directory('roboracer_description'),
        'config',
        'mux.yaml'
    )
    
    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')
    
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')

    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')

    ld = LaunchDescription([joy_la, vesc_la, sensors_la, mux_la])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[sensors_config]
    )
    
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )

    camera_Node = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            output='screen',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_imu': True }]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)

    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(camera_Node)
    ld.add_action(static_tf_node)

    return ld