import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Environment Setup
    env_vars = [
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value='/opt/ros/humble/lib'),
        SetEnvironmentVariable(name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH', value='/opt/ros/humble/lib'),
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='0'),
    ]

    # 2. Path Configuration
    pkg_dir = get_package_share_directory('roboracer_description')
    world_path = os.path.join(pkg_dir, 'world', 'small_house.world')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'f1tenth_chassis.urdf')

    # 3. Ignition Gazebo Launch
    gazebo = ExecuteProcess(
        cmd=['ign','gazebo','-v','4','--render-engine','ogre2','-r',world_path],
        output='screen',
        additional_env={
          'GZ_SIM_SYSTEM_PLUGIN_PATH':'/opt/ros/humble/lib',
          'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':'/opt/ros/humble/lib'
        }
    )

    # 4. Bridge clock and topics immediately after Gazebo starts
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/car_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/hokuyo/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/car_1/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/model/car_1/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            # RGB Image and Camera Info
            '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Depth Image (corrected)
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            # Point Cloud (optional)
            # '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # '/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

        ],
        remappings=[
            ('/model/car_1/cmd_vel', '/car_1/cmd_vel'),
            ('/model/car_1/imu', '/car_1/imu'),
            # RGB camera remappings
            ('/rgbd_camera/image', '/car_1/rgbd_camera/rgb/image_raw'),
            ('/rgbd_camera/camera_info', '/car_1/rgbd_camera/rgb/camera_info'),
            # Depth camera remapping (updated!)
            ('/rgbd_camera/depth_image', '/car_1/rgbd_camera/depth/image_raw'),
            # If you want to relay camera_info for depth, see previous replies.
            # ('/rgbd_camera/points', '/car_1/rgbd_camera/points'),
            # ('/rgbd_camera/points', '/car_1/rgbd_camera/points'),

        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # 5. Robot Spawning and State Publishers (delayed until clock is active)
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', '/robot_description', '-name', 'car_1', '-x', '-5.0', '-y', '0.0', '-z', '0.0'],
        parameters=[{'use_sim_time': True}], output='screen'
    )
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path, 'r').read(), 'use_sim_time': True}],
        output='screen'
    )
    joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        parameters=[{'use_sim_time': True, 'source_list': ['/model/car_1/joint_state'], 'rate': 100}],
        output='screen'
    )

    # 6. Static TFs
    tf_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    # 7. Controller Nodes (starting without delay)
    ackermann_controller = Node(
        package='roboracer_controller', executable='ackerman_controller.py',
        parameters=[{'use_sim_time': True}], output='screen', name='ackermann_controller_node' # Give it a name
    )
    ackermann_to_twist = Node(
        package='roboracer_controller', executable='ackermann_to_twist.py', name='ackermann_to_twist',
        parameters=[{'wheelbase': 0.325}], output='screen'
    )
    
    camera_info_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=[
            '/car_1/rgbd_camera/rgb/camera_info',
            '/car_1/rgbd_camera/depth/camera_info'
        ],
        name='depth_camera_info_relay',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[bridge]
        )
    ))
    
    
    # Delay sim‑time nodes (except ackermann_controller) by 1 s
    ld.add_action(TimerAction(
        period=1.0,
        actions=[
            spawn_robot,
            robot_state_publisher,
            joint_state_publisher,
            ackermann_to_twist,
        ]
    ))
    
    # Start ackermann_controller immediately
    ld.add_action(ackermann_controller)
    ld.add_action(camera_info_relay)

    # Register the event handler to start tf_echo after ackermann_controller starts
    # ld.add_action(RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=ackermann_controller,
    #         on_start=[tf_echo_node]
    #     )
    # ))
    return ld