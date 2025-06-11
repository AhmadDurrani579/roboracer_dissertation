from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument # Import this
from launch.substitutions import LaunchConfiguration # Import this

def generate_launch_description():
    # 1. Declare a launch argument for use_sim_time
    #    This allows you to easily control it when launching,
    #    e.g., ros2 launch my_package my_launch.py use_sim_time:=false
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true', # <--- Set the default value to true for simulation
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        use_sim_time_arg, # <--- Add the declared argument to the launch description

        # base_link → laser (critical fix: adjust x/y/z/yaw to match your robot's URDF)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0.0', '0.3', '0.0', '0.0', '0.0', 'car_1_base_link', 'car_1_laser'],
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}] # <--- Add this parameter
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0.40',  # x (m)
                '0.00',  # y
                '0.15',  # z
                '0.00',  # roll
                '0.00',  # pitch
                '0.00',  # yaw
                'car_1_base_link',     # parent frame
                'car_1_camera_link'    # child frame
            ],
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}] # <--- Add this parameter
        ),

        Node(
            package='roboracer_py',
            executable='gap_follow',
            name='gap_follow',
            parameters=[
                # {'max_speed': 1.0},
                # {'kp': 0.2},
                {'use_sim_time': LaunchConfiguration('use_sim_time')} # <--- Add this parameter
            ],
            # arguments=['--ros-args', '--log-level', 'debug'] # Keep this for now
        ),
    ])