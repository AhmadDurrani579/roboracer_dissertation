from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # base_link → laser (critical fix: adjust x/y/z/yaw to match your robot's URDF)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.1', '0.0', '0.3', '0.0', '0.0', '0.0', 'car_1_base_link', 'car_1_laser'],
            output='screen'
        ),
        # Wall-following node (ensure it uses the same frame names)
        Node(
            package='roboracer_py',
            executable='gap_follow',
            name='gap_follow',
            parameters=[
                # {'max_speed': 1.0},
                # {'kp': 0.2}
            ],
            # arguments=['--ros-args', '--log-level', 'debug'] # Keep this for now
        ),
    ])