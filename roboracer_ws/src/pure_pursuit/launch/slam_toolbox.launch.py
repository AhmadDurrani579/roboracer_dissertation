from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Путь к YAML-файлу с параметрами
    param_file_path = os.path.join(
        os.getenv('HOME'),
        'roboracer_ws/src/pure_pursuit/config/mapper_params_online_async.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[param_file_path, {'use_sim_time': True}]
        )
    ])
