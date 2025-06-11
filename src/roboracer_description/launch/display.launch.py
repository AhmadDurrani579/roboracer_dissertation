from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("roboracer_description")
    rviz_config_path = os.path.join(pkg_share, "rviz", "display.rviz")

    return LaunchDescription([
        # Static TF Publisher (map to odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # RViz2 (delayed start to ensure TF is ready)
        TimerAction(
            period=2.0,  # Shorter delay since we have fewer dependencies
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_path],
                    parameters=[{
                        'use_sim_time': False  # Set to True if using simulation
                    }],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static')
                    ],
                    output='screen'
                )
            ]
        )
    ])


# from launch import LaunchDescription
# from launch_ros.actions import Node, LifecycleNode
# from launch.actions import TimerAction
# import os
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     pkg_share = get_package_share_directory("roboracer_description")
#     rviz_config_path = os.path.join(pkg_share, "rviz", "display.rviz")
#     map_yaml_path = os.path.join(pkg_share, "maps", "levine.yaml")

#     return LaunchDescription([
#         # 1. Map Server (lifecycle version with namespace)
#         LifecycleNode(
#             package='nav2_map_server',
#             executable='map_server',
#             name='map_server',
#             namespace='',  # Explicit empty namespace
#             output='screen',
#             parameters=[{'yaml_filename': map_yaml_path}]
#         ),

#         # 2. Lifecycle Manager
#         Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager',
#             output='screen',
#             parameters=[{
#                 'autostart': True,
#                 'node_names': ['map_server']
#             }]
#         ),

#         # 3. Static TF (new ROS 2 syntax)
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
#             output='screen'
#         ),

#         # 4. RViz (delayed start)
#         TimerAction(
#             period=5.0,
#             actions=[
#                 Node(
#                     package='rviz2',
#                     executable='rviz2',
#                     name='rviz2',
#                     arguments=['-d', rviz_config_path],
#                     parameters=[{
#                         'use_sim_time': True,
#                         'qos_overrides./map.subscription.durability': 'transient_local',
#                         'qos_overrides./map.subscription.reliability': 'reliable'
#                     }],
#                     remappings=[
#                         ('/tf', 'tf'),
#                         ('/tf_static', 'tf_static')  # Critical for TF visualization
#                     ]
#                 )
#             ]
#         )
#     ])


