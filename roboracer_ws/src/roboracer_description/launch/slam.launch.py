import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")
    map_yaml_path = LaunchConfiguration("map_yaml_path")

    ros_distro = os.environ["ROS_DISTRO"]
    lifecycle_nodes = ["map_saver_server"]
    if ros_distro != "humble":
        lifecycle_nodes.append("slam_toolbox")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("roboracer_description"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to slam yaml file to load"
    )

    map_yaml_path_arg = DeclareLaunchArgument(
        "map_yaml_path",
        default_value=os.path.join(
            get_package_share_directory("roboracer_description"),
            "maps",
            "levine.yaml"
        ),
        description="Full path to map yaml file to load"
    )

    nav2_map_saver = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_saver_server",
        namespace='',
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65},
            {"yaml_filename": map_yaml_path},
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ('/scan', '/your_robot/laser/scan'),
            ('/odom', '/your_robot/odom')
        ]
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )



    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        map_yaml_path_arg,
        nav2_map_saver,
        slam_toolbox,
        nav2_lifecycle_manager,
    ])
