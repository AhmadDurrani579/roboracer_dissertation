#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def noisy_controller(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheelbase = float(LaunchConfiguration("wheelbase").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheelbase_error = float(LaunchConfiguration("wheelbase_error").perform(context))

    noisy_controller_node = Node(
        package="roboracer_controller",  # Adjust package name if different
        executable="noisy_controller.py",
        name="noisy_controller",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheelbase": wheelbase + wheelbase_error,
             "use_sim_time": use_sim_time}
        ],
    )

    return [noisy_controller_node]

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation time (Gazebo clock)"
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.05",
        description="Radius of the rear wheels"
    )
    wheelbase_arg = DeclareLaunchArgument(
        "wheelbase",
        default_value="0.325",
        description="Distance between front and rear axles"
    )
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
        description="Error added to wheel radius for noise"
    )
    wheelbase_error_arg = DeclareLaunchArgument(
        "wheelbase_error",
        default_value="0.02",
        description="Error added to wheelbase for noise"
    )

    # Launch the noisy controller
    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription([
        use_sim_time_arg,
        wheel_radius_arg,
        wheelbase_arg,
        wheel_radius_error_arg,
        wheelbase_error_arg,
        noisy_controller_launch,
    ])
