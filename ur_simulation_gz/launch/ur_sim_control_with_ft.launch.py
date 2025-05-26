#!/usr/bin/env python3
"""Launch file for UR simulation with Force/Torque sensor enabled."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Path to the custom world file with FT sensor system
    world_file = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gz"), "worlds", "ur_with_ft_sensor.sdf"]
    )
    
    # Include the main UR simulation launch file with our custom world
    ur_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_simulation_gz"), "/launch/ur_sim_control.launch.py"
        ]),
        launch_arguments={
            "world_file": world_file,
            "ur_type": "ur5e",  # You can change this to your robot type
            "gazebo_gui": "true",
            "launch_rviz": "true",
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur7e", "ur10", "ur10e", "ur12e", "ur16e", "ur15", "ur20", "ur30"],
        ),
        ur_sim_launch
    ])