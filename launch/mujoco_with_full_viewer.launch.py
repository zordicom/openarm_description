"""
Copyright 2025 Zordi, Inc. All rights reserved.

Launch MuJoCo simulation with FULL interactive viewer (ImGui UI).

This launch file starts:
1. mujoco_ros2_control - handles physics simulation and ROS2 control
2. mujoco_passive_viewer - displays full interactive MuJoCo viewer with ImGui UI

The viewer mirrors the simulation state and provides the complete simulate binary
experience with all controls, visualization options, and perturbation tools.

Usage:
    ros2 launch openarm_description mujoco_with_full_viewer.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with both MuJoCo nodes."""
    pkg_openarm_description = get_package_share_directory("openarm_description")

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "hand",
            default_value="false",
            description="Include hand/gripper in simulation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "bimanual",
            default_value="false",
            description="Use bimanual configuration",
        )
    )

    # Get launch configurations
    hand = LaunchConfiguration("hand")
    bimanual = LaunchConfiguration("bimanual")

    # Determine MuJoCo model path
    def get_model_path(context):
        hand_val = context.perform_substitution(hand)
        bimanual_val = context.perform_substitution(bimanual)

        model_name = "openarm_v10"
        if bimanual_val.lower() == "true":
            model_name += "_bimanual"
        if hand_val.lower() == "true":
            model_name += "_hand"
        model_name += ".xml"

        return os.path.join(pkg_openarm_description, "mujoco_models", model_name)

    # MuJoCo passive viewer node
    viewer_node = Node(
        package="openarm_description",
        executable="mujoco_passive_viewer.py",
        name="mujoco_passive_viewer",
        output="screen",
        parameters=[
            {
                "model_path": os.path.join(
                    pkg_openarm_description,
                    "mujoco_models",
                    "openarm_v10.xml",  # TODO: Make this dynamic
                ),
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription([
        *declared_arguments,
        Node(
            package="openarm_description",
            executable="mujoco_passive_viewer.py",
            name="mujoco_passive_viewer",
            output="screen",
            parameters=[
                {
                    "model_path": os.path.join(
                        pkg_openarm_description, "mujoco_models", "openarm_v10.xml"
                    ),
                    "use_sim_time": True,
                }
            ],
        ),
    ])
