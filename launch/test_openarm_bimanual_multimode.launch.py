"""
Copyright 2025 Zordi, Inc. All rights reserved.

Launch file for OpenARM Bimanual 14-DOF multimode control testing.

This launch file:
1. Loads OpenARM Bimanual (two arms, no hands) with MuJoCo simulation
2. Loads multiple controllers for each arm (all inactive initially):
   Left Arm Controllers:
   - left_joint_trajectory_controller
   - left_zordi_hardware_pd_controller
   - left_zordi_software_pd_controller
   - left_zordi_grav_comp_controller
   - left_zordi_mit_rnea_controller
   - left_zordi_cartesian_controller
   - left_zordi_cartesian_rnea_controller

   Right Arm Controllers:
   - right_joint_trajectory_controller
   - right_zordi_hardware_pd_controller
   - right_zordi_software_pd_controller
   - right_zordi_grav_comp_controller
   - right_zordi_mit_rnea_controller
   - right_zordi_cartesian_controller
   - right_zordi_cartesian_rnea_controller

3. Enables MuJoCo viewer for visualization
4. Starts at specified initial keyframe (default: home)
5. Simulation starts with unpause flag

Usage:
    # Launch with default home keyframe
    ros2 launch openarm_description test_openarm_bimanual_multimode.launch.py

    # Launch with specific keyframe
    ros2 launch openarm_description test_openarm_bimanual_multimode.launch.py \
      initial_keyframe:=pose1

    # Check loaded controllers
    ros2 control list_controllers

    # Activate left arm controller
    ros2 control set_controller_state left_zordi_cartesian_controller active

    # Activate right arm controller
    ros2 control set_controller_state right_zordi_cartesian_controller active

Available Controllers:
    Left Arm:
    - left_joint_trajectory_controller: Standard ROS2 (pos+vel)
    - left_zordi_hardware_pd_controller: Hardware PD (MIT mode)
    - left_zordi_software_pd_controller: Software PD (effort only)
    - left_zordi_grav_comp_controller: Gravity comp only (backdrivable)
    - left_zordi_mit_rnea_controller: Full inverse dynamics
    - left_zordi_cartesian_controller: Cartesian impedance with nullspace
    - left_zordi_cartesian_rnea_controller: Advanced Cartesian with RNEA

    Right Arm:
    - right_joint_trajectory_controller: Standard ROS2 (pos+vel)
    - right_zordi_hardware_pd_controller: Hardware PD (MIT mode)
    - right_zordi_software_pd_controller: Software PD (effort only)
    - right_zordi_grav_comp_controller: Gravity comp only (backdrivable)
    - right_zordi_mit_rnea_controller: Full inverse dynamics
    - right_zordi_cartesian_controller: Cartesian impedance with nullspace
    - right_zordi_cartesian_rnea_controller: Advanced Cartesian with RNEA
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for OpenARM bimanual multimode testing."""
    pkg_share = Path(get_package_share_directory("openarm_description"))

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "initial_keyframe",
            default_value="home",
            description="Initial keyframe from MuJoCo XML model",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run MuJoCo in headless mode (no viewer)",
        ),
    ]

    # Get launch configurations
    initial_keyframe = LaunchConfiguration("initial_keyframe")
    headless = LaunchConfiguration("headless")

    # Paths
    urdf_file = pkg_share / "mujoco_models" / "openarm_v10_bimanual.urdf"
    mujoco_model = pkg_share / "mujoco_models" / "openarm_v10_bimanual.xml"
    controller_config = (
        pkg_share / "config" / "mujoco" / "controllers_bimanual_multimode_test.yaml"
    )

    # Read URDF
    robot_description = Path(urdf_file).read_text(encoding="utf-8")

    # Robot state publisher (required for gravity compensation)
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    # MuJoCo ROS2 Control node
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            str(controller_config),
            {
                "robot_description": robot_description,
                "mujoco_model_path": str(mujoco_model),
                "headless": headless,
                "unpause": True,
                "initial_keyframe": initial_keyframe,
            },
        ],
        output="screen",
    )

    # Load joint_state_broadcaster (active)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    # Left arm controllers
    load_left_jtc = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "left_joint_trajectory_controller",
        ],
        output="screen",
    )

    load_left_hw_pd = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "left_zordi_hardware_pd_controller",
        ],
        output="screen",
    )

    load_left_sw_pd = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "left_zordi_software_pd_controller",
        ],
        output="screen",
    )

    load_left_grav_comp = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "left_zordi_grav_comp_controller",
        ],
        output="screen",
    )

    load_left_rnea = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "left_zordi_mit_rnea_controller",
        ],
        output="screen",
    )

    load_left_cartesian = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "left_zordi_cartesian_controller",
        ],
        output="screen",
    )

    load_left_cartesian_rnea = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "left_zordi_cartesian_rnea_controller",
        ],
        output="screen",
    )

    # Right arm controllers
    load_right_jtc = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "right_joint_trajectory_controller",
        ],
        output="screen",
    )

    load_right_hw_pd = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "right_zordi_hardware_pd_controller",
        ],
        output="screen",
    )

    load_right_sw_pd = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "right_zordi_software_pd_controller",
        ],
        output="screen",
    )

    load_right_grav_comp = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "right_zordi_grav_comp_controller",
        ],
        output="screen",
    )

    load_right_rnea = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "right_zordi_mit_rnea_controller",
        ],
        output="screen",
    )

    load_right_cartesian = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "right_zordi_cartesian_controller",
        ],
        output="screen",
    )

    load_right_cartesian_rnea = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "right_zordi_cartesian_rnea_controller",
        ],
        output="screen",
    )

    nodes_to_start = [
        robot_state_pub_node,
        mujoco_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_node,
                on_start=[
                    load_joint_state_broadcaster,
                    # Left arm controllers
                    load_left_jtc,
                    load_left_hw_pd,
                    load_left_sw_pd,
                    load_left_grav_comp,
                    load_left_rnea,
                    load_left_cartesian,
                    load_left_cartesian_rnea,
                    # Right arm controllers
                    load_right_jtc,
                    load_right_hw_pd,
                    load_right_sw_pd,
                    load_right_grav_comp,
                    load_right_rnea,
                    load_right_cartesian,
                    load_right_cartesian_rnea,
                ],
            )
        ),
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
