"""
Copyright 2025 Zordi, Inc. All rights reserved.

Launch file for OpenARM 7-DOF multimode control testing with gravity compensation.

This launch file:
1. Loads OpenARM with MuJoCo simulation
2. Loads multiple controllers (all inactive initially):
   - joint_trajectory_controller (standard position+velocity control)
   - zordi_joint_mit_controller (Hardware PD mode, MIT mode)
   - zordi_joint_effort_controller (Software PD mode, effort only)
   - zordi_joint_effort_grav_comp_controller (Pure gravity compensation, backdrivable)
   - zordi_joint_mit_rnea_controller (Full inverse dynamics with cubic splines)
   - zordi_cartesian_mit_controller (Cartesian impedance control with nullspace)
   - zordi_cartesian_mit_rnea_controller (Advanced Cartesian with RNEA)
3. Enables MuJoCo viewer for visualization
4. Starts at specified initial keyframe (default: home)
5. Simulation starts PAUSED (unpause via service)

Usage:
    # Launch with default home keyframe
    ros2 launch openarm_description test_openarm_multimode.launch.py

    # Launch with specific keyframe
    ros2 launch openarm_description test_openarm_multimode.launch.py \
      initial_keyframe:=pose1

    # Check loaded controllers
    ros2 control list_controllers

    # Activate a controller (joint space)
    ros2 control set_controller_state zordi_joint_effort_controller active

    # Activate a controller (Cartesian space)
    ros2 control set_controller_state zordi_cartesian_mit_controller active

Available Controllers:
    Joint Space:
    - joint_trajectory_controller: Standard ROS2 (pos+vel)
    - zordi_joint_mit_controller: Hardware PD (pos+vel+eff, MIT mode)
    - zordi_joint_effort_controller: Software PD (eff only, PD in controller)
    - zordi_joint_effort_grav_comp_controller: Gravity comp only (backdrivable)
    - zordi_joint_mit_rnea_controller: Full inverse dynamics with cubic splines

    Cartesian Space (7-DOF features):
    - zordi_cartesian_mit_controller: Cartesian impedance with nullspace control
    - zordi_cartesian_mit_rnea_controller: Advanced Cartesian with full RNEA
"""
# NOTE: Controllers are loaded via CLI loader (ros2 control load_controller)
# to avoid early parameter visibility issues with generate-parameter-library
# controllers (e.g., zordi_cartesian_mit_controller) when using the spawner node.

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
    """Generate launch description for OpenARM multimode testing."""
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
    urdf_file = pkg_share / "mujoco_models" / "openarm_v10.urdf"
    # NOTE: The URDF and XML files were generated from xacro using scripts/urdf_to_mjcf.py
    # This ensures Pinocchio and MuJoCo use identical inertial properties
    mujoco_model = pkg_share / "mujoco_models" / "openarm_v10.xml"
    controller_config = (
        pkg_share / "config" / "mujoco" / "controllers_multimode_test.yaml"
    )

    # Read URDF directly (like planar 2-DoF pattern)
    robot_description = Path(urdf_file).read_text(encoding="utf-8")

    # Robot state publisher (required for gravity compensation)
    # Controllers fetch robot_description from this node to initialize Pinocchio
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

    # Load joint_trajectory_controller (inactive)
    load_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--inactive"],
        output="screen",
    )

    # Load zordi_joint_mit_controller (inactive)
    load_mit = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_joint_mit_controller", "--inactive"],
        output="screen",
    )

    # Load zordi_joint_effort_controller (inactive)
    load_effort = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_joint_effort_controller", "--inactive"],
        output="screen",
    )

    # Load zordi_joint_effort_grav_comp_controller (inactive)
    load_grav_comp = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_joint_effort_grav_comp_controller", "--inactive"],
        output="screen",
    )

    # Load zordi_joint_mit_rnea_controller (inactive)
    load_rnea = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_joint_mit_rnea_controller", "--inactive"],
        output="screen",
    )

    # Load joint_state_broadcaster (active) via CLI loader pattern
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

    # Load zordi_cartesian_mit_controller (inactive) via CLI loader pattern
    load_cartesian = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "zordi_cartesian_mit_controller",
        ],
        output="screen",
    )

    # Load zordi_cartesian_mit_rnea_controller (inactive) via CLI loader pattern
    load_cartesian_rnea = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "zordi_cartesian_mit_rnea_controller",
        ],
        output="screen",
    )

    nodes_to_start = [
        robot_state_pub_node,  # Start first so controllers can fetch robot_description
        mujoco_node,
        # Load controllers when mujoco node starts (NO DELAY - like planar 2-DoF)
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_node,
                on_start=[
                    # Load all controllers (inactive)
                    load_joint_state_broadcaster,
                    load_jtc,
                    load_mit,
                    load_effort,
                    load_grav_comp,
                    load_rnea,
                    load_cartesian,
                    load_cartesian_rnea,
                ],
            )
        ),
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
