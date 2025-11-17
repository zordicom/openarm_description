"""
Copyright 2025 Zordi, Inc. All rights reserved.

Launch file for OpenARM 7-DOF multimode control testing with gravity compensation.

This launch file:
1. Loads OpenARM with MuJoCo simulation
2. Loads multiple controllers (all inactive initially):
   - joint_trajectory_controller (standard position+velocity control)
   - zordi_hardware_pd_controller (Hardware PD mode, MIT mode)
   - zordi_software_pd_controller (Software PD mode, effort only)
   - zordi_grav_comp_controller (Pure gravity compensation, backdrivable)
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

    # Activate a controller
    ros2 control set_controller_state zordi_software_pd_controller active

Available Controllers:
    - joint_trajectory_controller: Standard ROS2 (pos+vel)
    - zordi_hardware_pd_controller: Hardware PD (pos+vel+eff, MIT mode)
    - zordi_software_pd_controller: Software PD (eff only, PD in controller)
    - zordi_grav_comp_controller: Gravity comp only (backdrivable)
    - zordi_mit_rnea_controller: Full inverse dynamics with cubic splines
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    urdf_file = pkg_share / "urdf" / "robot" / "v10.urdf.xacro"
    # NOTE: The XML file was generated from URDF using scripts/urdf_to_mjcf.py
    # This ensures Pinocchio and MuJoCo use identical inertial properties
    mujoco_model = str(pkg_share / "mujoco_models" / "openarm_v10.xml")
    controller_config = str(
        pkg_share / "config" / "mujoco" / "controllers_multimode_test.yaml"
    )

    # Generate robot description
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            str(urdf_file),
            " ",
            "ros2_control:=true",
            " ",
            "use_mujoco:=true",
            " ",
            "hand:=false",
            " ",
            "bimanual:=false",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Robot state publisher (required for gravity compensation)
    # Controllers fetch robot_description from this node to initialize Pinocchio
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # MuJoCo ROS2 Control node
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            {
                "mujoco_model_path": mujoco_model,
                "headless": headless,
                "unpause": True,
                "initial_keyframe": initial_keyframe,
            },
            controller_config,
        ],
        output="screen",
    )

    # Joint state broadcaster (start immediately)
    spawn_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Load joint_trajectory_controller (inactive)
    load_jtc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--inactive"],
        output="screen",
    )

    # Load zordi_hardware_pd_controller (inactive)
    load_hw_pd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_hardware_pd_controller", "--inactive"],
        output="screen",
    )

    # Load zordi_software_pd_controller (inactive)
    load_sw_pd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_software_pd_controller", "--inactive"],
        output="screen",
    )

    # Load zordi_grav_comp_controller (inactive)
    load_grav_comp = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_grav_comp_controller", "--inactive"],
        output="screen",
    )

    # Load zordi_mit_rnea_controller (inactive)
    load_rnea = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_mit_rnea_controller", "--inactive"],
        output="screen",
    )

    nodes_to_start = [
        robot_state_pub_node,  # Start first so controllers can fetch robot_description
        mujoco_node,
        # Start controllers when mujoco node starts
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=mujoco_node,
                on_start=[
                    spawn_joint_broadcaster,
                    load_jtc,
                    load_hw_pd,
                    load_sw_pd,
                    load_grav_comp,
                    load_rnea,
                ],
            )
        ),
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
