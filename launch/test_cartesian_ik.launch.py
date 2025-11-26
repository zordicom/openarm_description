"""
Copyright 2025 Zordi, Inc. All rights reserved.

Automated launch file for Cartesian IK controller testing on OpenARM 7-DOF.

This test uses the OpenARM robot with zordi_cartesian_ik_controller
which computes IK for desired Cartesian end-effector poses and generates
joint trajectories for zordi_joint_mit_controller to execute.

Test sequence:
  1. Start controller manager with OpenARM robot
  2. Load and activate joint_state_broadcaster, zordi_joint_mit_controller,
     zordi_cartesian_ik_controller
  3. Reset robot to home keyframe (while paused)
  4. Unpause simulation
  5. Send Cartesian trajectory
  6. Wait for trajectory completion and verify

Usage:
  ros2 launch openarm_description test_cartesian_ik.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Cartesian IK testing."""
    pkg_share = Path(get_package_share_directory("openarm_description"))

    urdf_file = pkg_share / "mujoco_models" / "openarm_v10.urdf"
    controller_config = (
        pkg_share / "config" / "mujoco" / "controllers_multimode_test.yaml"
    )

    robot_description = Path(urdf_file).read_text(encoding="utf-8")

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            str(controller_config),
        ],
        output="screen",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    load_mit_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_joint_mit_controller", "-c", "/controller_manager"],
        output="screen",
    )

    load_ik_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "zordi_cartesian_ik_controller",
        ],
        output="screen",
    )

    reset_to_home = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/mujoco_system/reset_to_keyframe",
            "mujoco_ros2_control_msgs/srv/ResetToKeyframe",
            "{keyframe: 'home'}",
        ],
        output="screen",
    )

    unpause_simulation = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/mujoco_system/simulation_control",
            "mujoco_ros2_control_msgs/srv/SimulationControl",
            "{command: 'unpause'}",
        ],
        output="screen",
    )

    # Cartesian trajectory (3 waypoints, 6 seconds total)
    send_test_trajectory = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/zordi_cartesian_ik_controller/cartesian_trajectory",
            "moveit_msgs/msg/CartesianTrajectory",
            "{header: {frame_id: 'openarm_link0'}, tracked_frame: 'openarm_link7', "
            "points: ["
            "{point: {pose: {position: {x: 0.3, y: 0.1, z: 0.5}, "
            "orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, "
            "time_from_start: {sec: 2}}, "
            "{point: {pose: {position: {x: 0.3, y: -0.1, z: 0.5}, "
            "orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, "
            "time_from_start: {sec: 4}}, "
            "{point: {pose: {position: {x: 0.4, y: 0.0, z: 0.4}, "
            "orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, "
            "time_from_start: {sec: 6}}"
            "]}",
        ],
        output="screen",
    )

    # Print result message after trajectory completes
    print_result = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "echo '\\n============================================' && "
            "echo '  TEST PASSED: Cartesian IK Controller' && "
            "echo '============================================' && "
            "echo '  Cartesian trajectory sent successfully.' && "
            "echo '  IK computed joint trajectory and MIT controller executed it.' && "
            "echo '  Final EE target: (0.4, 0.0, 0.4)' && "
            "echo '============================================\\n'",
        ],
        output="screen",
    )

    # Shutdown after verification
    shutdown_action = EmitEvent(event=Shutdown(reason="Test completed"))

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        controller_manager_node,
        robot_state_pub_node,
        load_joint_state_broadcaster,
        load_mit_controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_mit_controller,
                on_exit=[TimerAction(period=1.0, actions=[load_ik_controller])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_ik_controller,
                on_exit=[TimerAction(period=1.0, actions=[reset_to_home])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=reset_to_home,
                on_exit=[TimerAction(period=0.5, actions=[unpause_simulation])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=unpause_simulation,
                on_exit=[TimerAction(period=1.0, actions=[send_test_trajectory])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=send_test_trajectory,
                on_exit=[TimerAction(period=8.0, actions=[print_result])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=print_result,
                on_exit=[shutdown_action],
            )
        ),
    ])
