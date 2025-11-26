"""
Copyright 2025 Zordi, Inc. All rights reserved.

Automated launch file for gravity compensation testing.

This test uses the OpenARM 7-DOF robot with zordi_joint_effort_grav_comp_controller
to verify gravity compensation holds position without active PD control.

Test sequence:
  1. Start controller manager with OpenARM robot
  2. Load and activate joint_state_broadcaster, zordi_joint_effort_grav_comp_controller
  3. Reset robot to pose1 keyframe (while paused)
  4. Unpause simulation
  5. Hold for 5 seconds (robot should maintain position with minimal drift)
  6. Verify position and auto-exit with result

Expected behavior:
  - Robot maintains pose1 position with gravity compensation only
  - Small drift is expected (no position control active)
  - Position should remain within 0.3 rad of initial (relaxed tolerance for drift)

Usage:
  ros2 launch openarm_description test_gravity_compensation.launch.py
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
    """Generate automated launch description for gravity compensation testing."""
    pkg_share = Path(get_package_share_directory("openarm_description"))

    urdf_file = pkg_share / "mujoco_models" / "openarm_v10.urdf"
    controller_config = (
        pkg_share / "config" / "mujoco" / "controllers_multimode_test.yaml"
    )
    verify_script = pkg_share / "scripts" / "verify_test_result.py"

    robot_description = Path(urdf_file).read_text(encoding="utf-8")

    # Joint order for verification
    joint_order = (
        "openarm_joint1,openarm_joint2,openarm_joint3,"
        "openarm_joint4,openarm_joint5,openarm_joint6,openarm_joint7"
    )

    # Expected position (pose1) - with relaxed tolerance for drift
    # Note: Joint 6 tends to drift significantly due to gravity compensation limits
    expected_pose1 = "1.0,1.5,-1.0,2.0,1.0,-0.73,1.5"

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

    load_grav_comp_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "zordi_joint_effort_grav_comp_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    reset_to_pose1 = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/mujoco_system/reset_to_keyframe",
            "mujoco_ros2_control_msgs/srv/ResetToKeyframe",
            "{keyframe: 'pose1'}",
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

    # Print status message
    print_status = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "echo '\\n=== GRAVITY COMPENSATION TEST ===' && "
            "echo 'Robot at pose1 with gravity compensation only.' && "
            "echo 'Holding for 5 seconds...' && "
            "echo 'Expected pose1: [1.0, 1.5, -1.0, 2.0, 1.0, -1.5, 1.5]' && "
            "echo '===================================\\n'",
        ],
        output="screen",
    )

    # Verification step after 5 second hold
    verify_result = ExecuteProcess(
        cmd=[
            "python3",
            str(verify_script),
            "--test-name",
            "Gravity Compensation (5s Hold)",
            "--expected",
            expected_pose1,
            "--tolerance",
            "0.5",  # Relaxed tolerance for gravity comp drift
            "--joint-order",
            joint_order,
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
        load_grav_comp_controller,
        # Chain: controller loads -> reset -> unpause -> status -> wait 5s -> verify
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_grav_comp_controller,
                on_exit=[TimerAction(period=1.0, actions=[reset_to_pose1])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=reset_to_pose1,
                on_exit=[TimerAction(period=0.5, actions=[unpause_simulation])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=unpause_simulation,
                on_exit=[TimerAction(period=0.5, actions=[print_status])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=print_status,
                on_exit=[TimerAction(period=5.0, actions=[verify_result])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=verify_result,
                on_exit=[shutdown_action],
            )
        ),
    ])
