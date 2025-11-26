"""
Copyright 2025 Zordi, Inc. All rights reserved.

Automated launch file for RNEA joint controller trajectory tracking.

This test uses the OpenARM 7-DOF robot with zordi_joint_mit_rnea_controller
for trajectory tracking with full inverse dynamics (M(q)*qdd + C(q,qd) + g(q)).

The RNEA controller provides:
  - Full inverse dynamics compensation
  - Cubic Hermite spline interpolation
  - Acceleration feedforward for superior tracking
  - Computed torque control with feedback

Test sequence:
  1. Start controller manager with OpenARM robot
  2. Load and activate joint_state_broadcaster, zordi_joint_mit_rnea_controller
  3. Reset robot to home keyframe (while paused)
  4. Unpause simulation
  5. Send test trajectory (home -> pose1 -> home)
  6. Verify final position and auto-exit with result

Usage:
  ros2 launch openarm_description test_joint_rnea.launch.py

Expected result: Robot returns to home [0,0,0,0,0,0,0] with < 0.1 rad error
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
    """Generate automated launch description for RNEA controller testing."""
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

    # Expected final position (home)
    expected_home = "0.0,0.0,0.0,0.0,0.0,0.0,0.0"

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

    load_rnea_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zordi_joint_mit_rnea_controller", "-c", "/controller_manager"],
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

    # Trajectory: home -> pose1 -> home
    send_test_trajectory = ExecuteProcess(
        cmd=[
            "ros2",
            "action",
            "send_goal",
            "/zordi_joint_mit_rnea_controller/follow_joint_trajectory",
            "control_msgs/action/FollowJointTrajectory",
            """{
                trajectory: {
                    joint_names: [openarm_joint1, openarm_joint2, openarm_joint3,
                                  openarm_joint4, openarm_joint5, openarm_joint6,
                                  openarm_joint7],
                    points: [
                        {
                            positions: [1.0, 1.5, -1.0, 2.0, 1.0, -0.5, 1.5],
                            time_from_start: {sec: 3}
                        },
                        {
                            positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            time_from_start: {sec: 6}
                        }
                    ]
                }
            }""",
        ],
        output="screen",
    )

    # Verification step
    verify_result = ExecuteProcess(
        cmd=[
            "python3",
            str(verify_script),
            "--test-name",
            "Joint Trajectory (RNEA Controller)",
            "--expected",
            expected_home,
            "--tolerance",
            "0.1",
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
        load_rnea_controller,
        # Chain: RNEA loads -> reset -> unpause -> trajectory -> verify -> shutdown
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_rnea_controller,
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
                on_exit=[TimerAction(period=2.0, actions=[verify_result])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=verify_result,
                on_exit=[shutdown_action],
            )
        ),
    ])
