"""
Copyright 2025 Zordi, Inc. All rights reserved.

Automated launch file for Cartesian IK controller testing on OpenARM 7-DOF.

This test uses the OpenARM robot with zordi_cartesian_ik_controller
which computes IK for desired Cartesian end-effector poses and generates
joint trajectories for zordi_joint_mit_controller to execute.

FK-verified waypoints (computed from home position, all joints at 0):
  home (START): q=[0,0,0,0,0,0,0] -> EE=(0.0, 0.0, 0.5585)
  wp1: q=[0.2, 0.3, 0.0, 0.2, 0.0, 0.0, 0.0] -> EE=(0.0167, 0.1336, 0.5349)
  wp2: q=[-0.2, 0.3, 0.0, 0.2, 0.0, 0.0, 0.0] -> EE=(0.0674, 0.1165, 0.5349)
  wp3: q=[0.0, 0.4, 0.0, 0.3, 0.0, 0.0, 0.0] -> EE=(0.0638, 0.1660, 0.5152)

Test sequence:
  1. Start controller manager with OpenARM robot (starts at home)
  2. Load and activate joint_state_broadcaster, zordi_joint_mit_controller,
     zordi_cartesian_ik_controller
  3. Unpause simulation
  4. Send sequential PoseStamped targets (FK-verified waypoints)
  5. Verify robot executes trajectory

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

    # FK-verified waypoints from home position (all joints at 0)
    # wp1: q=[0.2, 0.3, 0.0, 0.2, 0.0, 0.0, 0.0] -> EE=(0.0167, 0.1336, 0.5349)
    send_pose_wp1 = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/zordi_cartesian_ik_controller/target_pose",
            "geometry_msgs/msg/PoseStamped",
            "{header: {frame_id: 'openarm_link0'}, "
            "pose: {position: {x: 0.0167, y: 0.1336, z: 0.5349}, "
            "orientation: {w: 0.9804, x: -0.1578, y: 0.0834, z: 0.0834}}}",
        ],
        output="screen",
    )

    # wp2: q=[-0.2, 0.3, 0.0, 0.2, 0.0, 0.0, 0.0] -> EE=(0.0674, 0.1165, 0.5349)
    send_pose_wp2 = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/zordi_cartesian_ik_controller/target_pose",
            "geometry_msgs/msg/PoseStamped",
            "{header: {frame_id: 'openarm_link0'}, "
            "pose: {position: {x: 0.0674, y: 0.1165, z: 0.5349}, "
            "orientation: {w: 0.9774, x: -0.1381, y: 0.1131, z: -0.1131}}}",
        ],
        output="screen",
    )

    # wp3: q=[0.0, 0.4, 0.0, 0.3, 0.0, 0.0, 0.0] -> EE=(0.0638, 0.1660, 0.5152)
    send_pose_wp3 = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/zordi_cartesian_ik_controller/target_pose",
            "geometry_msgs/msg/PoseStamped",
            "{header: {frame_id: 'openarm_link0'}, "
            "pose: {position: {x: 0.0638, y: 0.1660, z: 0.5152}, "
            "orientation: {w: 0.9691, x: -0.1964, y: 0.1465, z: -0.0297}}}",
        ],
        output="screen",
    )

    # Print result message (note: this only indicates commands were sent)
    print_result = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "echo '\\n============================================' && "
            "echo '  TEST COMPLETE: Cartesian IK Controller' && "
            "echo '============================================' && "
            "echo '  Sent 3 FK-verified PoseStamped targets.' && "
            "echo '  Check console for IK convergence errors.' && "
            "echo '  Check viewer to verify robot movement.' && "
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
        # Chain: MIT loads -> IK loads -> unpause -> poses -> result
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_mit_controller,
                on_exit=[TimerAction(period=1.0, actions=[load_ik_controller])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_ik_controller,
                on_exit=[TimerAction(period=1.0, actions=[unpause_simulation])],
            )
        ),
        # After unpause, wait for robot to settle then send wp1
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=unpause_simulation,
                on_exit=[TimerAction(period=2.0, actions=[send_pose_wp1])],
            )
        ),
        # After wp1, wait and send wp2
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=send_pose_wp1,
                on_exit=[TimerAction(period=3.0, actions=[send_pose_wp2])],
            )
        ),
        # After wp2, wait and send wp3
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=send_pose_wp2,
                on_exit=[TimerAction(period=3.0, actions=[send_pose_wp3])],
            )
        ),
        # After wp3, wait and print result
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=send_pose_wp3,
                on_exit=[TimerAction(period=3.0, actions=[print_result])],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=print_result,
                on_exit=[shutdown_action],
            )
        ),
    ])
