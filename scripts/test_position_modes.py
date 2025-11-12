#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Test script for comparing position_pid vs direct position control modes.

This script sends a simple trajectory from all-zeros to all-ones (1 radian)
over 5 seconds, starting and ending at zero velocity.

Usage:
    # Test PID mode (requires position_control_mode:=pid)
    python3 scripts/test_position_modes.py --mode pid

    # Test direct mode (requires position_control_mode:=direct)
    python3 scripts/test_position_modes.py --mode direct

    # Custom trajectory duration
    python3 scripts/test_position_modes.py --mode pid --duration 10.0
"""

import argparse
import sys
from typing import List

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryTester(Node):
    """Node to send test trajectories to JointTrajectoryController."""

    def __init__(self, mode: str = "pid"):
        """Initialize the trajectory tester node.

        Args:
            mode: Control mode ('pid' or 'direct')
        """
        super().__init__("trajectory_tester")

        # Determine controller name based on mode
        if mode == "pid":
            controller_name = "pid_trajectory_controller"
        elif mode == "direct":
            controller_name = "position_trajectory_controller"
        else:
            self.get_logger().error(f"Invalid mode: {mode}. Use 'pid' or 'direct'")
            raise ValueError(f"Invalid mode: {mode}")

        self.mode = mode
        self.controller_name = controller_name

        # Create action client
        self._action_client = ActionClient(
            self, FollowJointTrajectory, f"/{controller_name}/follow_joint_trajectory"
        )

        self.get_logger().info(
            f"Trajectory tester initialized for {mode} mode "
            f"(controller: {controller_name})"
        )

    def create_zero_to_one_trajectory(self, duration: float = 30.0) -> JointTrajectory:
        """Create a simple trajectory from 0 to 1 radian for all joints.

        Args:
            duration: Trajectory duration in seconds (default: 30.0)

        Returns:
            JointTrajectory message
        """
        self.get_logger().info(f"Creating trajectory with duration={duration} seconds")
        trajectory = JointTrajectory()

        # Joint names for OpenArm v10
        trajectory.joint_names = [
            "openarm_joint1",
            "openarm_joint2",
            "openarm_joint3",
            "openarm_joint4",
            "openarm_joint5",
            "openarm_joint6",
            "openarm_joint7",
        ]

        num_joints = len(trajectory.joint_names)

        # Waypoint 1: Start at zeros with zero velocity
        point_start = JointTrajectoryPoint()
        point_start.positions = [0.0] * num_joints
        point_start.velocities = [0.0] * num_joints
        point_start.time_from_start = Duration(sec=0, nanosec=0)

        # Waypoint 2: End at ones with zero velocity (smooth stop)
        point_end = JointTrajectoryPoint()
        point_end.positions = [1.0] * num_joints
        point_end.velocities = [0.0] * num_joints

        # Create Duration - be explicit about conversion
        duration_sec = int(duration)
        duration_nanosec = int((duration - float(duration_sec)) * 1_000_000_000)
        point_end.time_from_start = Duration(sec=duration_sec, nanosec=duration_nanosec)

        self.get_logger().info(
            f"Waypoint timing: sec={duration_sec}, nanosec={duration_nanosec}, "
            f"total={duration_sec + duration_nanosec / 1e9:.3f}s"
        )

        trajectory.points = [point_start, point_end]

        self.get_logger().info(
            f"Created trajectory: 0→1 rad in {duration}s for {num_joints} joints"
        )
        self.get_logger().info(
            f"  Start: positions={point_start.positions}, "
            f"velocities={point_start.velocities}"
        )
        self.get_logger().info(
            f"  End (t={point_end.time_from_start.sec}.{point_end.time_from_start.nanosec}s): "
            f"positions={point_end.positions}, velocities={point_end.velocities}"
        )

        return trajectory

    def send_trajectory(self, trajectory: JointTrajectory) -> bool:
        """Send trajectory to the controller via action interface.

        Args:
            trajectory: JointTrajectory to execute

        Returns:
            True if trajectory was accepted, False otherwise
        """
        self.get_logger().info(
            f"Waiting for action server: /{self.controller_name}/follow_joint_trajectory"
        )

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"Action server /{self.controller_name}/follow_joint_trajectory "
                "not available after 5 seconds"
            )
            self.get_logger().error("\nMake sure you launched with the correct mode:")
            self.get_logger().error(
                f"  ros2 launch openarm_description single_arm.launch.py "
                f"position_control_mode:={self.mode}"
            )
            return False

        # Create goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.get_logger().info("Sending trajectory goal...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal was rejected by controller")
            return False

        self.get_logger().info("Trajectory goal accepted! Executing...")

        # Wait for trajectory completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        error_code = result.error_code

        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("✓ Trajectory completed successfully!")
            return True
        else:
            self.get_logger().error(
                f"✗ Trajectory failed with error code: {error_code}"
            )
            return False


def main():
    """Main entry point for trajectory testing."""
    parser = argparse.ArgumentParser(
        description="Test position control modes with 0→1 trajectory"
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["pid", "direct"],
        default="pid",
        help="Control mode: 'pid' (position_pid) or 'direct' (position)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Trajectory duration in seconds (default: 30.0)",
    )

    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("OpenArm Position Control Mode Test")
    print("=" * 60)
    print(f"Mode: {args.mode}")
    print(f"Duration: {args.duration}s")
    print("Trajectory: 0.0 → 1.0 rad (all joints)")
    print("=" * 60 + "\n")

    rclpy.init()

    try:
        print(
            f"Python received: mode={args.mode}, duration={args.duration} (type={type(args.duration)})"
        )
        tester = TrajectoryTester(mode=args.mode)
        trajectory = tester.create_zero_to_one_trajectory(duration=args.duration)
        success = tester.send_trajectory(trajectory)

        if success:
            print("\n" + "=" * 60)
            print("✓ Test completed successfully!")
            print("=" * 60 + "\n")
            return 0
        else:
            print("\n" + "=" * 60)
            print("✗ Test failed!")
            print("=" * 60 + "\n")
            return 1

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        return 1
    except Exception as e:
        print(f"\nError: {e}")
        return 1
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
