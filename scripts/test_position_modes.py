#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Test script for joint_trajectory_controller with real-time synchronized motion.

After fixing the simulation timing (mujoco_ros2_control_node.cpp), trajectories
now execute at correct wall-clock time!

This script sends a trajectory from 0 to target angle over specified duration.

⚠️  NOTE: Joint5-7 have 7 Nm torque limits. Use target ≤ 0.5 rad to avoid
    saturation at extended configurations.

Usage:
    # Test with safe angles (default: 0.5 rad, 10s)
    python3 scripts/test_position_modes.py

    # Custom target angle (radians)
    python3 scripts/test_position_modes.py --target 0.3

    # Custom duration (seconds)
    python3 scripts/test_position_modes.py --duration 5.0

    # Combined
    python3 scripts/test_position_modes.py --target 0.5 --duration 10.0

    # Launch simulation first:
    ros2 launch openarm_description single_arm.launch.py \
        default_controller:=joint_trajectory_controller
"""

import argparse
import sys

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryTester(Node):
    """Node to send test trajectories to joint_trajectory_controller."""

    def __init__(self):
        """Initialize the trajectory tester node."""
        super().__init__("trajectory_tester")

        self.controller_name = "joint_trajectory_controller"

        # Create action client
        action_topic = f"/{self.controller_name}/follow_joint_trajectory"
        self._action_client = ActionClient(self, FollowJointTrajectory, action_topic)

        self.get_logger().info(
            f"Trajectory tester initialized (controller: {self.controller_name})"
        )

    def create_zero_to_one_trajectory(
        self, duration: float = 10.0, target_angle: float = 1.0
    ) -> JointTrajectory:
        """Create a simple trajectory from 0 to target_angle for all joints.

        Args:
            duration: Trajectory duration in seconds (default: 10.0)
            target_angle: Target angle in radians (default: 1.0)

        Returns:
            JointTrajectory message
        """
        self.get_logger().info(
            f"Creating trajectory with duration={duration}s, target={target_angle} rad"
        )
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

        # Waypoint 2: End at target_angle with zero velocity (smooth stop)
        point_end = JointTrajectoryPoint()
        point_end.positions = [target_angle] * num_joints
        point_end.velocities = [0.0] * num_joints

        # Create Duration - be explicit about conversion
        duration_sec = int(duration)
        duration_nanosec = int((duration - float(duration_sec)) * 1_000_000_000)
        point_end.time_from_start = Duration(sec=duration_sec, nanosec=duration_nanosec)

        total_time = duration_sec + duration_nanosec / 1e9
        self.get_logger().info(
            f"Waypoint timing: sec={duration_sec}, nanosec={duration_nanosec}, "
            f"total={total_time:.3f}s"
        )

        trajectory.points = [point_start, point_end]

        self.get_logger().info(
            f"Created trajectory: 0→{target_angle} rad in {duration}s "
            f"for {num_joints} joints"
        )
        self.get_logger().info(
            f"  Start: positions={point_start.positions}, "
            f"velocities={point_start.velocities}"
        )
        end_time_sec = point_end.time_from_start.sec
        end_time_nsec = point_end.time_from_start.nanosec
        self.get_logger().info(
            f"  End (t={end_time_sec}.{end_time_nsec}s): "
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
        action_topic = f"/{self.controller_name}/follow_joint_trajectory"
        self.get_logger().info(f"Waiting for action server: {action_topic}")

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"Action server /{self.controller_name}/follow_joint_trajectory "
                "not available after 5 seconds"
            )
            self.get_logger().error("\nMake sure you launched the simulation:")
            self.get_logger().error(
                "  ros2 launch openarm_description single_arm.launch.py "
                "default_controller:=joint_trajectory_controller"
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
        description="Test joint_trajectory_controller with trajectory"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Trajectory duration in seconds (default: 10.0)",
    )
    parser.add_argument(
        "--target",
        type=float,
        default=0.5,
        help="Target angle in radians (default: 0.5, safe for all joints)",
    )

    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("OpenArm joint_trajectory_controller Test")
    print("=" * 60)
    print("Controller: joint_trajectory_controller")
    print("Mode: position_servo (auto-triggered)")
    print(f"Duration: {args.duration}s")
    print(f"Trajectory: 0.0 → {args.target} rad (all joints)")
    print("=" * 60 + "\n")

    rclpy.init()

    try:
        tester = TrajectoryTester()
        trajectory = tester.create_zero_to_one_trajectory(
            duration=args.duration, target_angle=args.target
        )
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
