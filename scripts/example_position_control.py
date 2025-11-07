#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Example script demonstrating position control of OpenARM in MuJoCo simulation.

This script sends a simple joint trajectory to the robot using the
JointTrajectoryController.

Usage:
    # Terminal 1: Start simulation
    ros2 launch openarm_description mujoco_sim.launch.py control_mode:=position

    # Terminal 2: Run this script
    python3 example_position_control.py
"""

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class PositionControlExample(Node):
    """Example node for position control."""

    def __init__(self):
        """Initialize node and action client."""
        super().__init__("position_control_example")

        self.get_logger().info("Initializing position control example...")

        # Create action client for joint trajectory controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        # Wait for action server
        self.get_logger().info("Waiting for action server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available!")
            raise RuntimeError("Action server not available")

        self.get_logger().info("Action server connected!")

    def send_trajectory(self, positions, duration=3.0):
        """Send a joint trajectory goal.

        Args:
            positions: List of 7 joint positions (rad)
            duration: Time to reach goal (seconds)
        """
        if len(positions) != 7:
            raise ValueError("Must provide 7 joint positions")

        # Create goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            "openarm_joint1",
            "openarm_joint2",
            "openarm_joint3",
            "openarm_joint4",
            "openarm_joint5",
            "openarm_joint6",
            "openarm_joint7",
        ]

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)

        goal_msg.trajectory.points = [point]

        self.get_logger().info(f"Sending trajectory to positions: {positions}")

        # Send goal
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        self.get_logger().info("Goal accepted! Waiting for result...")

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f"Result: {result.error_code}")

        return result.error_code == 0


def main():
    """Run example trajectory sequence."""
    rclpy.init()

    node = PositionControlExample()

    try:
        # Home position (all zeros)
        node.get_logger().info("\n=== Moving to home position ===")
        home_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        node.send_trajectory(home_positions, duration=3.0)

        time.sleep(1.0)

        # Configuration 1: Reach forward and up
        node.get_logger().info("\n=== Config 1: Reaching forward and up ===")
        config1 = [0.8, -0.6, 0.5, 1.2, -0.4, 0.3, 0.5]
        node.send_trajectory(config1, duration=4.0)

        time.sleep(1.0)

        # Configuration 2: Side reach with wrist rotation
        node.get_logger().info("\n=== Config 2: Side reach with wrist rotation ===")
        config2 = [-1.0, 0.8, -0.7, 0.9, 0.6, -0.5, -0.8]
        node.send_trajectory(config2, duration=4.0)

        time.sleep(1.0)

        # Configuration 3: High reach with elbow bent
        node.get_logger().info("\n=== Config 3: High reach with elbow bent ===")
        config3 = [0.3, -1.2, 0.9, 1.5, -0.3, 0.7, 0.2]
        node.send_trajectory(config3, duration=4.0)

        time.sleep(1.0)

        # Configuration 4: Low sweep position
        node.get_logger().info("\n=== Config 4: Low sweep position ===")
        config4 = [-0.7, 0.5, -1.0, 0.6, 0.8, -0.4, -0.6]
        node.send_trajectory(config4, duration=4.0)

        time.sleep(1.0)

        # Back to home
        node.get_logger().info("\n=== Returning to home position ===")
        node.send_trajectory(home_positions, duration=5.0)

        node.get_logger().info("\n=== Example complete! ===")

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")

    except Exception as e:
        node.get_logger().error(f"Error: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
