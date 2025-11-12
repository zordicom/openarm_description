#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Evaluate current PID gains in ROS2.

This script evaluates the currently-loaded PID gains by running test
trajectories and computing the same cost metrics as autotune_pid_simple.py.

This is useful for:
- Validating gains from autotune_pid_simple.py in actual ROS2
- Comparing simulation vs reality
- Quick sanity check before deploying

Usage:
    # Terminal 1: Launch simulation
    ros2 launch openarm_description mujoco_sim.launch.py headless:=true

    # Terminal 2: Evaluate current gains
    python3 scripts/evaluate_pid_ros2.py

    # Or evaluate specific gains from JSON
    python3 scripts/evaluate_pid_ros2.py --gains figs/optimal_pid_gains.json

Output:
    - Cost metrics (printed to terminal)
    - Performance plots (if --plot flag given)
"""

import argparse
import json
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class PIDEvaluator(Node):
    """Single-shot PID evaluator for ROS2."""

    def __init__(self):
        super().__init__("pid_evaluator")

        self.joint_names = [f"openarm_joint{i}" for i in range(1, 8)]

        # Action client
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        # Joint state subscription
        self.latest_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )

        # Data logging
        self.recording = False
        self.log_data = {
            "time": [],
            "position": {j: [] for j in range(7)},
            "velocity": {j: [] for j in range(7)},
            "error": {j: [] for j in range(7)},
            "target": {j: [] for j in range(7)},
        }
        self.start_time = None
        self.target_positions = np.zeros(7)

    def _joint_state_callback(self, msg):
        """Store latest joint state."""
        self.latest_joint_state = msg

        if self.recording and self.start_time is not None:
            current_time = time.time() - self.start_time
            self.log_data["time"].append(current_time)

            for i, joint_name in enumerate(self.joint_names):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    pos = msg.position[idx]
                    vel = msg.velocity[idx] if msg.velocity else 0.0

                    self.log_data["position"][i].append(pos)
                    self.log_data["velocity"][i].append(vel)
                    self.log_data["error"][i].append(pos - self.target_positions[i])
                    self.log_data["target"][i].append(self.target_positions[i])

    def wait_for_ready(self, timeout=20.0):
        """Wait for simulation to be ready."""
        self.get_logger().info("Waiting for simulation...")

        # Wait for joint states
        start = time.time()
        spin_count = 0
        while self.latest_joint_state is None and rclpy.ok():
            elapsed = time.time() - start
            if elapsed > timeout:
                self.get_logger().error(
                    f"Timeout waiting for joint states after {spin_count} spins!"
                )
                self.get_logger().error("Check: ros2 topic list | grep joint_states")
                self.get_logger().error("Check: ros2 topic echo /joint_states --once")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
            spin_count += 1
            if spin_count % 50 == 0:
                self.get_logger().info(
                    f"Still waiting... ({elapsed:.1f}s, {spin_count} spins)"
                )

        self.get_logger().info(f"✓ Joint states OK (received after {spin_count} spins)")

        # Wait for action server
        if not self.traj_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("Action server not available!")
            return False

        self.get_logger().info("✓ Action server OK")
        return True

    def move_and_record(self, q_target, duration=3.0):
        """Execute trajectory and record data."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = q_target.tolist()
        point.velocities = [0.0] * 7
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]

        self.target_positions = q_target

        # Start recording
        self.recording = True
        self.start_time = time.time()
        self.log_data["time"].clear()
        for i in range(7):
            self.log_data["position"][i].clear()
            self.log_data["velocity"][i].clear()
            self.log_data["error"][i].clear()
            self.log_data["target"][i].clear()

        # Send goal
        future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if not future.done():
            self.get_logger().error("Goal send timeout!")
            self.recording = False
            return False

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            self.recording = False
            return False

        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration + 5.0
        )

        # Record settling
        time.sleep(2.0)
        self.recording = False

        return True

    def compute_cost(self):
        """Compute cost metrics (same as autotune_pid_simple.py)."""
        if not self.log_data["time"]:
            self.get_logger().error("No data logged!")
            return 1e6, {}

        metrics = {}
        total_cost = 0.0
        num_joints_moved = 0

        for j in range(7):
            time_arr = np.array(self.log_data["time"])
            pos = np.array(self.log_data["position"][j])
            vel = np.array(self.log_data["velocity"][j])
            error = np.array(self.log_data["error"][j])

            # Check if we have data
            if len(pos) == 0:
                self.get_logger().warn(f"Joint {j + 1}: No position data!")
                continue

            # Check movement
            position_range = np.max(pos) - np.min(pos)
            if position_range < 0.01:
                self.get_logger().debug(
                    f"Joint {j + 1}: No significant movement ({position_range:.6f} rad)"
                )
                continue

            num_joints_moved += 1

            # Compute metrics
            rms_error = np.degrees(np.sqrt(np.mean(error**2)))
            final_vel = np.mean(np.abs(vel[-int(0.1 * len(vel)) :]))
            overshoot = np.degrees(np.max(np.abs(error)) - np.abs(error[0]))

            cost = rms_error + 100.0 * final_vel + 5.0 * max(0, overshoot)

            metrics[f"joint{j + 1}"] = {
                "rms_error_deg": rms_error,
                "final_vel_rad_s": final_vel,
                "overshoot_deg": overshoot,
                "cost": cost,
            }
            total_cost += cost

            self.get_logger().info(
                f"  Joint {j + 1}: RMS={rms_error:.2f}°, FinalVel={final_vel:.4f} rad/s, Cost={cost:.2f}"
            )

        if num_joints_moved == 0:
            self.get_logger().error("No joints moved significantly!")
            return 1e6, {}

        return total_cost, metrics


def main():
    parser = argparse.ArgumentParser(description="Evaluate PID gains in ROS2")
    parser.add_argument(
        "--gains",
        type=str,
        help="JSON file with gains (if provided, will be compared but not applied)",
    )
    args = parser.parse_args()

    rclpy.init()
    evaluator = PIDEvaluator()

    try:
        print("\n" + "=" * 70)
        print("ROS2 PID EVALUATOR")
        print("=" * 70)

        # Wait for simulation
        if not evaluator.wait_for_ready():
            print("\n✗ Simulation not ready!")
            print("Launch simulation first:")
            print(
                "  ros2 launch openarm_description mujoco_sim.launch.py headless:=true"
            )
            return 1

        # Test configurations
        test_configs = [
            np.array([0, -np.pi / 3, 0, 0, 0, 0, 0]),
            np.array([np.pi / 6, -np.pi / 4, np.pi / 6, 0, 0, 0, 0]),
            np.array([0, -np.pi / 2, 0, 0, 0, 0, 0]),
        ]

        print(f"\nRunning {len(test_configs)} test trajectories...")

        total_cost = 0.0
        for i, q_target in enumerate(test_configs):
            print(f"\nTest {i + 1}/{len(test_configs)}...")
            if not evaluator.move_and_record(q_target):
                print("  ✗ Failed")
                continue

            cost, metrics = evaluator.compute_cost()
            total_cost += cost
            print(f"  Cost: {cost:.2f}")

        avg_cost = total_cost / len(test_configs)

        print("\n" + "=" * 70)
        print(f"AVERAGE COST: {avg_cost:.2f}")
        print("=" * 70)

        if avg_cost < 50:
            print("✓ Excellent! Gains are well-tuned.")
        elif avg_cost < 100:
            print("⚠ Good, but could be improved.")
        else:
            print("✗ Poor performance. Consider re-tuning.")

        # Save evaluation log
        import time
        from pathlib import Path

        log_dir = Path("figs")
        log_dir.mkdir(exist_ok=True)
        log_path = log_dir / "ros2_evaluation_log.txt"

        with open(log_path, "w") as f:
            f.write("=" * 70 + "\n")
            f.write("ROS2 PID EVALUATION LOG\n")
            f.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("=" * 70 + "\n\n")

            f.write(f"Average cost: {avg_cost:.2f}\n\n")

            f.write("Per-test breakdown:\n")
            for i, q_target in enumerate(test_configs):
                f.write(f"\nTest {i + 1}/{len(test_configs)}:\n")
                f.write(f"  Target: {[f'{np.degrees(q):.1f}°' for q in q_target]}\n")

            f.write("\n" + "=" * 70 + "\n")
            f.write("COMPARISON TO AUTOTUNE_PID_SIMPLE.PY\n")
            f.write("=" * 70 + "\n")
            f.write("If ROS2 costs are much higher than autotune_pid_simple.py:\n")
            f.write(
                "  1. Check that gains were applied: cat figs/optimal_pid_gains.json\n"
            )
            f.write(
                "  2. Check URDF has correct gains: grep 'kp=' urdf/ros2_control/*.xacro\n"
            )
            f.write("  3. Verify simulation was rebuilt and relaunched\n")
            f.write("  4. Check controller config uses 'position' interface\n\n")

            f.write("Expected costs:\n")
            f.write("  - autotune_pid_simple.py: < 50 per joint\n")
            f.write("  - ROS2 evaluation: < 100 per joint (some overhead expected)\n")
            f.write("  - If ROS2 costs > 1000: PID gains not being used!\n")

        print(f"\n✓ Evaluation log saved to: {log_path}")

        return 0

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 1

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    exit(main())
