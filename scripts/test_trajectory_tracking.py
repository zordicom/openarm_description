#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Test trajectory tracking performance to evaluate PID gains.

This script measures:
1. Tracking error during trajectory execution
2. Settling time to zero velocity
3. Steady-state error at goal
4. Overshoot and oscillation

Use this to tune PID gains in openarm.ros2_control.xacro

Usage:
    # Terminal 1: Launch simulation
    ros2 launch openarm_description mujoco_sim.launch.py

    # Terminal 2: Run test
    python3 scripts/test_trajectory_tracking.py

    # After tuning, update gains in:
    #   urdf/ros2_control/openarm.ros2_control.xacro (lines 91-97)
    # Then rebuild and test again:
    #   colcon build --packages-select openarm_description
"""

import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # Non-interactive backend for headless/remote use
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryTrackingTester(Node):
    """Test trajectory tracking performance for PID tuning."""

    def __init__(self):
        """Initialize the tester."""
        super().__init__("trajectory_tracking_tester")

        self.joint_names = [
            "openarm_joint1",
            "openarm_joint2",
            "openarm_joint3",
            "openarm_joint4",
            "openarm_joint5",
            "openarm_joint6",
            "openarm_joint7",
        ]

        # Action client
        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        # Joint state subscription
        self.latest_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Data logging
        self.recording = False
        self.log_data = {
            "time": [],
            "position": {j: [] for j in range(7)},
            "velocity": {j: [] for j in range(7)},
            "position_error": {j: [] for j in range(7)},
            "target_position": {j: [] for j in range(7)},
        }
        self.start_time = None
        self.target_positions = np.zeros(7)

    def joint_state_callback(self, msg):
        """Store latest joint state and log if recording."""
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
                    self.log_data["position_error"][i].append(
                        pos - self.target_positions[i]
                    )
                    self.log_data["target_position"][i].append(self.target_positions[i])

    def get_current_positions(self):
        """Get current joint positions."""
        if self.latest_joint_state is None:
            return None

        positions = []
        for joint_name in self.joint_names:
            if joint_name in self.latest_joint_state.name:
                idx = self.latest_joint_state.name.index(joint_name)
                positions.append(self.latest_joint_state.position[idx])

        return np.array(positions) if len(positions) == 7 else None

    def get_current_velocities(self):
        """Get current joint velocities."""
        if self.latest_joint_state is None:
            return None

        velocities = []
        for joint_name in self.joint_names:
            if joint_name in self.latest_joint_state.name:
                idx = self.latest_joint_state.name.index(joint_name)
                if self.latest_joint_state.velocity:
                    velocities.append(self.latest_joint_state.velocity[idx])
                else:
                    velocities.append(0.0)

        return np.array(velocities) if len(velocities) == 7 else None

    def wait_for_zero_velocity(self, velocity_threshold=0.005, timeout=10.0):
        """
        Wait until all joint velocities are below threshold.

        Returns:
            float: Time taken to reach zero velocity, or -1 if timeout
        """
        start_time = time.time()

        while time.time() - start_time < timeout:
            velocities = self.get_current_velocities()
            if velocities is None:
                time.sleep(0.01)
                continue

            max_velocity = np.max(np.abs(velocities))

            if max_velocity < velocity_threshold:
                settling_time = time.time() - start_time
                self.get_logger().info(
                    f"✓ Velocities settled in {settling_time:.3f}s (max: {max_velocity:.6f} rad/s)"
                )
                return settling_time

            time.sleep(0.01)

        # Timeout
        velocities = self.get_current_velocities()
        max_velocity = (
            np.max(np.abs(velocities)) if velocities is not None else float("inf")
        )
        self.get_logger().warn(
            f"⚠️ Timeout! Velocities did not settle (max: {max_velocity:.6f} rad/s)"
        )
        return -1.0

    def move_to_pose(self, q_target, duration=3.0):
        """Send trajectory goal and record performance."""
        self.get_logger().info(f"Moving to target: {np.degrees(q_target[:3])}° (J1-J3)")

        # Build trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = q_target.tolist()
        point.velocities = [0.0] * 7
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]

        # Set target for error tracking
        self.target_positions = q_target

        # Start recording
        self.recording = True
        self.start_time = time.time()
        self.log_data["time"].clear()
        for i in range(7):
            self.log_data["position"][i].clear()
            self.log_data["velocity"][i].clear()
            self.log_data["position_error"][i].clear()
            self.log_data["target_position"][i].clear()

        # Send goal
        future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.recording = False
            return False

        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration + 5.0
        )

        # Continue recording for settling
        self.get_logger().info("Trajectory complete, monitoring settling...")
        settling_time = self.wait_for_zero_velocity(
            velocity_threshold=0.005, timeout=5.0
        )

        # Stop recording after settling or timeout
        time.sleep(1.0)  # Capture steady-state
        self.recording = False

        return True

    def analyze_performance(self):
        """Analyze recorded data and compute metrics."""
        if not self.log_data["time"]:
            self.get_logger().error("No data recorded!")
            return None

        metrics = {}

        for joint_idx in range(7):
            joint_name = f"joint{joint_idx + 1}"

            # Extract data
            time_arr = np.array(self.log_data["time"])
            pos_err = np.array(self.log_data["position_error"][joint_idx])
            vel = np.array(self.log_data["velocity"][joint_idx])
            pos = np.array(self.log_data["position"][joint_idx])

            # Skip if no movement
            if np.max(np.abs(pos_err)) < 0.001:
                continue

            # Compute metrics
            max_error = np.max(np.abs(pos_err))
            rms_error = np.sqrt(np.mean(pos_err**2))

            # Steady-state error (last 10% of data)
            steady_start_idx = int(0.9 * len(pos_err))
            steady_state_error = np.mean(pos_err[steady_start_idx:])

            # Settling time (2% of max error)
            settling_threshold = 0.02 * max_error
            settled_indices = np.where(np.abs(pos_err) < settling_threshold)[0]
            if len(settled_indices) > 0:
                settling_idx = settled_indices[0]
                settling_time = time_arr[settling_idx]
            else:
                settling_time = time_arr[-1]

            # Velocity settling
            vel_settled = (
                np.max(np.abs(vel[-10:])) if len(vel) >= 10 else np.max(np.abs(vel))
            )

            metrics[joint_name] = {
                "max_error_rad": max_error,
                "max_error_deg": np.degrees(max_error),
                "rms_error_rad": rms_error,
                "rms_error_deg": np.degrees(rms_error),
                "steady_state_error_rad": steady_state_error,
                "steady_state_error_deg": np.degrees(steady_state_error),
                "settling_time_s": settling_time,
                "final_velocity_rad_s": vel_settled,
            }

        return metrics

    def plot_results(self, save_path=None):
        """Plot tracking performance."""
        if not self.log_data["time"]:
            self.get_logger().error("No data to plot!")
            return

        time_arr = np.array(self.log_data["time"])

        # Plot first 3 joints (most important for gravity compensation)
        fig, axes = plt.subplots(3, 3, figsize=(18, 12))
        fig.suptitle(
            "Trajectory Tracking Performance (Joints 1-3)",
            fontsize=16,
            fontweight="bold",
        )

        for col_idx, joint_idx in enumerate([0, 1, 2]):
            joint_name = f"Joint {joint_idx + 1}"

            # Position tracking
            ax = axes[0, col_idx]
            ax.plot(
                time_arr,
                self.log_data["target_position"][joint_idx],
                "k--",
                label="Target",
                linewidth=2,
            )
            ax.plot(
                time_arr,
                self.log_data["position"][joint_idx],
                "b-",
                label="Actual",
                linewidth=1.5,
            )
            ax.set_title(f"{joint_name} - Position")
            ax.set_ylabel("Position (rad)")
            ax.grid(True, alpha=0.3)
            ax.legend()

            # Position error
            ax = axes[1, col_idx]
            pos_err = np.array(self.log_data["position_error"][joint_idx])
            ax.plot(time_arr, np.degrees(pos_err), "r-", linewidth=1.5)
            ax.axhline(y=0, color="k", linestyle=":", linewidth=0.8)
            ax.set_title(f"{joint_name} - Position Error")
            ax.set_ylabel("Error (deg)")
            ax.grid(True, alpha=0.3)

            # Velocity
            ax = axes[2, col_idx]
            ax.plot(time_arr, self.log_data["velocity"][joint_idx], "g-", linewidth=1.5)
            ax.axhline(y=0, color="k", linestyle=":", linewidth=0.8)
            ax.axhline(
                y=0.005,
                color="r",
                linestyle="--",
                linewidth=0.8,
                label="Threshold ±0.005",
            )
            ax.axhline(y=-0.005, color="r", linestyle="--", linewidth=0.8)
            ax.set_title(f"{joint_name} - Velocity")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Velocity (rad/s)")
            ax.grid(True, alpha=0.3)
            ax.legend()

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150)
            self.get_logger().info(f"Plot saved to: {save_path}")

        plt.close(fig)  # Clean up to avoid memory leaks

    def run_test_sequence(self):
        """Run test sequence to evaluate PID gains."""
        self.get_logger().info("=" * 80)
        self.get_logger().info("TRAJECTORY TRACKING TEST - PID GAIN EVALUATION")
        self.get_logger().info("=" * 80)

        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        while self.latest_joint_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        # Wait for action server
        self.get_logger().info("Waiting for trajectory action server...")
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return False

        # Test configurations - focus on joints with gravity load
        test_configs = [
            ("J2 = -60° (shoulder loaded)", np.array([0, -np.pi / 3, 0, 0, 0, 0, 0])),
            ("J2 = -90° (horizontal)", np.array([0, -np.pi / 2, 0, 0, 0, 0, 0])),
            ("J3 = +60° (elbow flexed)", np.array([0, 0, np.pi / 3, 0, 0, 0, 0])),
        ]

        all_metrics = {}

        for config_name, q_target in test_configs:
            self.get_logger().info(f"\n{'=' * 60}")
            self.get_logger().info(f"Test: {config_name}")
            self.get_logger().info(f"{'=' * 60}")

            # Execute trajectory
            if not self.move_to_pose(q_target, duration=3.0):
                self.get_logger().error(
                    f"Failed to execute trajectory for {config_name}"
                )
                continue

            # Analyze performance
            metrics = self.analyze_performance()
            if metrics:
                all_metrics[config_name] = metrics

                self.get_logger().info(f"\n📊 Performance Metrics for {config_name}:")
                for joint_name, m in metrics.items():
                    self.get_logger().info(f"\n  {joint_name}:")
                    self.get_logger().info(
                        f"    Max error:         {m['max_error_deg']:.3f}°"
                    )
                    self.get_logger().info(
                        f"    RMS error:         {m['rms_error_deg']:.3f}°"
                    )
                    self.get_logger().info(
                        f"    Steady-state err:  {m['steady_state_error_deg']:.3f}°"
                    )
                    self.get_logger().info(
                        f"    Settling time:     {m['settling_time_s']:.3f} s"
                    )
                    self.get_logger().info(
                        f"    Final velocity:    {m['final_velocity_rad_s']:.6f} rad/s"
                    )

                # Plot results
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                plot_path = (
                    f"trajectory_test_{config_name.replace(' ', '_')}_{timestamp}.png"
                )
                self.plot_results(save_path=plot_path)

            # Pause between tests
            time.sleep(2.0)

        # Summary
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("TEST COMPLETE - TUNING RECOMMENDATIONS")
        self.get_logger().info("=" * 80)

        # Analyze all metrics to give tuning advice
        self._print_tuning_recommendations(all_metrics)

        return True

    def _print_tuning_recommendations(self, all_metrics):
        """Analyze metrics and suggest gain adjustments."""
        if not all_metrics:
            return

        self.get_logger().info("\n🔧 TUNING RECOMMENDATIONS:")

        # Aggregate metrics across tests
        for joint_idx in range(1, 4):  # Focus on joints 1-3
            joint_name = f"joint{joint_idx}"

            # Collect metrics for this joint across all tests
            max_errors = []
            steady_errors = []
            settling_times = []
            final_vels = []

            for test_name, metrics in all_metrics.items():
                if joint_name in metrics:
                    m = metrics[joint_name]
                    max_errors.append(m["max_error_deg"])
                    steady_errors.append(abs(m["steady_state_error_deg"]))
                    settling_times.append(m["settling_time_s"])
                    final_vels.append(abs(m["final_velocity_rad_s"]))

            if not max_errors:
                continue

            avg_max_error = np.mean(max_errors)
            avg_steady_error = np.mean(steady_errors)
            avg_settling_time = np.mean(settling_times)
            avg_final_vel = np.mean(final_vels)

            self.get_logger().info(f"\n  Joint {joint_idx}:")
            self.get_logger().info(f"    Avg max error:    {avg_max_error:.3f}°")
            self.get_logger().info(f"    Avg steady error: {avg_steady_error:.3f}°")
            self.get_logger().info(f"    Avg settling:     {avg_settling_time:.3f} s")
            self.get_logger().info(f"    Avg final vel:    {avg_final_vel:.6f} rad/s")

            # Recommendations
            if avg_steady_error > 0.5:
                self.get_logger().info("    → INCREASE Kp (reduce steady-state error)")
            if avg_final_vel > 0.01:
                self.get_logger().info("    → INCREASE Kd (improve velocity damping)")
            if avg_settling_time > 2.0:
                self.get_logger().info("    → INCREASE Kp and Kd (faster settling)")
            if avg_max_error < 0.5 and avg_steady_error < 0.2 and avg_final_vel < 0.005:
                self.get_logger().info("    ✓ Gains are well-tuned!")

        self.get_logger().info("\n📝 To update gains, edit:")
        self.get_logger().info(
            "    urdf/ros2_control/openarm.ros2_control.xacro (lines 91-97)"
        )
        self.get_logger().info("\nThen rebuild:")
        self.get_logger().info("    cd ~/ros2_ws")
        self.get_logger().info("    colcon build --packages-select openarm_description")
        self.get_logger().info("    source install/setup.bash")


def main():
    """Run the trajectory tracking tester."""
    rclpy.init()

    try:
        tester = TrajectoryTrackingTester()
        success = tester.run_test_sequence()
        return 0 if success else 1

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        return 1

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    exit(main())
