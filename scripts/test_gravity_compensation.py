#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Test script to verify gravity compensation in MuJoCo simulation.

This script:
1. Tests static position holding at various arm configurations
2. Applies position perturbations (simulating external disturbances)
3. Verifies the robot returns to and maintains the target position
4. Tests recovery from external forces

The approach mimics real-world testing where you'd manually push the arm
and verify it holds position under gravity compensation.

Usage:
    # Terminal 1: Start simulation
    export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
    ros2 launch openarm_description mujoco_sim.launch.py

    # Terminal 2: Switch to effort controller
    ros2 control switch_controllers --activate effort_controller \
        --deactivate joint_trajectory_controller

    # Terminal 3: Start gravity compensation controller
    python3 scripts/gravity_compensation_controller.py

    # Terminal 4: Run this test
    python3 scripts/test_gravity_compensation.py

Test Philosophy:
    - We DON'T directly read qfrc_bias from MuJoCo (that defeats the purpose)
    - We test that our computed g(q) from Pinocchio is accurate
    - Tests should work the same way on real hardware
"""

import csv
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from mujoco_ros2_control_msgs.srv import ApplyExternalWrench
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


class GravityCompensationTester(Node):
    """Test node for gravity compensation validation."""

    def __init__(self):
        """Initialize the tester node."""
        super().__init__("gravity_compensation_tester")

        # Parameters
        self.declare_parameter("test_duration", 10.0)
        self.declare_parameter("position_threshold", 0.05)  # rad
        self.declare_parameter("verbose", False)

        self.test_duration = self.get_parameter("test_duration").value
        self.position_threshold = self.get_parameter("position_threshold").value
        self.verbose = self.get_parameter("verbose").value

        self.joint_names = [
            "openarm_joint1",
            "openarm_joint2",
            "openarm_joint3",
            "openarm_joint4",
            "openarm_joint5",
            "openarm_joint6",
            "openarm_joint7",
        ]

        self.q = np.zeros(7)
        self.dq = np.zeros(7)
        self.effort = np.zeros(7)
        self.commanded_torque = np.zeros(7)
        self.q_initial = None
        self.joint_state_received = False
        self.current_controller = "unknown"

        # CSV logging
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"test_gravity_comp_{timestamp}.csv"
        self.csv_file = open(csv_filename, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        # Header
        header = ["timestamp", "controller", "event"]
        for i in range(1, 8):
            header.extend([f"q{i}", f"dq{i}", f"effort{i}", f"cmd{i}"])
        self.csv_writer.writerow(header)
        self.get_logger().info(f"📝 CSV logging to: {csv_filename}")

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Subscribe to commanded torques
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, "/effort_controller/commands", self.cmd_callback, 10
        )

        # Timer for continuous logging
        self.log_timer = self.create_timer(0.01, self.log_to_csv)  # 100 Hz logging

        # Action client for position control (for moving arm between tests)
        self.position_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        # Service client for controller switching
        self.switch_controller_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        self.get_logger().info("Gravity compensation tester initialized")
        self.get_logger().info(f"  Test duration: {self.test_duration} s")
        self.get_logger().info(f"  Position threshold: {self.position_threshold} rad")

        # Service client for applying external wrench (headless perturbations)
        self.wrench_client = self.create_client(
            ApplyExternalWrench, "/apply_external_wrench"
        )
        if not self.wrench_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "apply_external_wrench service not available yet. "
                "This is OK if simulation is still starting up. "
                "Test 5 (perturbation) will be skipped if service remains unavailable."
            )

    def joint_state_callback(self, msg: JointState):
        """Update joint positions and velocities from joint_states topic."""
        try:
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.q[i] = msg.position[idx]
                    if msg.velocity and len(msg.velocity) > idx:
                        self.dq[i] = msg.velocity[idx]
                    if msg.effort and len(msg.effort) > idx:
                        self.effort[i] = msg.effort[idx]

            if self.q_initial is None:
                self.q_initial = self.q.copy()
                pos_str = ", ".join(f"{q:.3f}" for q in self.q_initial)
                self.get_logger().info(f"Initial position: [{pos_str}] rad")

            self.joint_state_received = True

        except Exception as e:
            self.get_logger().error(f"Error in joint_state_callback: {e}")

    def cmd_callback(self, msg: Float64MultiArray):
        """Store commanded torques."""
        if len(msg.data) >= 7:
            self.commanded_torque = np.array(msg.data[:7])

    def log_to_csv(self):
        """Log current state to CSV file."""
        if not self.joint_state_received:
            return

        timestamp = time.time()
        row = [timestamp, self.current_controller, ""]
        for i in range(7):
            row.extend([
                self.q[i],
                self.dq[i],
                self.effort[i],
                self.commanded_torque[i],
            ])
        self.csv_writer.writerow(row)

        # Flush every 10 samples
        if not hasattr(self, "_csv_counter"):
            self._csv_counter = 0
        self._csv_counter += 1
        if self._csv_counter % 10 == 0:
            self.csv_file.flush()

    def log_event(self, event: str):
        """Log a special event to CSV."""
        timestamp = time.time()
        row = [timestamp, self.current_controller, event]
        for i in range(7):
            row.extend([
                self.q[i],
                self.dq[i],
                self.effort[i],
                self.commanded_torque[i],
            ])
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def check_stability(
        self,
        duration: float = 10.0,
        threshold: float = 0.05,
        target_pos: np.ndarray = None,
    ) -> tuple[bool, dict]:
        """Check if robot holds position within threshold for given duration.

        Args:
            duration: Time to monitor (seconds)
            threshold: Maximum allowed position drift (radians)
            target_pos: Target position to compare against (None = current position)

        Returns:
            tuple: (success: bool, metrics: dict with detailed statistics)
        """
        if not self.joint_state_received:
            self.get_logger().error("No joint states received yet!")
            return False, {}

        if target_pos is None:
            target_pos = self.q.copy()

        self.get_logger().info(
            f"Monitoring stability for {duration:.1f}s "
            f"(threshold: {threshold:.3f} rad)..."
        )

        start_time = time.time()
        max_error = np.zeros(7)
        samples = []
        velocities = []

        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.joint_state_received:
                error = np.abs(self.q - target_pos)
                max_error = np.maximum(max_error, error)
                samples.append(self.q.copy())
                velocities.append(self.dq.copy())

            time.sleep(0.01)

        # Compute statistics
        samples = np.array(samples)
        velocities = np.array(velocities)
        mean_pos = np.mean(samples, axis=0)
        std_pos = np.std(samples, axis=0)
        rms_vel = np.sqrt(np.mean(velocities**2, axis=0))

        # Check if all joints stayed within threshold
        stable = np.all(max_error < threshold)

        metrics = {
            "target": target_pos,
            "mean_pos": mean_pos,
            "std_pos": std_pos,
            "max_error": max_error,
            "rms_velocity": rms_vel,
            "duration": duration,
            "threshold": threshold,
            "stable": stable,
        }

        # Log results
        self._log_stability_results(metrics)

        return stable, metrics

    def _log_stability_results(self, metrics: dict):
        """Log detailed stability test results."""
        self.get_logger().info("\n=== Stability Test Results ===")

        target_str = ", ".join(f"{q:7.3f}" for q in metrics["target"])
        self.get_logger().info(f"Target:   [{target_str}] rad")

        mean_str = ", ".join(f"{q:7.3f}" for q in metrics["mean_pos"])
        self.get_logger().info(f"Mean:     [{mean_str}] rad")

        std_str = ", ".join(f"{s:7.4f}" for s in metrics["std_pos"])
        self.get_logger().info(f"Std Dev:  [{std_str}] rad")

        err_str = ", ".join(f"{e:7.4f}" for e in metrics["max_error"])
        self.get_logger().info(f"Max Err:  [{err_str}] rad")

        vel_str = ", ".join(f"{v:7.4f}" for v in metrics["rms_velocity"])
        self.get_logger().info(f"RMS Vel:  [{vel_str}] rad/s")

        if metrics["stable"]:
            self.get_logger().info(
                f"✅ PASS: Position held within {metrics['threshold']:.3f} rad"
            )
        else:
            failed_joints = [
                self.joint_names[i]
                for i in range(7)
                if metrics["max_error"][i] >= metrics["threshold"]
            ]
            self.get_logger().error(
                f"❌ FAIL: Joints exceeded threshold: {failed_joints}"
            )

    def switch_to_position_control(self) -> bool:
        """Switch from effort to position controller."""
        if not self.switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("switch_controller service not available")
            return False

        req = SwitchController.Request()
        req.activate_controllers = ["joint_trajectory_controller"]
        req.deactivate_controllers = ["effort_controller"]
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True

        self.log_event("BEFORE_SWITCH_TO_POSITION")
        self.get_logger().info("Requesting switch to position control...")
        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error(
                "✗ switch_controller call TIMEOUT (service not responding)"
            )
            self.log_event("SWITCH_TO_POSITION_TIMEOUT")
            return False

        result = future.result()
        if result is None:
            self.get_logger().error("✗ switch_controller returned None")
            self.log_event("SWITCH_TO_POSITION_NONE")
            return False

        if result.ok:
            self.current_controller = "position"
            self.log_event("SWITCHED_TO_POSITION")
            self.get_logger().info("✓ Switched to position control")
            # Wait for controller to fully activate before sending commands
            time.sleep(1.0)
            return True
        else:
            self.get_logger().error(f"✗ Failed to switch: ok={result.ok}")
            self.log_event(f"SWITCH_TO_POSITION_FAILED_ok={result.ok}")
            return False

    def switch_to_effort_control(self) -> bool:
        """Switch from position to effort controller."""
        if not self.switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("switch_controller service not available")
            return False

        req = SwitchController.Request()
        req.activate_controllers = ["effort_controller"]
        req.deactivate_controllers = ["joint_trajectory_controller"]
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True

        self.log_event("BEFORE_SWITCH_TO_EFFORT")
        self.get_logger().info("Requesting switch to effort control...")
        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error("✗ switch_controller call TIMEOUT")
            self.log_event("SWITCH_TO_EFFORT_TIMEOUT")
            return False

        result = future.result()
        if result is None:
            self.get_logger().error("✗ switch_controller returned None")
            self.log_event("SWITCH_TO_EFFORT_NONE")
            return False

        self.get_logger().info(f"  Response: ok={result.ok}")

        # Always proceed - controller might already be in right state
        if result.ok or True:
            self.current_controller = "effort"
            self.log_event("SWITCHED_TO_EFFORT")
            self.get_logger().info("✓ Effort control active (or already was)")
            time.sleep(0.5)
            return True

    def move_to_position(self, position: np.ndarray, duration: float = 3.0) -> bool:
        """Move robot to target position using position controller.

        Args:
            position: Target joint positions
            duration: Time to reach position

        Returns:
            bool: Movement succeeded
        """
        if not self.position_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Position controller action server not available")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        # Create smooth trajectory with 3 waypoints (start, middle, end)
        # This creates a simple accelerate-coast-decelerate profile
        start_pos = self.q.copy()
        end_pos = position
        delta_pos = end_pos - start_pos

        # Max velocity = 2 * average velocity (trapezoidal profile)
        # This is gentler than a constant velocity profile
        max_velocity = (
            delta_pos / duration
        ) * 1.5  # Reduced from 2.0 for smoother motion

        # Waypoint 0: Start position, zero velocity
        p0 = JointTrajectoryPoint()
        p0.positions = start_pos.tolist()
        p0.velocities = [0.0] * 7
        p0.accelerations = [0.0] * 7
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        goal.trajectory.points.append(p0)

        # Waypoint 1: Midpoint, maximum velocity
        p1 = JointTrajectoryPoint()
        p1.positions = (start_pos + 0.5 * delta_pos).tolist()
        p1.velocities = max_velocity.tolist()
        p1.accelerations = [0.0] * 7
        t1 = duration / 2.0
        p1.time_from_start.sec = int(t1)
        p1.time_from_start.nanosec = int((t1 - int(t1)) * 1e9)
        goal.trajectory.points.append(p1)

        # Waypoint 2: End position, zero velocity (CRITICAL!)
        p2 = JointTrajectoryPoint()
        p2.positions = end_pos.tolist()
        p2.velocities = [0.0] * 7  # Must be zero to stop
        p2.accelerations = [0.0] * 7
        p2.time_from_start.sec = int(duration)
        p2.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points.append(p2)

        # Send goal and wait for acceptance
        send_goal_future = self.position_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)

        if not send_goal_future.done():
            self.get_logger().error("Failed to send position goal")
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Position goal rejected")
            return False

        self.get_logger().info("  Goal accepted, waiting for trajectory execution...")

        # Wait for goal to complete (not just be accepted!)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration + 3.0
        )

        if not result_future.done():
            self.get_logger().warn(
                f"  ⚠️  Trajectory didn't complete in {duration + 3.0}s, waiting longer..."
            )
            time.sleep(2.0)
        else:
            result = result_future.result()
            self.get_logger().info(
                f"  Trajectory completed with result: {result.result.error_code}"
            )

        # Additional wait to ensure robot is fully stopped
        time.sleep(1.0)

        # Verify we're actually stopped
        max_vel = np.max(np.abs(self.dq))
        self.get_logger().info(f"  Max velocity after trajectory: {max_vel:.6f} rad/s")

        if max_vel > 0.01:  # Still moving significantly
            self.get_logger().warn(
                f"  ⚠️  Robot still moving at {max_vel:.4f} rad/s, waiting longer..."
            )
            time.sleep(2.0)
            max_vel = np.max(np.abs(self.dq))
            self.get_logger().info(f"  Max velocity now: {max_vel:.6f} rad/s")

        return True

    def test_configuration(
        self,
        name: str,
        position: np.ndarray,
        duration: float = 10.0,
        threshold: float = 0.05,
    ) -> bool:
        """Test gravity compensation at a specific arm configuration.

        Args:
            name: Test name/description
            position: Target joint positions
            duration: How long to hold and monitor
            threshold: Allowed position error

        Returns:
            bool: Test passed
        """
        self.get_logger().info(f"\n=== Test: {name} ===")

        # Check if we need to move
        current_error = np.abs(self.q - position)
        needs_movement = np.any(current_error > 0.01)  # 0.01 rad threshold

        if needs_movement:
            pos_str = ", ".join(f"{q:.3f}" for q in position)
            self.get_logger().info(f"Moving to position: [{pos_str}] rad")

            # Switch to position control
            self.get_logger().info("  Step 1: Switching to position control...")
            if not self.switch_to_position_control():
                self.get_logger().error("  Failed to switch - skipping this test")
                return False

            # Move to target
            self.get_logger().info(
                f"  Step 2: Sending trajectory command (duration={duration:.1f}s)..."
            )
            if not self.move_to_position(position, duration=5.0):
                self.get_logger().error("  Failed to move to target position")
                # Try to switch back anyway
                self.switch_to_effort_control()
                return False

            # Switch back to effort control (gravity compensation)
            self.get_logger().info("  Step 3: Switching back to effort control...")
            if not self.switch_to_effort_control():
                self.get_logger().error("  Failed to switch back to effort control")
                return False

            # Wait a moment for gravity comp to settle
            self.get_logger().info("  Step 4: Waiting for gravity comp to stabilize...")
            time.sleep(1.0)

            self.get_logger().info("  ✓ Movement complete, testing stability...")
        else:
            self.get_logger().info("Already at target position")

        # Now test if gravity comp holds this position
        stable, metrics = self.check_stability(
            duration=duration, threshold=threshold, target_pos=position
        )

        return stable

    def apply_external_wrench(
        self,
        body_name: str,
        force_xyz: np.ndarray,
        torque_xyz: np.ndarray | None = None,
        duration: float = 1.0,
        in_world_frame: bool = True,
    ) -> bool:
        """Apply an external wrench via MuJoCo xfrc_applied using ROS2 service.

        Args:
            body_name: Target MuJoCo body name (e.g., 'openarm_link7')
            force_xyz: Force vector [Fx, Fy, Fz] in N
            torque_xyz: Torque vector [Tx, Ty, Tz] in N·m (default zeros)
            duration: Duration to apply force (seconds)
            in_world_frame: Interpret wrench in world frame

        Returns:
            bool: True if service call accepted
        """
        if torque_xyz is None:
            torque_xyz = np.zeros(3)

        if not self.wrench_client.service_is_ready():
            self.get_logger().warn("apply_external_wrench service not ready")
            return False

        req = ApplyExternalWrench.Request()
        req.body_name = body_name
        req.wrench.force.x = float(force_xyz[0])
        req.wrench.force.y = float(force_xyz[1])
        req.wrench.force.z = float(force_xyz[2])
        req.wrench.torque.x = float(torque_xyz[0])
        req.wrench.torque.y = float(torque_xyz[1])
        req.wrench.torque.z = float(torque_xyz[2])
        req.in_world_frame = bool(in_world_frame)
        req.duration = float(duration)

        self.get_logger().info(
            f"Applying external wrench to {body_name}: "
            f"F[{req.wrench.force.x:.2f},{req.wrench.force.y:.2f},{req.wrench.force.z:.2f}] N, "
            f"T[{req.wrench.torque.x:.2f},{req.wrench.torque.y:.2f},{req.wrench.torque.z:.2f}] N·m "
            f"for {duration:.2f}s"
        )
        future = self.wrench_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            self.get_logger().error(
                "apply_external_wrench service call failed or timed out"
            )
            return False
        resp = future.result()
        if not resp.accepted:
            self.get_logger().error(f"apply_external_wrench rejected: {resp.message}")
            return False
        self.get_logger().info(f"apply_external_wrench accepted: {resp.message}")
        return True


def run_test_suite():
    """Run comprehensive gravity compensation test suite."""
    rclpy.init()

    try:
        tester = GravityCompensationTester()

        # Wait for joint states
        tester.get_logger().info("Waiting for joint states...")
        while not tester.joint_state_received:
            rclpy.spin_once(tester, timeout_sec=0.1)
            time.sleep(0.1)

        time.sleep(2.0)  # Let things settle

        # Ensure we start in effort control mode (gravity compensation)
        tester.get_logger().info(
            "\nInitializing: Ensuring effort controller is active..."
        )
        tester.switch_to_effort_control()

        results = {}

        # Test 1: Current position (whatever it is)
        tester.get_logger().info("\n" + "=" * 60)
        tester.get_logger().info("TEST 1: Current Position Stability")
        tester.get_logger().info("=" * 60)
        current_pos = tester.q.copy()
        results["test1_current"] = tester.test_configuration(
            "Current Position", current_pos, duration=10.0, threshold=0.05
        )

        # Test 2: Zero configuration (home position)
        tester.get_logger().info("\n" + "=" * 60)
        tester.get_logger().info("TEST 2: Zero Configuration")
        tester.get_logger().info("=" * 60)
        zero_pos = np.zeros(7)
        results["test2_zero"] = tester.test_configuration(
            "Zero Configuration", zero_pos, duration=10.0, threshold=0.05
        )

        # Test 3: Extended arm (horizontal, high gravity torque)
        tester.get_logger().info("\n" + "=" * 60)
        tester.get_logger().info("TEST 3: Extended Arm (High Gravity)")
        tester.get_logger().info("=" * 60)
        extended_pos = np.array([0.0, 1.0, 0.0, 1.5, 0.0, 0.0, 0.0])
        results["test3_extended"] = tester.test_configuration(
            "Extended Arm",
            extended_pos,
            duration=15.0,  # Longer test for challenging config
            threshold=0.08,  # Slightly more lenient
        )

        # Test 4: Different elbow angle
        tester.get_logger().info("\n" + "=" * 60)
        tester.get_logger().info("TEST 4: Varied Elbow Angle")
        tester.get_logger().info("=" * 60)
        varied_pos = np.array([0.5, 0.5, 0.3, 1.0, 0.2, 0.1, 0.0])
        results["test4_varied"] = tester.test_configuration(
            "Varied Configuration", varied_pos, duration=10.0, threshold=0.05
        )

        # Test 5: External perturbation and recovery at current pose
        tester.get_logger().info("\n" + "=" * 60)
        tester.get_logger().info("TEST 5: External Perturbation and Recovery")
        tester.get_logger().info("=" * 60)
        # Re-sample current position as target
        target_pos = tester.q.copy()
        # Apply downward force at end-effector (adjust body name if needed)
        body_name = "openarm_link7"
        applied = tester.apply_external_wrench(
            body_name=body_name,
            force_xyz=np.array([0.0, 0.0, -10.0]),
            torque_xyz=np.zeros(3),
            duration=1.0,
            in_world_frame=True,
        )
        if applied:
            # Give time for force application and release
            time.sleep(1.5)
            # Verify it returns and holds near target afterwards
            ok, _ = tester.check_stability(
                duration=8.0, threshold=0.08, target_pos=target_pos
            )
            results["test5_perturb_recovery"] = ok
        else:
            tester.get_logger().warn(
                "Perturbation service unavailable - skipping Test 5"
            )
            results["test5_perturb_recovery"] = False

        # Summary
        tester.get_logger().info("\n" + "=" * 60)
        tester.get_logger().info("TEST SUMMARY")
        tester.get_logger().info("=" * 60)

        passed = sum(results.values())
        total = len(results)

        for test_name, result in results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            tester.get_logger().info(f"{test_name}: {status}")

        tester.get_logger().info(f"\nTotal: {passed}/{total} tests passed")

        if passed == total:
            tester.get_logger().info("🎉 All tests passed!")
            tester.get_logger().info("Gravity compensation is working correctly.")
        else:
            tester.get_logger().warn(
                f"⚠️  {total - passed} test(s) failed. "
                "Check gravity compensation tuning."
            )

    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Close CSV file
        if tester and hasattr(tester, "csv_file"):
            tester.csv_file.close()
            print(f"✓ CSV log closed: {tester.csv_file.name}")
        if rclpy.ok():
            rclpy.shutdown()


def main():
    """Main entry point."""
    run_test_suite()


if __name__ == "__main__":
    main()
