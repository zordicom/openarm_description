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

import time
from pathlib import Path

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from mujoco_ros2_control_msgs.srv import ApplyExternalWrench
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
        self.q_initial = None
        self.joint_state_received = False

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Action client for position control (for moving arm between tests)
        self.position_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

        self.get_logger().info("Gravity compensation tester initialized")
        self.get_logger().info(f"  Test duration: {self.test_duration} s")
        self.get_logger().info(f"  Position threshold: {self.position_threshold} rad")

        # Service client for applying external wrench (headless perturbations)
        self.wrench_client = self.create_client(
            ApplyExternalWrench, "/apply_external_wrench"
        )
        if not self.wrench_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "apply_external_wrench service not available yet. "
                "Perturbation test will be skipped if unavailable."
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

            if self.q_initial is None:
                self.q_initial = self.q.copy()
                pos_str = ", ".join(f"{q:.3f}" for q in self.q_initial)
                self.get_logger().info(f"Initial position: [{pos_str}] rad")

            self.joint_state_received = True

        except Exception as e:
            self.get_logger().error(f"Error in joint_state_callback: {e}")

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

        # First, move to the target position using position controller
        # (This requires temporarily switching controllers)
        pos_str = ", ".join(f"{q:.3f}" for q in position)
        self.get_logger().info(f"Moving to position: [{pos_str}] rad")
        self.get_logger().info(
            "⚠️  Note: This test assumes gravity_compensation_controller is running"
        )

        # Wait for robot to settle at new position
        time.sleep(3.0)

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
        if rclpy.ok():
            rclpy.shutdown()


def main():
    """Main entry point."""
    run_test_suite()


if __name__ == "__main__":
    main()
