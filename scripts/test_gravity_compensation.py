#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Test script to verify gravity compensation in MuJoCo simulation.

This script:
1. Launches the gravity compensation controller
2. Monitors joint positions to verify the robot holds steady
3. Optionally applies small perturbations to test stability

Usage:
    # Terminal 1: Start simulation with effort control
    export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
    ros2 launch openarm_description mujoco_sim.launch.py control_mode:=effort

    # Terminal 2: Run this test
    python3 scripts/test_gravity_compensation.py
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GravityCompensationTester(Node):
    """Test node for gravity compensation."""

    def __init__(self):
        """Initialize the tester node."""
        super().__init__("gravity_compensation_tester")

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
        self.q_initial = None
        self.joint_state_received = False

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Publisher for effort commands (for perturbation test)
        self.effort_pub = self.create_publisher(
            Float64MultiArray, "/effort_controller/commands", 10
        )

        self.get_logger().info("Gravity compensation tester initialized")

    def joint_state_callback(self, msg: JointState):
        """Update joint positions from joint_states topic."""
        try:
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.q[i] = msg.position[idx]

            if self.q_initial is None:
                self.q_initial = self.q.copy()
                self.get_logger().info(
                    f"Initial position: [{', '.join(f'{q:.3f}' for q in self.q_initial)}] rad"
                )

            self.joint_state_received = True

        except Exception as e:
            self.get_logger().error(f"Error in joint_state_callback: {e}")

    def check_stability(self, duration=10.0, threshold=0.05):
        """Check if robot holds position within threshold for given duration.

        Args:
            duration: Time to monitor (seconds)
            threshold: Maximum allowed position drift (radians)

        Returns:
            bool: True if stable, False otherwise
        """
        if not self.joint_state_received:
            self.get_logger().error("No joint states received yet!")
            return False

        self.get_logger().info(
            f"Monitoring stability for {duration} seconds (threshold: {threshold} rad)..."
        )

        start_time = time.time()
        max_error = np.zeros(7)
        samples = []

        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.joint_state_received:
                error = np.abs(self.q - self.q_initial)
                max_error = np.maximum(max_error, error)
                samples.append(self.q.copy())

            time.sleep(0.01)

        # Compute statistics
        samples = np.array(samples)
        mean_pos = np.mean(samples, axis=0)
        std_pos = np.std(samples, axis=0)

        self.get_logger().info("\n=== Stability Test Results ===")
        self.get_logger().info(
            f"Initial:  [{', '.join(f'{q:7.3f}' for q in self.q_initial)}] rad"
        )
        self.get_logger().info(
            f"Mean:     [{', '.join(f'{q:7.3f}' for q in mean_pos)}] rad"
        )
        self.get_logger().info(
            f"Std Dev:  [{', '.join(f'{s:7.4f}' for s in std_pos)}] rad"
        )
        self.get_logger().info(
            f"Max Err:  [{', '.join(f'{e:7.4f}' for e in max_error)}] rad"
        )

        # Check if all joints stayed within threshold
        stable = np.all(max_error < threshold)

        if stable:
            self.get_logger().info(
                f"✅ PASS: Robot held position within {threshold} rad"
            )
        else:
            failed_joints = [
                self.joint_names[i] for i in range(7) if max_error[i] >= threshold
            ]
            self.get_logger().error(
                f"❌ FAIL: Joints exceeded threshold: {failed_joints}"
            )

        return stable

    def apply_perturbation(self, torques, duration=0.5):
        """Apply a torque perturbation to test recovery.

        Args:
            torques: Array of 7 torques to apply (Nm)
            duration: How long to apply perturbation (seconds)
        """
        self.get_logger().info(
            f"Applying perturbation: [{', '.join(f'{t:.1f}' for t in torques)}] Nm for {duration}s"
        )

        msg = Float64MultiArray()
        msg.data = torques.tolist()

        start_time = time.time()
        while time.time() - start_time < duration:
            self.effort_pub.publish(msg)
            time.sleep(0.01)

        self.get_logger().info("Perturbation complete, monitoring recovery...")


def main():
    """Run gravity compensation tests."""
    rclpy.init()

    try:
        tester = GravityCompensationTester()

        # Wait for joint states
        tester.get_logger().info("Waiting for joint states...")
        while not tester.joint_state_received:
            rclpy.spin_once(tester, timeout_sec=0.1)
            time.sleep(0.1)

        time.sleep(1.0)  # Let things settle

        # Test 1: Basic stability test
        tester.get_logger().info("\n=== Test 1: Basic Stability (10 seconds) ===")
        stable = tester.check_stability(duration=10.0, threshold=0.05)

        if not stable:
            tester.get_logger().error(
                "Basic stability test failed! Check gravity compensation gains."
            )
            return

        # Test 2: Extended stability test
        tester.get_logger().info("\n=== Test 2: Extended Stability (30 seconds) ===")
        stable = tester.check_stability(duration=30.0, threshold=0.1)

        if not stable:
            tester.get_logger().warn(
                "Extended stability test failed. May need damping tuning."
            )

        # Test 3: Perturbation recovery (optional)
        tester.get_logger().info("\n=== Test 3: Perturbation Recovery ===")

        # Apply small torque to joint 1
        perturbation = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        tester.apply_perturbation(perturbation, duration=0.5)

        time.sleep(2.0)  # Wait for recovery

        # Check if robot returned to initial position
        recovery_stable = tester.check_stability(duration=5.0, threshold=0.1)

        if recovery_stable:
            tester.get_logger().info("✅ Robot recovered from perturbation")
        else:
            tester.get_logger().warn("⚠️  Robot did not fully recover")

        tester.get_logger().info("\n=== All tests complete! ===")

    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()

    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

