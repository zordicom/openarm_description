#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Verification script for automated test launch files.
Reads joint states and verifies against expected positions.

Usage:
  verify_test_result.py --test-name NAME --expected "p1,p2,..." --tolerance TOL
  verify_test_result.py --action-result RESULT  # 0=success, nonzero=fail
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateChecker(Node):
    """Node to check joint states against expected values."""

    def __init__(self):
        """Initialize the checker node."""
        super().__init__("joint_state_checker")
        self.joint_positions = None
        self.joint_names = None
        self.sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )

    def _joint_state_cb(self, msg: JointState):
        """Store received joint states."""
        self.joint_names = list(msg.name)
        self.joint_positions = list(msg.position)

    def wait_for_joint_states(self, timeout: float = 5.0) -> bool:
        """Wait for joint states to be received."""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_positions is not None:
                return True
        return False


def print_banner(text: str, char: str = "="):
    """Print a banner with the given text."""
    width = max(60, len(text) + 4)
    print(char * width)
    print(f"  {text}")
    print(char * width)


def verify_action_result(result: int, test_name: str) -> int:
    """Verify result from ros2 action send_goal."""
    print()
    if result == 0:
        print_banner(f"TEST PASSED: {test_name}", "=")
        print("  Action completed successfully")
        return 0
    else:
        print_banner(f"TEST FAILED: {test_name}", "!")
        print(f"  Action failed with exit code: {result}")
        return 1


def verify_joint_positions(
    test_name: str,
    expected: list[float],
    tolerance: float,
    joint_order: list[str] | None = None,
) -> int:
    """Verify current joint positions against expected values."""
    rclpy.init()
    node = JointStateChecker()

    try:
        if not node.wait_for_joint_states(timeout=10.0):
            print()
            print_banner(f"TEST FAILED: {test_name}", "!")
            print("  Could not read joint states within timeout")
            return 1

        actual = node.joint_positions
        names = node.joint_names

        # Reorder if joint_order specified
        if joint_order:
            reordered = []
            for name in joint_order:
                if name in names:
                    idx = names.index(name)
                    reordered.append(actual[idx])
                else:
                    reordered.append(0.0)
            actual = reordered
            names = joint_order

        # Compare
        errors = [abs(a - e) for a, e in zip(actual, expected)]
        max_error = max(errors)
        max_error_idx = errors.index(max_error)

        print()
        print(f"Expected: {[f'{e:.4f}' for e in expected]}")
        print(f"Actual:   {[f'{a:.4f}' for a in actual]}")
        print(f"Errors:   {[f'{e:.4f}' for e in errors]}")
        print(f"Max error: {max_error:.4f} rad at joint {max_error_idx + 1}")
        print(f"Tolerance: {tolerance:.4f} rad")
        print()

        if max_error <= tolerance:
            print_banner(f"TEST PASSED: {test_name}", "=")
            return 0
        else:
            print_banner(f"TEST FAILED: {test_name}", "!")
            print(f"  Max error {max_error:.4f} exceeds tolerance {tolerance:.4f}")
            return 1

    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    """Parse arguments and run verification."""
    parser = argparse.ArgumentParser(description="Verify test results")
    parser.add_argument("--test-name", required=True, help="Name of the test")
    parser.add_argument(
        "--action-result",
        type=int,
        default=None,
        help="Exit code from action (0=success)",
    )
    parser.add_argument(
        "--expected",
        type=str,
        default=None,
        help="Expected joint positions (comma-separated)",
    )
    parser.add_argument(
        "--tolerance", type=float, default=0.1, help="Position tolerance in radians"
    )
    parser.add_argument(
        "--joint-order",
        type=str,
        default=None,
        help="Joint names in order (comma-separated)",
    )

    args = parser.parse_args()

    if args.action_result is not None:
        # Simple action result verification
        exit_code = verify_action_result(args.action_result, args.test_name)
    elif args.expected is not None:
        # Joint position verification
        expected = [float(x.strip()) for x in args.expected.split(",")]
        joint_order = None
        if args.joint_order:
            joint_order = [x.strip() for x in args.joint_order.split(",")]
        exit_code = verify_joint_positions(
            args.test_name, expected, args.tolerance, joint_order
        )
    else:
        print("Error: Must specify either --action-result or --expected")
        exit_code = 1

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
