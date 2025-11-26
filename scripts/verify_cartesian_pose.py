#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Verify end-effector Cartesian pose for test automation.

This script subscribes to joint_states, computes FK using MuJoCo,
and compares the actual EE pose to the expected target pose.

Usage:
    python3 verify_cartesian_pose.py --test-name "Test Name" \
        --target-x 0.4 --target-y 0.0 --target-z 0.5 \
        --tolerance 0.05
"""

import argparse
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

import mujoco


def print_success(test_name: str):
    """Print success message."""
    print("\n" + "=" * 50)
    print(f"  ✅ TEST PASSED: {test_name}")
    print("=" * 50 + "\n")


def print_failure(test_name: str, reason: str):
    """Print failure message."""
    print("\n" + "=" * 50)
    print(f"  ❌ TEST FAILED: {test_name}")
    print(f"  Reason: {reason}")
    print("=" * 50 + "\n")


class CartesianPoseVerifier(Node):
    """Node to verify Cartesian end-effector pose."""

    def __init__(self):
        super().__init__("cartesian_pose_verifier")
        self.joint_states = None
        self.sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

    def joint_states_callback(self, msg: JointState):
        """Store latest joint states."""
        self.joint_states = msg


def load_mujoco_model():
    """Load the OpenARM MuJoCo model."""
    try:
        pkg_share = get_package_share_directory("openarm_description")
        xml_path = f"{pkg_share}/mujoco_models/openarm_v10.xml"
    except Exception:
        xml_path = (
            "/home/gilwoo/ros2_ws/src/openarm_description/mujoco_models/openarm_v10.xml"
        )

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    return model, data


def compute_ee_pose(
    model, data, joint_positions: list, ee_body_name: str = "openarm_link7"
):
    """Compute end-effector pose from joint positions using MuJoCo FK."""
    # Set joint positions
    data.qpos[: len(joint_positions)] = joint_positions
    data.qvel[:] = 0.0

    # Forward kinematics
    mujoco.mj_forward(model, data)

    # Get EE body pose
    ee_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name)
    if ee_body_id < 0:
        raise ValueError(f"Body '{ee_body_name}' not found in model")

    ee_pos = data.xpos[ee_body_id].copy()
    ee_quat = data.xquat[ee_body_id].copy()  # MuJoCo uses (w, x, y, z)

    return ee_pos, ee_quat


def main():
    parser = argparse.ArgumentParser(
        description="Verify Cartesian end-effector pose for tests."
    )
    parser.add_argument("--test-name", type=str, required=True, help="Name of the test")
    parser.add_argument(
        "--target-x", type=float, required=True, help="Target EE X position (m)"
    )
    parser.add_argument(
        "--target-y", type=float, required=True, help="Target EE Y position (m)"
    )
    parser.add_argument(
        "--target-z", type=float, required=True, help="Target EE Z position (m)"
    )
    parser.add_argument(
        "--tolerance", type=float, default=0.05, help="Position tolerance (m)"
    )
    parser.add_argument(
        "--joint-order",
        type=str,
        default="openarm_joint1,openarm_joint2,openarm_joint3,openarm_joint4,openarm_joint5,openarm_joint6,openarm_joint7",
        help="Comma-separated joint names in order",
    )
    args = parser.parse_args()

    rclpy.init()
    node = CartesianPoseVerifier()

    target_pos = np.array([args.target_x, args.target_y, args.target_z])
    joint_names = args.joint_order.split(",")

    node.get_logger().info(f"Verifying test: {args.test_name}")
    node.get_logger().info(
        f"Target EE position: ({args.target_x}, {args.target_y}, {args.target_z})"
    )
    node.get_logger().info(f"Tolerance: {args.tolerance} m")

    # Wait for joint states
    start_time = time.time()
    timeout = 10.0
    while node.joint_states is None and (time.time() - start_time < timeout):
        node.get_logger().info("Waiting for /joint_states...")
        rclpy.spin_once(node, timeout_sec=1.0)

    if node.joint_states is None:
        print_failure(args.test_name, "No joint states received")
        rclpy.shutdown()
        sys.exit(1)

    # Reorder joint positions to match expected order
    joint_pos_map = {
        name: pos
        for name, pos in zip(node.joint_states.name, node.joint_states.position)
    }
    joint_positions = [joint_pos_map.get(name, 0.0) for name in joint_names]

    # Load MuJoCo model and compute FK
    try:
        model, data = load_mujoco_model()
        actual_pos, actual_quat = compute_ee_pose(model, data, joint_positions)
    except Exception as e:
        print_failure(args.test_name, f"FK computation failed: {e}")
        rclpy.shutdown()
        sys.exit(1)

    # Compute position error
    pos_error = np.linalg.norm(actual_pos - target_pos)

    print("\n")
    print(
        f"Target EE:  ({target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}) m"
    )
    print(
        f"Actual EE:  ({actual_pos[0]:.4f}, {actual_pos[1]:.4f}, {actual_pos[2]:.4f}) m"
    )
    print(f"Position error: {pos_error:.4f} m")
    print(f"Tolerance: {args.tolerance:.4f} m")
    print("\n")

    if pos_error <= args.tolerance:
        print_success(args.test_name)
        rclpy.shutdown()
        sys.exit(0)
    else:
        print_failure(
            args.test_name,
            f"Position error {pos_error:.4f}m exceeds tolerance {args.tolerance:.4f}m",
        )
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
