#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Compute FK poses for OpenARM 7-DOF robot using MuJoCo.

This script loads the OpenARM MuJoCo model and computes end-effector poses
for given joint configurations. Use this to generate verified waypoints
for Cartesian controller tests.

Usage:
    # Compute FK for home and predefined poses
    python3 compute_openarm_fk.py

    # Compute FK for custom joint configuration
    python3 compute_openarm_fk.py --joints 0.1 0.2 -0.1 0.3 0.0 -0.2 0.1

    # Generate test waypoints with incremental movement
    python3 compute_openarm_fk.py --generate-waypoints
"""

import argparse
import math
from pathlib import Path

import mujoco
import numpy as np
from ament_index_python.packages import get_package_share_directory


def quat_to_euler(quat: np.ndarray) -> tuple[float, float, float]:
    """Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw)."""
    w, x, y, z = quat

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def load_openarm_model() -> tuple:
    """Load the OpenARM MuJoCo model."""
    try:
        pkg_share = Path(get_package_share_directory("openarm_description"))
        xml_path = pkg_share / "mujoco_models" / "openarm_v10.xml"
    except Exception:
        # Fallback to source path
        xml_path = Path(
            "/home/gilwoo/ros2_ws/src/openarm_description/mujoco_models/openarm_v10.xml"
        )

    if not xml_path.exists():
        raise FileNotFoundError(f"MuJoCo model not found: {xml_path}")

    print(f"Loading model: {xml_path}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    return model, data


def get_ee_pose(model, data, ee_body_name: str = "openarm_link7") -> dict:
    """Get end-effector position and orientation from MuJoCo data."""
    ee_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name)
    if ee_body_id < 0:
        raise ValueError(f"Body '{ee_body_name}' not found in model")

    pos = data.xpos[ee_body_id].copy()
    quat = data.xquat[ee_body_id].copy()  # MuJoCo uses (w, x, y, z)

    return {
        "position": {"x": pos[0], "y": pos[1], "z": pos[2]},
        "orientation": {"w": quat[0], "x": quat[1], "y": quat[2], "z": quat[3]},
    }


def compute_fk(
    model, data, joint_positions: list[float], ee_body_name: str = "openarm_link7"
) -> dict:
    """Compute FK for given joint positions."""
    # Set joint positions
    nq = min(len(joint_positions), model.nq)
    data.qpos[:nq] = joint_positions[:nq]
    data.qvel[:] = 0

    # Forward kinematics
    mujoco.mj_forward(model, data)

    return get_ee_pose(model, data, ee_body_name)


def print_pose(name: str, q: list[float], pose: dict):
    """Print pose in a formatted way."""
    pos = pose["position"]
    quat = pose["orientation"]
    euler = quat_to_euler(
        np.array([quat["w"], quat["x"], quat["y"], quat["z"]])
    )

    print(f"\n  {name}:")
    print(f"    q = [{', '.join(f'{v:.3f}' for v in q)}]")
    print(f"    EE position:    ({pos['x']:.4f}, {pos['y']:.4f}, {pos['z']:.4f}) m")
    print(
        f"    EE orientation: (w={quat['w']:.4f}, x={quat['x']:.4f}, "
        f"y={quat['y']:.4f}, z={quat['z']:.4f})"
    )
    print(
        f"    Euler (RPY):    ({math.degrees(euler[0]):.1f}°, "
        f"{math.degrees(euler[1]):.1f}°, {math.degrees(euler[2]):.1f}°)"
    )


def print_launch_format(name: str, pose: dict, time_sec: int):
    """Print pose in ROS2 launch file format."""
    pos = pose["position"]
    quat = pose["orientation"]

    print(
        f"    # {name}: t={time_sec}s\n"
        f"    send_pose_{name.lower()} = ExecuteProcess(\n"
        f"        cmd=[\n"
        f'            "ros2", "topic", "pub", "--once",\n'
        f'            "/zordi_cartesian_ik_controller/target_pose",\n'
        f'            "geometry_msgs/msg/PoseStamped",\n'
        f"            \"{{header: {{frame_id: 'openarm_link0'}}, \"\n"
        f'            "pose: {{position: {{x: {pos["x"]:.4f}, y: {pos["y"]:.4f}, '
        f'z: {pos["z"]:.4f}}}, "\n'
        f'            "orientation: {{w: {quat["w"]:.4f}, x: {quat["x"]:.4f}, '
        f'y: {quat["y"]:.4f}, z: {quat["z"]:.4f}}}}}}}",\n'
        f"        ],\n"
        f'        output="screen",\n'
        f"    )"
    )


def print_trajectory_format(waypoints: list[tuple[str, list[float], dict]]):
    """Print waypoints in CartesianTrajectory format."""
    print("\n# CartesianTrajectory format:")
    print('"{header: {frame_id: \'openarm_link0\'}, tracked_frame: \'openarm_link7\', "')
    print('"points: ["')

    for i, (name, _, pose) in enumerate(waypoints):
        pos = pose["position"]
        quat = pose["orientation"]
        time_sec = (i + 1) * 2  # 2 seconds per waypoint

        comma = "," if i < len(waypoints) - 1 else ""
        print(
            f'  "{{point: {{pose: {{position: {{x: {pos["x"]:.4f}, '
            f'y: {pos["y"]:.4f}, z: {pos["z"]:.4f}}}, '
            f'orientation: {{w: {quat["w"]:.4f}, x: {quat["x"]:.4f}, '
            f'y: {quat["y"]:.4f}, z: {quat["z"]:.4f}}}}}}}, '
            f'time_from_start: {{sec: {time_sec}}}}}{comma}"'
        )

    print('"]}",')


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Compute OpenARM FK poses")
    parser.add_argument(
        "--joints",
        nargs=7,
        type=float,
        metavar="Q",
        help="Joint positions (7 values for 7-DOF arm)",
    )
    parser.add_argument(
        "--generate-waypoints",
        action="store_true",
        help="Generate incremental waypoints from home",
    )
    parser.add_argument(
        "--launch-format",
        action="store_true",
        help="Output in ROS2 launch file format",
    )
    args = parser.parse_args()

    # Load model
    model, data = load_openarm_model()

    print("\n" + "=" * 60)
    print("  OpenARM FK Computation")
    print("=" * 60)
    print(f"  Model: nq={model.nq}, nv={model.nv}")

    # Find joint names
    print("\n  Joints:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if joint_name:
            print(f"    {i}: {joint_name}")

    if args.joints:
        # Compute FK for specified joint configuration
        q = args.joints
        pose = compute_fk(model, data, q)
        print_pose("Custom", q, pose)

    elif args.generate_waypoints:
        # Generate incremental waypoints from home
        print("\n" + "-" * 60)
        print("  Generating waypoints from home position")
        print("-" * 60)

        # Home position (all zeros)
        home_q = [0.0] * 7
        home_pose = compute_fk(model, data, home_q)
        print_pose("home", home_q, home_pose)

        waypoints = [("home", home_q, home_pose)]

        # Generate 3 waypoints with small movements
        # Move joints 1, 2, 4 slightly to create EE motion
        wp_configs = [
            ("wp1", [0.2, 0.3, 0.0, 0.2, 0.0, 0.0, 0.0]),
            ("wp2", [-0.2, 0.3, 0.0, 0.2, 0.0, 0.0, 0.0]),
            ("wp3", [0.0, 0.4, 0.0, 0.3, 0.0, 0.0, 0.0]),
        ]

        for name, q in wp_configs:
            pose = compute_fk(model, data, q)
            print_pose(name, q, pose)
            waypoints.append((name, q, pose))

        if args.launch_format:
            print("\n" + "-" * 60)
            print("  Launch file format:")
            print("-" * 60)
            for i, (name, _, pose) in enumerate(waypoints[1:], 1):  # Skip home
                print_launch_format(name, pose, i * 2)

        print_trajectory_format(waypoints[1:])  # Skip home for trajectory

    else:
        # Compute FK for predefined keyframes
        print("\n" + "-" * 60)
        print("  Predefined keyframe poses")
        print("-" * 60)

        keyframes = {
            "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "pose1": [1.0, 1.5, -1.0, 2.0, 1.0, -1.5, 1.5],
            "pose2": [-1.0, -1.5, 1.0, -2.0, -1.0, 1.5, -1.5],
            "small_move": [0.1, 0.2, 0.0, 0.1, 0.0, 0.0, 0.0],
        }

        for name, q in keyframes.items():
            pose = compute_fk(model, data, q)
            print_pose(name, q, pose)

        if args.launch_format:
            print("\n" + "-" * 60)
            print("  Launch file format (for small_move):")
            print("-" * 60)
            pose = compute_fk(model, data, keyframes["small_move"])
            print_launch_format("small_move", pose, 2)

    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()

