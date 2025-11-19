#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Compute forward kinematics for OpenARM to get end-effector pose.
"""

import numpy as np
import pinocchio as pin
from pathlib import Path


def compute_ee_pose(urdf_path: str, joint_positions: np.ndarray, ee_frame_name: str = "openarm_link7"):
    """
    Compute end-effector pose given joint positions.

    Args:
        urdf_path: Path to URDF file
        joint_positions: Joint positions (7-DOF for OpenARM)
        ee_frame_name: Name of end-effector frame

    Returns:
        SE3 pose of end-effector in base frame
    """
    # Load model
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()

    # Find end-effector frame
    if not model.existFrame(ee_frame_name):
        raise ValueError(f"Frame '{ee_frame_name}' not found in model")

    ee_frame_id = model.getFrameId(ee_frame_name)

    # Compute forward kinematics
    pin.framesForwardKinematics(model, data, joint_positions)

    # Get end-effector pose
    ee_pose = data.oMf[ee_frame_id]

    return ee_pose


def pose_to_dict(pose: pin.SE3):
    """Convert SE3 pose to dictionary with position and quaternion."""
    position = pose.translation
    rotation = pose.rotation
    quat = pin.Quaternion(rotation)

    return {
        "position": {
            "x": float(position[0]),
            "y": float(position[1]),
            "z": float(position[2]),
        },
        "orientation": {
            "x": float(quat.x),
            "y": float(quat.y),
            "z": float(quat.z),
            "w": float(quat.w),
        }
    }


def main():
    # Path to URDF
    urdf_path = Path(__file__).parent.parent / "mujoco_models" / "openarm_v10.urdf"

    if not urdf_path.exists():
        print(f"Error: URDF not found at {urdf_path}")
        return

    # Initial configuration (home pose - all zeros)
    q_home = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Target configuration (rotate joint1 by 0.5 rad)
    q_target = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # Compute poses
    print("=" * 70)
    print("OpenARM Forward Kinematics - End-Effector Pose Calculation")
    print("=" * 70)

    print("\n1. HOME POSE (all joints at 0.0 rad):")
    print(f"   Joint positions: {q_home}")
    pose_home = compute_ee_pose(str(urdf_path), q_home)
    pose_home_dict = pose_to_dict(pose_home)
    print(f"\n   End-effector pose:")
    print(f"   Position (x, y, z): ({pose_home_dict['position']['x']:.6f}, "
          f"{pose_home_dict['position']['y']:.6f}, {pose_home_dict['position']['z']:.6f})")
    print(f"   Orientation (x, y, z, w): ({pose_home_dict['orientation']['x']:.6f}, "
          f"{pose_home_dict['orientation']['y']:.6f}, {pose_home_dict['orientation']['z']:.6f}, "
          f"{pose_home_dict['orientation']['w']:.6f})")

    print("\n" + "-" * 70)
    print("\n2. TARGET POSE (joint1 = 0.5 rad, others at 0.0 rad):")
    print(f"   Joint positions: {q_target}")
    pose_target = compute_ee_pose(str(urdf_path), q_target)
    pose_target_dict = pose_to_dict(pose_target)
    print(f"\n   End-effector pose:")
    print(f"   Position (x, y, z): ({pose_target_dict['position']['x']:.6f}, "
          f"{pose_target_dict['position']['y']:.6f}, {pose_target_dict['position']['z']:.6f})")
    print(f"   Orientation (x, y, z, w): ({pose_target_dict['orientation']['x']:.6f}, "
          f"{pose_target_dict['orientation']['y']:.6f}, {pose_target_dict['orientation']['z']:.6f}, "
          f"{pose_target_dict['orientation']['w']:.6f})")

    print("\n" + "=" * 70)
    print("\nROS2 COMMAND to send target pose to Cartesian controller:")
    print("=" * 70)
    print(f"""
ros2 topic pub /zordi_cartesian_controller/target_pose \\
  geometry_msgs/msg/PoseStamped \\
  "{{
    header: {{
      stamp: {{sec: 0, nanosec: 0}},
      frame_id: 'openarm_link0'
    }},
    pose: {{
      position: {{
        x: {pose_target_dict['position']['x']:.6f},
        y: {pose_target_dict['position']['y']:.6f},
        z: {pose_target_dict['position']['z']:.6f}
      }},
      orientation: {{
        x: {pose_target_dict['orientation']['x']:.6f},
        y: {pose_target_dict['orientation']['y']:.6f},
        z: {pose_target_dict['orientation']['z']:.6f},
        w: {pose_target_dict['orientation']['w']:.6f}
      }}
    }}
  }}"
""")

    print("\nAlternatively, use the Python API or action interface for smoother control.")
    print("=" * 70)


if __name__ == "__main__":
    main()

