#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Apply PID gains from JSON file to URDF and rebuild workspace.

This helper script automates the process of:
1. Loading gains from JSON (e.g., optimal_pid_gains.json)
2. Updating the ros2_control URDF xacro file
3. Rebuilding the workspace
4. (Optional) Relaunching simulation

Usage:
    # Apply gains from autotune_pid_simple.py output
    python3 scripts/apply_gains.py figs/optimal_pid_gains.json

    # Apply gains and auto-rebuild
    python3 scripts/apply_gains.py figs/optimal_pid_gains.json --rebuild

    # Dry-run (show what would be changed without modifying files)
    python3 scripts/apply_gains.py figs/optimal_pid_gains.json --dry-run

Output:
    - Updates urdf/ros2_control/openarm.ros2_control.xacro
    - Backs up original to openarm.ros2_control.xacro.backup
    - Rebuilds workspace (if --rebuild flag is set)
"""

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path


def load_gains(json_path):
    """Load PID gains from JSON file."""
    with open(json_path, "r") as f:
        data = json.load(f)

    gains = []
    for i in range(1, 8):
        joint_key = f"joint{i}"
        if joint_key not in data:
            raise ValueError(f"Missing {joint_key} in JSON file")

        gains.append({
            "kp": float(data[joint_key]["kp"]),
            "kd": float(data[joint_key]["kd"]),
            "ki": float(data[joint_key].get("ki", 0.0)),
        })

    return gains


def update_urdf(urdf_path, gains, dry_run=False):
    """
    Update URDF file with new PID gains.

    Args:
        urdf_path: Path to openarm.ros2_control.xacro
        gains: List of 7 dicts with 'kp', 'kd', 'ki' keys
        dry_run: If True, only show changes without modifying file

    Returns:
        bool: True if successful
    """
    # Read current URDF
    urdf_content = urdf_path.read_text()
    lines = urdf_content.split("\n")

    print("\n" + "=" * 70)
    print("URDF GAIN UPDATES")
    print("=" * 70)

    modified_lines = []
    for i in range(7):
        joint_num = i + 1
        kp = gains[i]["kp"]
        kd = gains[i]["kd"]
        ki = gains[i]["ki"]

        # Find the line
        old_pattern = f'<xacro:configure_joint joint_name="openarm_${{arm_prefix}}joint{joint_num}"'

        for line_idx, line in enumerate(lines):
            if old_pattern in line:
                old_line = line
                new_line = (
                    f'      <xacro:configure_joint joint_name="openarm_${{arm_prefix}}joint{joint_num}" '
                    f'initial_position="0.0" kp="{kp:.1f}" kd="{kd:.1f}" ki="{ki:.1f}"/>'
                )

                print(f"\nJoint {joint_num}:")
                print(f"  OLD: {old_line.strip()}")
                print(f"  NEW: {new_line.strip()}")

                if not dry_run:
                    lines[line_idx] = new_line
                    modified_lines.append(joint_num)
                break

    if dry_run:
        print("\n" + "=" * 70)
        print("DRY RUN - No changes made")
        print("=" * 70)
        return True

    # Backup original
    backup_path = urdf_path.parent / f"{urdf_path.name}.backup"
    if not backup_path.exists():
        shutil.copy(urdf_path, backup_path)
        print(f"\n✓ Backed up original to: {backup_path}")

    # Write updated content
    urdf_content = "\n".join(lines)
    urdf_path.write_text(urdf_content)

    print("\n" + "=" * 70)
    print(f"✓ Updated {len(modified_lines)} joints in URDF")
    print(f"  File: {urdf_path}")
    print("=" * 70)

    return True


def rebuild_workspace(workspace_path):
    """Rebuild ROS2 workspace."""
    print("\n" + "=" * 70)
    print("REBUILDING WORKSPACE")
    print("=" * 70)

    try:
        result = subprocess.run(
            ["colcon", "build", "--packages-select", "openarm_description"],
            cwd=workspace_path,
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
        )

        if result.returncode == 0:
            print("✓ Workspace rebuilt successfully")
            print("\nNext steps:")
            print("  1. Source the workspace:")
            print("     source ~/ros2_ws/install/setup.bash")
            print("  2. Relaunch simulation:")
            print("     ros2 launch openarm_description mujoco_sim.launch.py")
            print("  3. Test the new gains:")
            print("     python3 scripts/test_trajectory_tracking.py")
            return True
        else:
            print("✗ Build failed:")
            print(result.stderr)
            return False

    except subprocess.TimeoutExpired:
        print("✗ Build timeout (exceeded 120 seconds)")
        return False
    except Exception as e:
        print(f"✗ Build error: {e}")
        return False


def main():
    """Apply PID gains from JSON to URDF."""
    parser = argparse.ArgumentParser(description="Apply PID gains from JSON to URDF")
    parser.add_argument(
        "json_file",
        type=str,
        help="Path to JSON file with PID gains",
    )
    parser.add_argument(
        "--urdf",
        type=str,
        help="Path to ros2_control URDF xacro (default: auto-detect)",
    )
    parser.add_argument(
        "--rebuild",
        action="store_true",
        help="Rebuild workspace after updating URDF",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show changes without modifying files",
    )

    args = parser.parse_args()

    # Resolve paths
    json_path = Path(args.json_file)
    if not json_path.exists():
        print(f"✗ Error: JSON file not found: {json_path}")
        return 1

    if args.urdf:
        urdf_path = Path(args.urdf)
    else:
        urdf_path = (
            Path.home()
            / "ros2_ws/src/openarm_description/urdf/ros2_control"
            / "openarm.ros2_control.xacro"
        )

    if not urdf_path.exists():
        print(f"✗ Error: URDF file not found: {urdf_path}")
        return 1

    workspace_path = Path.home() / "ros2_ws"

    print("=" * 70)
    print("APPLY PID GAINS TO URDF")
    print("=" * 70)
    print(f"JSON file:  {json_path}")
    print(f"URDF file:  {urdf_path}")
    print(f"Workspace:  {workspace_path}")
    if args.dry_run:
        print("Mode:       DRY RUN (no changes will be made)")
    print("=" * 70)

    try:
        # Load gains
        gains = load_gains(json_path)
        print(f"\n✓ Loaded gains for {len(gains)} joints from JSON")

        # Update URDF
        if not update_urdf(urdf_path, gains, dry_run=args.dry_run):
            return 1

        if args.dry_run:
            return 0

        # Rebuild workspace if requested
        if args.rebuild:
            if not rebuild_workspace(workspace_path):
                return 1
        else:
            print("\n" + "=" * 70)
            print("URDF UPDATED (workspace not rebuilt)")
            print("=" * 70)
            print("\nTo apply changes, rebuild workspace:")
            print("  cd ~/ros2_ws")
            print("  colcon build --packages-select openarm_description")
            print("  source install/setup.bash")
            print("\nOr run this script with --rebuild flag:")
            print(f"  python3 scripts/apply_gains.py {args.json_file} --rebuild")

        return 0

    except Exception as e:
        print(f"\n✗ Error: {e}")
        return 1


if __name__ == "__main__":
    exit(main())
