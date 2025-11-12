#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Compare tracking performance between position_pid and direct position control modes.

This script records joint states during trajectory execution and generates
comparison plots showing tracking error, velocities, and torques.

Usage:
    # Start recording (run BEFORE sending trajectory)
    python3 scripts/compare_position_modes.py --mode pid --record --output pid_data.csv

    # Or for direct mode
    python3 scripts/compare_position_modes.py --mode direct --record --output direct_data.csv

    # After recording both modes, generate comparison plots
    python3 scripts/compare_position_modes.py --plot \
        --pid-data pid_data.csv \
        --direct-data direct_data.csv \
        --output comparison_plots.png
"""

import argparse
import csv
import sys
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")  # Non-interactive backend
    import matplotlib.pyplot as plt
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Warning: matplotlib not available. Plotting disabled.")


class TrajectoryRecorder(Node):
    """Record joint states during trajectory execution."""

    def __init__(self, output_file: str):
        """Initialize the trajectory recorder.

        Args:
            output_file: Path to save recorded data (CSV format)
        """
        super().__init__("trajectory_recorder")

        self.output_file = Path(output_file)
        self.data = []
        self.start_time = None

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        self.get_logger().info(f"Recording trajectory data to: {self.output_file}")
        self.get_logger().info("Press Ctrl+C to stop recording and save data")

    def joint_state_callback(self, msg: JointState):
        """Record joint state data."""
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        # Extract data for OpenArm joints only
        joint_indices = []
        for i in range(1, 8):
            joint_name = f"openarm_joint{i}"
            if joint_name in msg.name:
                joint_indices.append(msg.name.index(joint_name))

        if not joint_indices:
            return

        # Record: time, positions, velocities, efforts for each joint
        row = [elapsed]
        for idx in joint_indices:
            row.append(msg.position[idx] if idx < len(msg.position) else 0.0)
        for idx in joint_indices:
            row.append(msg.velocity[idx] if idx < len(msg.velocity) else 0.0)
        for idx in joint_indices:
            row.append(msg.effort[idx] if idx < len(msg.effort) else 0.0)

        self.data.append(row)

    def save_data(self):
        """Save recorded data to CSV file."""
        if not self.data:
            self.get_logger().warn("No data recorded!")
            return

        # Create header
        header = ["time"]
        for i in range(1, 8):
            header.append(f"pos_joint{i}")
        for i in range(1, 8):
            header.append(f"vel_joint{i}")
        for i in range(1, 8):
            header.append(f"effort_joint{i}")

        # Write to CSV
        with open(self.output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(self.data)

        self.get_logger().info(
            f"Saved {len(self.data)} samples to {self.output_file}"
        )


def plot_comparison(pid_file: str, direct_file: str, output_file: str):
    """Generate comparison plots from recorded data.

    Args:
        pid_file: Path to PID mode data CSV
        direct_file: Path to direct mode data CSV
        output_file: Path to save comparison plots
    """
    if not PLOTTING_AVAILABLE:
        print("Error: matplotlib not installed. Cannot generate plots.")
        print("Install with: pip install matplotlib")
        return False

    # Load data
    try:
        pid_data = np.loadtxt(pid_file, delimiter=",", skiprows=1)
        direct_data = np.loadtxt(direct_file, delimiter=",", skiprows=1)
    except Exception as e:
        print(f"Error loading data: {e}")
        return False

    # Extract time and joint data
    pid_time = pid_data[:, 0]
    direct_time = direct_data[:, 0]

    # Columns: time, pos1-7, vel1-7, effort1-7
    pid_pos = pid_data[:, 1:8]
    direct_pos = direct_data[:, 1:8]
    pid_vel = pid_data[:, 8:15]
    direct_vel = direct_data[:, 8:15]
    pid_effort = pid_data[:, 15:22]
    direct_effort = direct_data[:, 15:22]

    # Create reference trajectory (0 to 1 over duration)
    max_time = max(pid_time[-1], direct_time[-1])
    ref_time = np.linspace(0, max_time, 100)
    ref_pos = ref_time / max_time  # Linear 0 to 1

    # Create figure with subplots
    fig, axes = plt.subplots(3, 3, figsize=(18, 12))
    fig.suptitle(
        "Position Control Mode Comparison: PID vs Direct",
        fontsize=16,
        fontweight="bold"
    )

    # Plot joint 1, 4, 7 as representatives (proximal, mid, distal)
    joint_indices = [0, 3, 6]  # joint1, joint4, joint7
    joint_labels = ["Joint 1 (Shoulder)", "Joint 4 (Elbow)", "Joint 7 (Wrist)"]

    for col, (joint_idx, joint_label) in enumerate(zip(joint_indices, joint_labels)):
        # Position tracking
        ax_pos = axes[0, col]
        ax_pos.plot(ref_time, ref_pos, "k--", label="Reference", linewidth=2)
        ax_pos.plot(
            pid_time,
            pid_pos[:, joint_idx],
            "b-",
            label="PID",
            alpha=0.8
        )
        ax_pos.plot(
            direct_time,
            direct_pos[:, joint_idx],
            "r-",
            label="Direct",
            alpha=0.8
        )
        ax_pos.set_ylabel("Position (rad)")
        ax_pos.set_title(f"{joint_label} - Position")
        ax_pos.legend()
        ax_pos.grid(True, alpha=0.3)

        # Velocity
        ax_vel = axes[1, col]
        ax_vel.plot(
            pid_time,
            pid_vel[:, joint_idx],
            "b-",
            label="PID",
            alpha=0.8
        )
        ax_vel.plot(
            direct_time,
            direct_vel[:, joint_idx],
            "r-",
            label="Direct",
            alpha=0.8
        )
        ax_vel.set_ylabel("Velocity (rad/s)")
        ax_vel.set_title(f"{joint_label} - Velocity")
        ax_vel.legend()
        ax_vel.grid(True, alpha=0.3)

        # Effort/Torque
        ax_effort = axes[2, col]
        ax_effort.plot(
            pid_time,
            pid_effort[:, joint_idx],
            "b-",
            label="PID",
            alpha=0.8
        )
        ax_effort.plot(
            direct_time,
            direct_effort[:, joint_idx],
            "r-",
            label="Direct",
            alpha=0.8
        )
        ax_effort.set_xlabel("Time (s)")
        ax_effort.set_ylabel("Effort (N·m)")
        ax_effort.set_title(f"{joint_label} - Effort")
        ax_effort.legend()
        ax_effort.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches="tight")
    print(f"\n✓ Comparison plots saved to: {output_file}")

    # Compute and print tracking statistics
    print("\n" + "=" * 60)
    print("Tracking Performance Summary")
    print("=" * 60)

    for joint_idx, joint_label in zip(joint_indices, joint_labels):
        # Interpolate reference to match sample times
        ref_pid = np.interp(pid_time, ref_time, ref_pos)
        ref_direct = np.interp(direct_time, ref_time, ref_pos)

        pid_error = pid_pos[:, joint_idx] - ref_pid
        direct_error = direct_pos[:, joint_idx] - ref_direct

        pid_rmse = np.sqrt(np.mean(pid_error**2))
        direct_rmse = np.sqrt(np.mean(direct_error**2))
        pid_max_error = np.max(np.abs(pid_error))
        direct_max_error = np.max(np.abs(direct_error))

        print(f"\n{joint_label}:")
        print(f"  PID Mode:    RMSE={pid_rmse:.4f} rad, Max Error={pid_max_error:.4f} rad")
        print(f"  Direct Mode: RMSE={direct_rmse:.4f} rad, Max Error={direct_max_error:.4f} rad")

    print("=" * 60 + "\n")

    return True


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Compare position control modes"
    )

    # Recording mode
    parser.add_argument(
        "--record",
        action="store_true",
        help="Record trajectory data"
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["pid", "direct"],
        help="Control mode for recording"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="trajectory_data.csv",
        help="Output file for recorded data or plots"
    )

    # Plotting mode
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Generate comparison plots"
    )
    parser.add_argument(
        "--pid-data",
        type=str,
        help="Path to PID mode data CSV"
    )
    parser.add_argument(
        "--direct-data",
        type=str,
        help="Path to direct mode data CSV"
    )

    args = parser.parse_args()

    # Validate arguments
    if args.record and args.plot:
        print("Error: Cannot use --record and --plot simultaneously")
        return 1

    if not args.record and not args.plot:
        print("Error: Must specify either --record or --plot")
        parser.print_help()
        return 1

    # Recording mode
    if args.record:
        if not args.mode:
            print("Error: --mode required for recording")
            return 1

        print("\n" + "=" * 60)
        print(f"Recording {args.mode.upper()} mode trajectory data")
        print("=" * 60)
        print(f"Output: {args.output}")
        print("\nStart the trajectory now, then press Ctrl+C to stop")
        print("=" * 60 + "\n")

        rclpy.init()
        try:
            recorder = TrajectoryRecorder(args.output)
            rclpy.spin(recorder)
        except KeyboardInterrupt:
            print("\nRecording stopped by user")
            recorder.save_data()
        finally:
            rclpy.shutdown()

        return 0

    # Plotting mode
    if args.plot:
        if not args.pid_data or not args.direct_data:
            print("Error: --pid-data and --direct-data required for plotting")
            return 1

        print("\n" + "=" * 60)
        print("Generating comparison plots")
        print("=" * 60)
        print(f"PID data: {args.pid_data}")
        print(f"Direct data: {args.direct_data}")
        print(f"Output: {args.output}")
        print("=" * 60 + "\n")

        success = plot_comparison(args.pid_data, args.direct_data, args.output)
        return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
