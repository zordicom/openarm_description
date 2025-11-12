#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Measure simulation speed: ratio of simulation time to real-world time.

This script subscribes to /clock (simulation time) and compares it to
wall-clock time to determine if simulation is running faster or slower
than real-time.

Expected result WITH bug: 8-15x real-time (sim runs too fast)
Expected result AFTER fix: ~1.0x real-time (proper real-time sync)

Usage:
    # Start simulation in separate terminal
    ros2 launch openarm_description single_arm.launch.py

    # Run this script
    python3 scripts/measure_sim_speed.py
"""

import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class SimSpeedMeasure(Node):
    """Measure simulation vs real-time speed ratio."""

    def __init__(self):
        """Initialize the node."""
        super().__init__("sim_speed_measure")

        self.clock_sub = self.create_subscription(
            Clock, "/clock", self.clock_callback, 10
        )

        self.start_real_time = None
        self.start_sim_time = None
        self.last_measurement_time = None

        self.get_logger().info("Waiting for first clock message...")
        self.get_logger().info(
            "Will measure simulation speed over 5-second intervals"
        )

    def clock_callback(self, msg):
        """Process clock messages and measure speed."""
        current_real_time = time.time()
        current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        # Initialize on first message
        if self.start_real_time is None:
            self.start_real_time = current_real_time
            self.start_sim_time = current_sim_time
            self.last_measurement_time = current_real_time
            self.get_logger().info(
                f"Started measurement at sim_time={current_sim_time:.3f}s"
            )
            return

        # Measure every 5 seconds of real-time
        real_elapsed_since_last = current_real_time - self.last_measurement_time
        if real_elapsed_since_last >= 5.0:
            # Calculate speed since start
            total_real_elapsed = current_real_time - self.start_real_time
            total_sim_elapsed = current_sim_time - self.start_sim_time

            speedup = total_sim_elapsed / total_real_elapsed

            # Format output
            self.get_logger().info("=" * 60)
            self.get_logger().info("Simulation Speed Measurement")
            self.get_logger().info("=" * 60)
            self.get_logger().info(
                f"Real-time elapsed:   {total_real_elapsed:8.2f} seconds"
            )
            self.get_logger().info(
                f"Sim-time elapsed:    {total_sim_elapsed:8.2f} seconds"
            )
            self.get_logger().info(f"Speed ratio:         {speedup:8.2f}x")

            if speedup > 1.5:
                self.get_logger().warn(
                    f"⚠️  Simulation running {speedup:.1f}x FASTER than real-time!"
                )
                self.get_logger().warn(
                    "   This explains why 10s trajectories complete in ~"
                    f"{10.0/speedup:.1f}s"
                )
            elif speedup < 0.8:
                self.get_logger().warn(
                    f"⚠️  Simulation running {1.0/speedup:.1f}x SLOWER than real-time"
                )
                self.get_logger().warn("   Computer may be too slow for real-time")
            else:
                self.get_logger().info("✓ Simulation running at proper real-time speed")

            self.get_logger().info("=" * 60 + "\n")

            # Update last measurement time
            self.last_measurement_time = current_real_time


def main():
    """Main entry point."""
    print("\n" + "=" * 60)
    print("OpenArm Simulation Speed Measurement")
    print("=" * 60)
    print("Measuring ratio of simulation time to wall-clock time...")
    print("Press Ctrl+C to exit")
    print("=" * 60 + "\n")

    rclpy.init()
    node = SimSpeedMeasure()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
