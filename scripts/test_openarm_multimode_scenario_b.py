#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Automated test script for OpenARM 7-DOF multimode control - Scenario B.

Test workflow: Use zordi_mit_controller for both moving and holding

For each test configuration:
  1. Ensure zordi_mit_controller is active
  2. Send 7-DOF trajectory to target position (5s duration, via action)
  3. Wait for trajectory completion
  4. Hold for 15 seconds (controller auto-holds with gravity comp)
  5. Record drift for each of 7 joints
  6. Compute RMS drift

Expected results:
  - zordi_mit holds position with < 0.01 rad/joint, < 0.015 rad RMS

Usage:
  ros2 launch openarm_description test_openarm_multimode.launch.py
  # Then in another terminal:
  python3 test_openarm_multimode_scenario_b.py
"""

import math
import time
from typing import List, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class OpenARMMultimodeTestScenarioB(Node):
    def __init__(self):
        super().__init__("test_openarm_multimode_scenario_b")

        # 7-DOF joint names
        self.joint_names = [
            "openarm_joint1",
            "openarm_joint2",
            "openarm_joint3",
            "openarm_joint4",
            "openarm_joint5",
            "openarm_joint6",
            "openarm_joint7",
        ]

        # Test configurations
        self.configurations = {
            "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "pose1": [1.0, 1.5, -1.0, 2.0, 1.0, -1.5, 1.5],
            "pose2": [-1.0, -1.5, 1.0, -2.0, -1.0, 1.5, -1.5],
            "pose3": [2.0, 1.0, 2.5, 1.5, 2.5, 1.0, 2.0],
            "pose4": [-2.0, -1.0, -2.5, -1.5, -2.5, -1.0, -2.0],
            "pose5": [1.5, -1.0, 2.0, -1.5, 1.0, -2.0, 1.5],
        }

        # Action client
        self.action_client = ActionClient(
            self, FollowJointTrajectory, "/zordi_mit_controller/follow_joint_trajectory"
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Service clients
        self.switch_controller_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        # State tracking
        self.current_positions = [0.0] * 7
        self.current_velocities = [0.0] * 7
        self.joint_state_received = False

        # Wait for services
        self.get_logger().info("Waiting for controller_manager services...")
        self.switch_controller_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Waiting for action server...")
        self.action_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info("Services available!")

        # Wait for initial joint state
        self.get_logger().info("Waiting for joint states...")
        while not self.joint_state_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(
            f"Initial positions: {[f'{p:.3f}' for p in self.current_positions]}"
        )

    def joint_state_callback(self, msg):
        """Store current joint state."""
        try:
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.current_positions[i] = msg.position[idx]
                    if msg.velocity:
                        self.current_velocities[i] = msg.velocity[idx]
            self.joint_state_received = True
        except Exception as e:
            self.get_logger().error(f"Error in joint_state_callback: {e}")

    def switch_controller(
        self, activate: List[str], deactivate: List[str], strict: bool = True
    ) -> bool:
        """Switch controllers via controller_manager service."""
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = (
            SwitchController.Request.STRICT
            if strict
            else SwitchController.Request.BEST_EFFORT
        )
        req.start_asap = True
        req.timeout = Duration(sec=0, nanosec=0)

        future = self.switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().ok:
            return True
        else:
            self.get_logger().error(
                f"Failed to switch controllers: {activate} / {deactivate}"
            )
            return False

    def send_trajectory_action(
        self, target_positions: List[float], duration_sec: float = 5.0
    ) -> bool:
        """Send trajectory via action and wait for completion."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * 7
        point.time_from_start = Duration(
            sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9)
        )
        goal_msg.trajectory.points.append(point)

        self.get_logger().info(
            f"Sending trajectory action: target={[f'{p:.2f}' for p in target_positions]}, "
            f"duration={duration_sec}s"
        )
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Action goal rejected")
            return False

        self.get_logger().info("Action goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=duration_sec + 5.0
        )

        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("Trajectory execution succeeded")
            return True
        else:
            self.get_logger().warn(
                f"Trajectory execution finished with status: {result.status}"
            )
            return False

    def measure_drift(self, duration: float = 15.0) -> Tuple[List[float], float]:
        """Measure position drift over a duration for all 7 joints."""
        start_positions = self.current_positions.copy()
        self.get_logger().info(
            f"Measuring drift for {duration}s (start={[f'{p:.3f}' for p in start_positions]})..."
        )

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        end_positions = self.current_positions.copy()
        drifts = [abs(end_positions[i] - start_positions[i]) for i in range(7)]
        rms_drift = math.sqrt(sum(d**2 for d in drifts) / 7)

        self.get_logger().info(
            f"Drift measurement complete: drifts={[f'{d:.5f}' for d in drifts]}, "
            f"RMS={rms_drift:.5f} rad"
        )
        return drifts, rms_drift

    def run_test(self) -> List[Tuple[str, List[float], float, bool]]:
        """Run the full test sequence."""
        results = []

        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("SCENARIO B TEST: zordi_mit for both move and hold")
        self.get_logger().info("=" * 80)

        # Activate zordi_mit once at the start
        self.get_logger().info("Activating zordi_mit_controller...")
        if not self.switch_controller(
            activate=["zordi_mit_controller"],
            deactivate=["joint_trajectory_controller"],
        ):
            self.get_logger().error(
                "Failed to activate zordi_mit_controller, aborting test"
            )
            return results

        time.sleep(1.0)  # Let controller stabilize

        for i, (config_name, target_pos) in enumerate(self.configurations.items()):
            self.get_logger().info(
                f"\n--- Test {i + 1}/{len(self.configurations)}: "
                f"Config = {config_name} ---"
            )

            # Step 1: Send trajectory via action
            self.get_logger().info("Step 1: Sending trajectory via action...")
            if not self.send_trajectory_action(target_pos, duration_sec=5.0):
                self.get_logger().error("Failed to execute trajectory, skipping test")
                results.append((config_name, [-1.0] * 7, -1.0, False))
                continue

            time.sleep(0.5)  # Settle after trajectory

            # Step 2: Measure drift (controller auto-holds with gravity comp)
            self.get_logger().info(
                "Step 2: Measuring drift (zordi_mit auto-holding with gravity comp)..."
            )
            drifts, rms_drift = self.measure_drift(duration=15.0)

            # Step 3: Record results
            success = rms_drift < 0.015  # Success if RMS drift < 15 millirad
            results.append((config_name, drifts, rms_drift, success))

            status = "✓ PASS" if success else "✗ FAIL"
            self.get_logger().info(f"Result: {status} (RMS drift={rms_drift:.6f} rad)")

        return results

    def print_summary(self, results: List[Tuple[str, List[float], float, bool]]):
        """Print test summary table."""
        self.get_logger().info("\n" + "=" * 110)
        self.get_logger().info("TEST SUMMARY")
        self.get_logger().info("=" * 110)
        self.get_logger().info(
            f"{'Config':<10} | {'J1':<8} | {'J2':<8} | {'J3':<8} | {'J4':<8} | "
            f"{'J5':<8} | {'J6':<8} | {'J7':<8} | {'RMS':<8} | {'Status':<8}"
        )
        self.get_logger().info("-" * 110)

        for config_name, drifts, rms_drift, success in results:
            status = "✓ PASS" if success else "✗ FAIL"
            drift_strs = [f"{d:.5f}" if d >= 0 else "N/A    " for d in drifts]
            rms_str = f"{rms_drift:.5f}" if rms_drift >= 0 else "N/A    "
            self.get_logger().info(
                f"{config_name:<10} | {drift_strs[0]:<8} | {drift_strs[1]:<8} | "
                f"{drift_strs[2]:<8} | {drift_strs[3]:<8} | {drift_strs[4]:<8} | "
                f"{drift_strs[5]:<8} | {drift_strs[6]:<8} | {rms_str:<8} | {status:<8}"
            )

        self.get_logger().info("-" * 110)
        passed = sum(1 for _, _, _, s in results if s)
        total = len(results)
        self.get_logger().info(f"Total: {passed}/{total} tests passed")
        self.get_logger().info("=" * 110 + "\n")


def main():
    rclpy.init()
    node = OpenARMMultimodeTestScenarioB()

    try:
        results = node.run_test()
        node.print_summary(results)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
