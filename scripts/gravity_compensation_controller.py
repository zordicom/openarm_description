#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Gravity Compensation Controller for OpenARM in MuJoCo simulation.

This controller demonstrates proper gravity compensation using Pinocchio dynamics.
It reads joint states and publishes effort commands that compensate for gravity,
allowing the robot to hold its position without falling.

This is the CORRECT way to control the robot in simulation - the same approach
you'd use on the real robot with CartesianController or other torque-based controllers.

Usage:
    # Terminal 1: Start simulation with effort control
    export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
    ros2 launch openarm_description mujoco_sim.launch.py control_mode:=effort

    # Terminal 2: Run this controller
    python3 scripts/gravity_compensation_controller.py

    # The robot should now hold its position under gravity!
"""

from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GravityCompensationController(Node):
    """Controller that applies gravity compensation to hold robot position."""

    def __init__(self):
        """Initialize the gravity compensation controller."""
        super().__init__("gravity_compensation_controller")

        # Parameters
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("gravity_scale", 1.0)
        self.declare_parameter("control_rate", 100.0)  # Hz
        self.declare_parameter("add_damping", True)
        self.declare_parameter("damping_gains", [2.0, 2.0, 1.5, 1.5, 0.5, 0.5, 0.5])

        urdf_path = self.get_parameter("urdf_path").value
        self.gravity_scale = self.get_parameter("gravity_scale").value
        control_rate = self.get_parameter("control_rate").value
        self.add_damping = self.get_parameter("add_damping").value
        self.damping_gains = np.array(
            self.get_parameter("damping_gains").value, dtype=float
        )

        # Load URDF and build Pinocchio model
        if not urdf_path:
            # Try to find URDF in workspace
            urdf_path = self._find_urdf()

        self.get_logger().info(f"Loading URDF from: {urdf_path}")
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()

        self.get_logger().info(
            f"Loaded model with {self.model.nq} DOF, {self.model.njoints} joints"
        )

        # Joint state storage
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
        self.joint_state_received = False

        # Publishers and subscribers
        self.effort_pub = self.create_publisher(
            Float64MultiArray, "/effort_controller/commands", 10
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Control timer
        control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info(
            f"Gravity compensation controller initialized (rate: {control_rate} Hz, "
            f"gravity_scale: {self.gravity_scale}, damping: {self.add_damping})"
        )

    def _find_urdf(self) -> Path:
        """Find the URDF file in the workspace."""
        # Try common locations
        possible_paths = [
            Path.home()
            / "ros2_ws/install/openarm_description/share/openarm_description/urdf/robot/v10.urdf",
            Path.home()
            / "ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro",
        ]

        for path in possible_paths:
            if path.exists():
                self.get_logger().info(f"Found URDF at: {path}")
                return path

        # If not found, try to generate from xacro
        xacro_path = (
            Path.home()
            / "ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro"
        )
        if xacro_path.exists():
            self.get_logger().info(f"Generating URDF from xacro: {xacro_path}")
            import subprocess
            import tempfile

            with tempfile.NamedTemporaryFile(
                mode="w", suffix=".urdf", delete=False
            ) as f:
                urdf_path = Path(f.name)

            # Generate URDF using xacro
            cmd = [
                "xacro",
                str(xacro_path),
                "ros2_control:=false",
                "use_mujoco:=false",
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)

            urdf_path.write_text(result.stdout)
            self.get_logger().info(f"Generated URDF at: {urdf_path}")
            return urdf_path

        raise FileNotFoundError(
            "Could not find URDF file. Please specify urdf_path parameter."
        )

    def joint_state_callback(self, msg: JointState):
        """Update joint positions and velocities from joint_states topic."""
        try:
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.q[i] = msg.position[idx]
                    if msg.velocity:
                        self.dq[i] = msg.velocity[idx]

            self.joint_state_received = True

        except Exception as e:
            self.get_logger().error(f"Error in joint_state_callback: {e}")

    def control_loop(self):
        """Compute and publish gravity compensation torques."""
        if not self.joint_state_received:
            self.get_logger().warn(
                "Waiting for joint states...", throttle_duration_sec=2.0
            )
            return

        try:
            # Compute gravity torques using Pinocchio
            q_pin = pin.neutral(self.model)
            q_pin[: len(self.q)] = self.q

            tau_gravity = pin.computeGeneralizedGravity(self.model, self.data, q_pin)

            # Extract only the actuated joints (first 7 DOF)
            tau_gravity = tau_gravity[: len(self.joint_names)]

            # Apply gravity scale (for tuning if URDF masses are inaccurate)
            tau_gravity *= self.gravity_scale

            # Add damping to stabilize (optional but recommended)
            tau_damping = np.zeros(7)
            if self.add_damping:
                tau_damping = -self.damping_gains * self.dq

            # Total torque command
            tau_total = tau_gravity + tau_damping

            # Publish effort commands
            msg = Float64MultiArray()
            msg.data = tau_total.tolist()
            self.effort_pub.publish(msg)

            # Log periodically
            if not hasattr(self, "_log_counter"):
                self._log_counter = 0
            self._log_counter += 1

            if self._log_counter % 100 == 0:  # Log every ~1 second at 100Hz
                self.get_logger().info(
                    f"Gravity torques: [{', '.join(f'{t:6.2f}' for t in tau_gravity)}] Nm"
                )
                if self.add_damping:
                    self.get_logger().info(
                        f"Damping torques: [{', '.join(f'{t:6.2f}' for t in tau_damping)}] Nm"
                    )

        except Exception as e:
            self.get_logger().error(f"Error in control_loop: {e}")


def main():
    """Run the gravity compensation controller."""
    rclpy.init()

    try:
        controller = GravityCompensationController()
        rclpy.spin(controller)

    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down gravity compensation controller")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

