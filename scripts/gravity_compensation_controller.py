#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Gravity Compensation Controller for OpenARM in MuJoCo simulation.

This is a STANDALONE testing/prototyping tool that demonstrates external gravity
compensation using Pinocchio dynamics. It publishes effort commands directly to
ForwardCommandController (effort_controller).

**NOTE**: This is for testing and system identification purposes.
For production control with the OpenARM, use zordi_mit_controller which has
integrated gravity compensation via Pinocchio, plus MIT-mode PD control.

This standalone approach is useful for:
- Validating Pinocchio model matches MuJoCo simulation
- Debugging gravity compensation algorithms
- System identification
- Educational purposes

Key Implementation Details:
    - Computes gravity torques g(q) using Pinocchio (NOT from MuJoCo's qfrc_bias)
    - Adds damping for stability
    - Optionally includes Coriolis compensation for moving scenarios
    - Matches real hardware control approach

Usage:
    # Terminal 1: Start simulation with effort control
    export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
    ros2 launch openarm_description mujoco_sim.launch.py

    # Terminal 2: Switch to effort controller
    ros2 control switch_controllers --activate effort_controller \
        --deactivate joint_trajectory_controller

    # Terminal 3: Run this controller
    python3 scripts/gravity_compensation_controller.py

    # The robot should now hold its position under gravity!

MuJoCo Dynamics Equation (NON-STANDARD):
    M(q)·q̈ = qfrc_passive - qfrc_bias + qfrc_applied

    Note the MINUS before qfrc_bias! This is MuJoCo-specific.

    For gravity compensation: qfrc_applied = +qfrc_bias (POSITIVE sign)

Standard Robotics Convention (Pinocchio, most textbooks):
    M(q)·q̈ + C(q,q̇)·q̇ + g(q) = τ

    Where g(q) is the generalized gravity force. Pinocchio's computeGeneralizedGravity
    returns g(q) in this form. For gravity compensation: τ = +g(q) (POSITIVE sign)

    Both MuJoCo and Pinocchio use POSITIVE sign - they're consistent!
"""

import csv
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import pinocchio as pin


class GravityCompensationController(Node):
    """Controller that applies gravity compensation to hold robot position.

    This controller computes feedforward torques to compensate for gravity
    (and optionally Coriolis forces) using the robot's dynamic model.
    """

    def __init__(self):
        """Initialize the gravity compensation controller."""
        super().__init__("gravity_compensation_controller")

        # Parameters
        self.declare_parameter("urdf_path", "")
        self.declare_parameter(
            "gravity_scale", 1.0
        )  # Scale gravity if needed for tuning
        self.declare_parameter("control_rate", 1000.0)  # Hz - Match ros2_control rate
        # add_damping: Disabled by default for pure gravity compensation
        # With collisions disabled and perfect model match, damping is not needed
        # and can cause instability if gains are too high
        self.declare_parameter("add_damping", False)
        self.declare_parameter("damping_gains", [1.0, 2.0, 1.5, 0.5, 0.3, 0.2, 0.2])
        self.declare_parameter("add_coriolis", False)  # For moving scenarios
        self.declare_parameter("verbose_logging", False)
        self.declare_parameter("enable_plot", True)  # Enable live visualization
        self.declare_parameter(
            "use_mujoco_qfrc_bias", False
        )  # SANITY CHECK: bypass Pinocchio, use MuJoCo directly
        self.declare_parameter("enable_csv_logging", True)  # Log to CSV for analysis

        urdf_path = self.get_parameter("urdf_path").value
        self.gravity_scale = self.get_parameter("gravity_scale").value
        control_rate = self.get_parameter("control_rate").value
        self.add_damping = self.get_parameter("add_damping").value
        self.damping_gains = np.array(
            self.get_parameter("damping_gains").value, dtype=float
        )
        self.add_coriolis = self.get_parameter("add_coriolis").value
        self.verbose_logging = self.get_parameter("verbose_logging").value
        self.enable_plot = self.get_parameter("enable_plot").value
        self.use_mujoco_qfrc_bias = self.get_parameter("use_mujoco_qfrc_bias").value
        self.enable_csv_logging = self.get_parameter("enable_csv_logging").value

        # CSV logging setup
        self.csv_file = None
        self.csv_writer = None
        if self.enable_csv_logging:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_filename = f"gravity_comp_log_{timestamp}.csv"
            self.csv_file = open(csv_filename, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            # Header
            header = ["timestamp", "mode"]
            for i in range(1, 8):
                header.extend([
                    f"q{i}",
                    f"dq{i}",
                    f"mujoco_qfrc_bias{i}",
                    f"tau_gravity{i}",
                    f"tau_damping{i}",
                    f"tau_coriolis{i}",
                    f"tau_total{i}",
                ])
            self.csv_writer.writerow(header)
            self.get_logger().info(f"📝 CSV logging enabled: {csv_filename}")

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

        # Print model parameters for debugging
        self._print_model_parameters()

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

        # Data history for plotting (keep last 500 samples = ~5 seconds at 100Hz)
        self.max_history = 500
        self.time_history = []
        self.gravity_history = [[] for _ in range(7)]
        self.coriolis_history = [[] for _ in range(7)]
        self.damping_history = [[] for _ in range(7)]
        self.total_history = [[] for _ in range(7)]
        self.position_history = [[] for _ in range(7)]
        self.velocity_history = [[] for _ in range(7)]
        self.mujoco_qfrc_bias_history = [[] for _ in range(7)]  # MuJoCo's computation
        self.start_time = self.get_clock().now()

        # MuJoCo qfrc_bias storage for comparison
        self.mujoco_qfrc_bias = np.zeros(7)
        self.qfrc_bias_received = False
        self.qfrc_bias_timestamp = None

        # Publishers and subscribers
        self.effort_pub = self.create_publisher(
            Float64MultiArray, "/effort_controller/commands", 10
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Subscribe to MuJoCo's qfrc_bias for comparison
        self.qfrc_bias_sub = self.create_subscription(
            Float64MultiArray, "/mujoco/qfrc_bias", self.qfrc_bias_callback, 10
        )

        # Control timer
        control_period = 1.0 / control_rate
        self.control_timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info("Gravity compensation controller initialized:")
        self.get_logger().info(f"  Control rate: {control_rate} Hz")
        self.get_logger().info(f"  Gravity scale: {self.gravity_scale}")
        self.get_logger().info(f"  Damping: {self.add_damping}")
        self.get_logger().info(f"  Coriolis compensation: {self.add_coriolis}")
        self.get_logger().info(f"  Verbose logging: {self.verbose_logging}")
        if self.use_mujoco_qfrc_bias:
            self.get_logger().warn(
                "⚠️  SANITY CHECK MODE: Using MuJoCo qfrc_bias directly "
                "(bypassing Pinocchio)"
            )
        self.get_logger().info(f"  Live plotting: {self.enable_plot}")

        # Initialize live plot if enabled
        if self.enable_plot:
            self._setup_plot()

    def _find_urdf(self) -> Path:
        """Find the URDF file in the workspace."""
        # Try common locations for processed URDF files
        possible_paths = [
            Path.home()
            / "ros2_ws/install/openarm_description/share"
            / "openarm_description/urdf/robot/v10.urdf",
            # Note: Don't look for .xacro files here - they need processing first
        ]

        for path in possible_paths:
            if path.exists():
                self.get_logger().info(f"Found URDF at: {path}")
                return path

        # No processed URDF found, generate from xacro
        xacro_path = (
            Path.home() / "ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro"
        )
        if xacro_path.exists():
            import os
            import subprocess
            import tempfile

            self.get_logger().info(f"Generating URDF from xacro: {xacro_path}")

            with tempfile.NamedTemporaryFile(
                mode="w", suffix=".urdf", delete=False, encoding="utf-8"
            ) as f:
                urdf_path = Path(f.name)

            # Generate URDF using xacro (inherits environment)
            # CRITICAL: Must match the MJCF model configuration!
            # For v10 without hand, use hand:=false to match simulation
            cmd = [
                "xacro",
                str(xacro_path),
                "ros2_control:=false",
                "use_mujoco:=false",
                "hand:=false",  # MUST match simulation (no hand = lighter link 7)
            ]

            try:
                result = subprocess.run(
                    cmd, capture_output=True, text=True, check=True, env=os.environ
                )
                urdf_path.write_text(result.stdout, encoding="utf-8")
                self.get_logger().info(f"Generated URDF at: {urdf_path}")
                return urdf_path

            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Failed to generate URDF from xacro: {e}")
                self.get_logger().error(f"stderr: {e.stderr}")
                self.get_logger().error(
                    "Make sure you sourced the workspace: "
                    "source ~/ros2_ws/install/setup.bash"
                )
                raise

        raise FileNotFoundError(
            "Could not find URDF file. Please specify urdf_path parameter."
        )

    def _print_model_parameters(self):
        """Print detailed model parameters to understand gravity computation."""
        self.get_logger().info("=" * 80)
        self.get_logger().info("PINOCCHIO MODEL PARAMETERS (from URDF)")
        self.get_logger().info("=" * 80)

        # Iterate through bodies (links)
        for i in range(1, self.model.njoints):  # Skip universe (0)
            body_name = self.model.names[i]
            mass = self.model.inertias[i].mass

            # Get center of mass in local frame
            com_local = self.model.inertias[i].lever

            # Get inertia tensor
            inertia = self.model.inertias[i].inertia

            self.get_logger().info(f"\nLink {i} ({body_name}):")
            self.get_logger().info(f"  Mass: {mass:.6f} kg")
            self.get_logger().info(
                f"  COM (local): [{com_local[0]:.6f}, {com_local[1]:.6f}, {com_local[2]:.6f}] m"
            )
            self.get_logger().info(
                f"  Inertia xx={inertia[0, 0]:.6f}, yy={inertia[1, 1]:.6f}, zz={inertia[2, 2]:.6f} kg·m²"
            )

            # Rough gravity torque estimate (for horizontal link)
            # τ ≈ m * g * r_com, where r_com is distance from joint to COM
            approx_torque = mass * 9.81 * np.linalg.norm(com_local)
            self.get_logger().info(
                f"  → Approx max gravity torque: {approx_torque:.3f} Nm "
                f"(if link is horizontal)"
            )

        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info(
            "NOTE: These are Pinocchio's view. Compare with MuJoCo XML to check match!"
        )
        self.get_logger().info("=" * 80)

    def _compute_gravity_breakdown(self, q):
        """Compute gravity with detailed breakdown showing the physics.

        This helps understand WHY gravity is what it is.
        """
        # Update forward kinematics to get positions
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        # Compute gravity vector in world frame
        gravity_vector = np.array([0, 0, -9.81])

        breakdown = []

        for i in range(1, min(4, self.model.njoints)):  # First 3 joints
            body_name = self.model.names[i]
            mass = self.model.inertias[i].mass

            # Get COM position in world frame
            # This is where the physics happens: τ = m * g × r
            joint_placement = self.data.oMi[i]  # Joint frame in world
            com_local = self.model.inertias[i].lever  # COM in local frame
            com_world = (
                joint_placement.translation + joint_placement.rotation @ com_local
            )

            # Moment arm: vector from joint to COM
            r_com = com_world - joint_placement.translation

            # Gravitational force at COM
            F_gravity = mass * gravity_vector

            # Torque = r × F (cross product)
            # This is the fundamental equation!
            torque_contribution = np.cross(r_com, F_gravity)

            # Project onto joint axis
            joint_axis = joint_placement.rotation @ np.array([
                0,
                0,
                1,
            ])  # Assuming Z-axis joints
            tau = np.dot(torque_contribution, joint_axis)

            breakdown.append({
                "joint": i - 1,
                "name": body_name,
                "mass": mass,
                "r_com": r_com,
                "r_com_magnitude": np.linalg.norm(r_com),
                "F_gravity": F_gravity,
                "torque_contribution": torque_contribution,
                "tau": tau,
            })

        return breakdown

    def _setup_plot(self):
        """Initialize the live plotting window."""
        plt.ion()  # Enable interactive mode
        self.fig, self.axes = plt.subplots(2, 3, figsize=(18, 10))
        self.fig.suptitle(
            "Gravity Compensation Diagnostics - Check Terminal for qfrc_bias Comparison",
            fontsize=14,
            fontweight="bold",
        )

        # Plot first 3 joints (most important for gravity)
        self.plot_joints = [0, 1, 2]  # joints 1, 2, 3

        # Initialize line objects for each subplot
        self.lines = {}

        # Row 1: Commanded torque breakdown (what we're sending)
        for idx, joint_idx in enumerate(self.plot_joints):
            ax = self.axes[0, idx]
            ax.set_title(
                f"Joint {joint_idx + 1} - Command vs MuJoCo\n"
                f"(Total cmd should match -qfrc_bias)",
                fontsize=10,
            )
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Torque (Nm)")
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color="gray", linestyle=":", linewidth=0.8)

            (self.lines[f"grav_{joint_idx}"],) = ax.plot(
                [], [], "r-", label="Pinocchio Gravity", linewidth=2.5, alpha=0.7
            )
            (self.lines[f"damp_{joint_idx}"],) = ax.plot(
                [], [], "b-", label="Damping", linewidth=1.5, alpha=0.6
            )
            if self.add_coriolis:
                (self.lines[f"cori_{joint_idx}"],) = ax.plot(
                    [], [], "g-", label="Coriolis", linewidth=1.5, alpha=0.6
                )
            (self.lines[f"total_{joint_idx}"],) = ax.plot(
                [], [], "k-", label="TOTAL CMD (Pinocchio)", linewidth=2.5
            )
            (self.lines[f"mujoco_{joint_idx}"],) = ax.plot(
                [],
                [],
                "orange",
                linestyle="--",
                label="-qfrc_bias (MuJoCo)",
                linewidth=2.5,
            )
            ax.legend(loc="best", fontsize=8)

        # Row 2: Joint configuration (to understand expected gravity)
        for idx, joint_idx in enumerate(self.plot_joints):
            ax = self.axes[1, idx]
            ax.set_title(
                f"Joint {joint_idx + 1} - Position (Configuration)", fontsize=10
            )
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Position (rad)")
            ax.grid(True, alpha=0.3)
            ax.axhline(y=0, color="gray", linestyle=":", linewidth=0.8)

            (self.lines[f"pos_{joint_idx}"],) = ax.plot(
                [], [], "m-", label=f"q{joint_idx + 1}", linewidth=2
            )
            ax.legend(loc="best", fontsize=8)

        plt.tight_layout()
        plt.show(block=False)

    def _update_plot_data(
        self, current_time, tau_gravity, tau_coriolis, tau_damping, tau_total
    ):
        """Update the data history for plotting."""
        if not self.enable_plot:
            return

        # Add new data point
        self.time_history.append(current_time)

        for i in range(7):
            self.gravity_history[i].append(tau_gravity[i])
            self.coriolis_history[i].append(tau_coriolis[i])
            self.damping_history[i].append(tau_damping[i])
            self.total_history[i].append(tau_total[i])
            self.position_history[i].append(self.q[i])
            self.velocity_history[i].append(self.dq[i])
            # Store -qfrc_bias (negative because we want to cancel it)
            self.mujoco_qfrc_bias_history[i].append(-self.mujoco_qfrc_bias[i])

        # Keep only recent data
        if len(self.time_history) > self.max_history:
            self.time_history.pop(0)
            for i in range(7):
                self.gravity_history[i].pop(0)
                self.coriolis_history[i].pop(0)
                self.damping_history[i].pop(0)
                self.total_history[i].pop(0)
                self.position_history[i].pop(0)
                self.velocity_history[i].pop(0)
                self.mujoco_qfrc_bias_history[i].pop(0)

    def _refresh_plot(self):
        """Refresh the matplotlib plot with new data."""
        if not self.enable_plot or len(self.time_history) < 2:
            return

        try:
            # Update torque plots
            for idx, joint_idx in enumerate(self.plot_joints):
                # Update line data
                self.lines[f"grav_{joint_idx}"].set_data(
                    self.time_history, self.gravity_history[joint_idx]
                )
                self.lines[f"damp_{joint_idx}"].set_data(
                    self.time_history, self.damping_history[joint_idx]
                )
                if self.add_coriolis:
                    self.lines[f"cori_{joint_idx}"].set_data(
                        self.time_history, self.coriolis_history[joint_idx]
                    )
                self.lines[f"total_{joint_idx}"].set_data(
                    self.time_history, self.total_history[joint_idx]
                )
                # MuJoCo qfrc_bias (negated)
                self.lines[f"mujoco_{joint_idx}"].set_data(
                    self.time_history, self.mujoco_qfrc_bias_history[joint_idx]
                )

                # Update position
                self.lines[f"pos_{joint_idx}"].set_data(
                    self.time_history, self.position_history[joint_idx]
                )

                # Auto-scale axes
                for row in range(2):
                    self.axes[row, idx].relim()
                    self.axes[row, idx].autoscale_view()

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

        except Exception as e:
            self.get_logger().warn(f"Plot update error: {e}", throttle_duration_sec=5.0)

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

    def qfrc_bias_callback(self, msg: Float64MultiArray):
        """Store MuJoCo's qfrc_bias for comparison with Pinocchio."""
        try:
            if len(msg.data) >= 7:
                self.mujoco_qfrc_bias = np.array(msg.data[:7])
                self.qfrc_bias_received = True
                self.qfrc_bias_timestamp = self.get_clock().now()
        except Exception as e:
            self.get_logger().error(f"Error in qfrc_bias_callback: {e}")

    def control_loop(self):
        """Compute and publish gravity compensation torques.

        Computes feedforward torques to compensate for:
            - Gravity: g(q)
            - Damping: -Kd * q̇ (optional, for stability)
            - Coriolis: C(q,q̇) * q̇ (optional, for moving scenarios)

        SANITY CHECK MODE:
            If use_mujoco_qfrc_bias=True, bypasses Pinocchio and uses
            MuJoCo's qfrc_bias directly. This should give PERFECT gravity
            compensation since it's the exact model being simulated.
        """
        if not self.joint_state_received:
            self.get_logger().warn(
                "Waiting for joint states...", throttle_duration_sec=2.0
            )
            return

        # SANITY CHECK MODE: Use MuJoCo's qfrc_bias directly (bypass Pinocchio)
        # This is a PURE test of the control pipeline - NO damping, NO modifications
        # Physics: M·q̈ = qfrc_applied + qfrc_bias
        # For zero acceleration: qfrc_applied = -qfrc_bias
        if self.use_mujoco_qfrc_bias:
            if not self.qfrc_bias_received:
                self.get_logger().warn(
                    "Waiting for MuJoCo qfrc_bias...", throttle_duration_sec=2.0
                )
                return

            # MuJoCo Sign Convention (NON-STANDARD):
            #   MuJoCo equation: M·q̈ = qfrc_passive - qfrc_bias + qfrc_applied
            #   For gravity comp: qfrc_applied = +qfrc_bias (POSITIVE!)
            #
            # Standard robotics (ROS, Pinocchio, textbooks) use NEGATIVE:
            #   Standard equation: M·q̈ = τ_cmd + τ_gravity
            #   For gravity comp: τ_cmd = -τ_gravity (NEGATIVE)
            #
            # MuJoCo is different because it subtracts qfrc_bias in the equation,
            # so we compensate by applying it with POSITIVE sign.
            tau_total = +self.mujoco_qfrc_bias.copy()

            # Store for logging (but don't use in command)
            tau_gravity_mujoco = tau_total.copy()
            tau_damping = np.zeros(7)
            tau_coriolis = np.zeros(7)

            # Publish
            msg = Float64MultiArray()
            msg.data = tau_total.tolist()
            self.effort_pub.publish(msg)

            # CSV logging
            if self.csv_writer:
                current_time = (
                    self.get_clock().now() - self.start_time
                ).nanoseconds / 1e9
                row = [current_time, "mujoco_direct"]
                for i in range(7):
                    row.extend([
                        self.q[i],
                        self.dq[i],
                        self.mujoco_qfrc_bias[i],
                        tau_gravity_mujoco[i],
                        tau_damping[i],
                        tau_coriolis[i],
                        tau_total[i],
                    ])
                self.csv_writer.writerow(row)
                self.csv_file.flush()

            # Log occasionally
            if not hasattr(self, "_log_counter"):
                self._log_counter = 0
            self._log_counter += 1
            if self._log_counter % 100 == 0:
                # Check data freshness
                age_ms = 0
                if self.qfrc_bias_timestamp:
                    age_ms = (
                        self.get_clock().now() - self.qfrc_bias_timestamp
                    ).nanoseconds / 1e6

                self.get_logger().info(
                    "🔧 MuJoCo qfrc_bias mode: cmd = +qfrc_bias | "
                    f"J1={tau_total[0]:.3f} Nm, "
                    f"J2={tau_total[1]:.3f} Nm, "
                    f"J3={tau_total[2]:.3f} Nm | "
                    f"dq=[{self.dq[0]:.3f}, {self.dq[1]:.3f}, {self.dq[2]:.3f}] rad/s"
                )
            return

        try:
            # Compute gravity torques using Pinocchio
            q_pin = pin.neutral(self.model)
            q_pin[: len(self.q)] = self.q

            # Update kinematics for Coriolis computation
            if self.add_coriolis:
                dq_pin = np.zeros(self.model.nv)
                dq_pin[: len(self.dq)] = self.dq
                pin.computeAllTerms(self.model, self.data, q_pin, dq_pin)

            tau_gravity = pin.computeGeneralizedGravity(self.model, self.data, q_pin)

            # Extract only the actuated joints (first 7 DOF)
            tau_gravity = tau_gravity[: len(self.joint_names)]

            # Apply gravity scale (for tuning if URDF masses are inaccurate)
            tau_gravity *= self.gravity_scale

            # Add Coriolis compensation (for moving scenarios)
            tau_coriolis = np.zeros(7)
            if self.add_coriolis:
                # Coriolis matrix already computed in computeAllTerms
                coriolis_matrix = pin.computeCoriolisMatrix(
                    self.model, self.data, q_pin, dq_pin
                )
                tau_coriolis = (coriolis_matrix @ dq_pin)[: len(self.joint_names)]

            # Add damping to stabilize (optional but recommended)
            tau_damping = np.zeros(7)
            if self.add_damping:
                tau_damping = -self.damping_gains * self.dq

            # Total torque command
            tau_total = tau_gravity + tau_coriolis + tau_damping

            # Publish effort commands
            msg = Float64MultiArray()
            msg.data = tau_total.tolist()
            self.effort_pub.publish(msg)

            # CSV logging at 1000 Hz (every control loop iteration)
            if self.csv_writer:
                current_time = (
                    self.get_clock().now() - self.start_time
                ).nanoseconds / 1e9
                row = [current_time, "pinocchio"]
                for i in range(7):
                    row.extend([
                        self.q[i],
                        self.dq[i],
                        self.mujoco_qfrc_bias[i],
                        tau_gravity[i],
                        tau_damping[i],
                        tau_coriolis[i],
                        tau_total[i],
                    ])
                self.csv_writer.writerow(row)
                # Flush every iteration for real-time debugging (1000 Hz)
                # This ensures we capture data even if process crashes
                self.csv_file.flush()

            # Update plot data
            current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self._update_plot_data(
                current_time, tau_gravity, tau_coriolis, tau_damping, tau_total
            )

            # Log periodically
            if not hasattr(self, "_log_counter"):
                self._log_counter = 0
            self._log_counter += 1

            log_interval = 10 if self.verbose_logging else 100
            if self._log_counter % log_interval == 0:
                # Print detailed comparison for first 3 joints
                self.get_logger().info("\n" + "=" * 80)
                self.get_logger().info("GRAVITY COMPENSATION DIAGNOSTIC")
                qfrc_status = (
                    "✓ Receiving" if self.qfrc_bias_received else "✗ NOT receiving"
                )
                self.get_logger().info(f"MuJoCo qfrc_bias: {qfrc_status}")
                self.get_logger().info("=" * 80)

                # Get detailed physics breakdown
                breakdown = self._compute_gravity_breakdown(q_pin)

                for i in range(min(3, len(self.joint_names))):  # First 3 joints
                    self.get_logger().info(
                        f"\n--- Joint {i + 1} ({self.joint_names[i]}) ---"
                    )
                    self.get_logger().info(
                        f"Configuration: q={self.q[i]:6.3f} rad ({np.degrees(self.q[i]):6.1f}°), "
                        f"q̇={self.dq[i]:6.3f} rad/s"
                    )

                    # Physics breakdown if available
                    if i < len(breakdown):
                        b = breakdown[i]
                        self.get_logger().info("\nPhysics Calculation:")
                        self.get_logger().info(f"  • Link mass: {b['mass']:.4f} kg")
                        self.get_logger().info(
                            f"  • Moment arm (joint→COM): {b['r_com_magnitude']:.4f} m"
                        )
                        self.get_logger().info(
                            f"    Position: [{b['r_com'][0]:.4f}, {b['r_com'][1]:.4f}, {b['r_com'][2]:.4f}]"
                        )
                        self.get_logger().info(
                            f"  • Gravitational force: [{b['F_gravity'][0]:.3f}, "
                            f"{b['F_gravity'][1]:.3f}, {b['F_gravity'][2]:.3f}] N"
                        )
                        self.get_logger().info(
                            f"  • Torque = r × F: {b['tau']:.4f} Nm (simplified)"
                        )

                    self.get_logger().info("\nTorque Breakdown:")
                    self.get_logger().info(
                        f"  • Pinocchio gravity: {tau_gravity[i]:7.3f} Nm  ← From full dynamics"
                    )
                    self.get_logger().info(
                        f"  • Damping:          {tau_damping[i]:7.3f} Nm  ← -Kd·q̇"
                    )
                    if self.add_coriolis:
                        self.get_logger().info(
                            f"  • Coriolis:         {tau_coriolis[i]:7.3f} Nm"
                        )
                    self.get_logger().info(
                        f"  • TOTAL cmd:        {tau_total[i]:7.3f} Nm  ← Sent to MuJoCo"
                    )

                    # DIRECT COMPARISON with MuJoCo
                    if self.qfrc_bias_received:
                        # MuJoCo uses +qfrc_bias (positive), so compare with that
                        # NOTE: qfrc_bias includes gravity + Coriolis + centrifugal
                        # If robot is moving and add_coriolis=False, expect mismatch!
                        mujoco_expected = +self.mujoco_qfrc_bias[i]
                        error = tau_total[i] - mujoco_expected
                        error_pct = (abs(error) / max(abs(mujoco_expected), 0.01)) * 100

                        self.get_logger().info("\n  📊 COMPARISON:")
                        self.get_logger().info(
                            f"     Pinocchio cmd:     {tau_total[i]:7.3f} Nm"
                        )
                        self.get_logger().info(
                            f"     MuJoCo +qfrc_bias: {mujoco_expected:7.3f} Nm"
                        )
                        self.get_logger().info(
                            f"     Error:             {error:7.3f} Nm ({error_pct:.1f}%)"
                        )
                        if not self.add_coriolis and np.max(np.abs(self.dq)) > 0.01:
                            self.get_logger().info(
                                "     ⚠️  Robot moving but Coriolis OFF - expect mismatch!"
                            )

                        if abs(error) < 0.5:
                            self.get_logger().info(
                                "     ✓ MATCH! Gravity comp working correctly"
                            )
                        else:
                            self.get_logger().info(
                                "     ✗ MISMATCH! Check URDF vs MuJoCo model"
                            )
                    else:
                        self.get_logger().info(
                            "\n  ⚠️  Waiting for MuJoCo qfrc_bias data..."
                        )

                self.get_logger().info("\n" + "-" * 80)
                self.get_logger().info("DIAGNOSIS GUIDE:")
                self.get_logger().info(
                    "  If cmd >> -qfrc_bias → Pinocchio sees MORE mass than MuJoCo"
                )
                self.get_logger().info(
                    "  If cmd << -qfrc_bias → Pinocchio sees LESS mass than MuJoCo"
                )
                self.get_logger().info(
                    "  Check: URDF inertial tags vs MuJoCo XML <body> tags"
                )
                self.get_logger().info("=" * 80 + "\n")

                # Refresh plot every log interval
                if self.enable_plot:
                    self._refresh_plot()

        except Exception as e:
            self.get_logger().error(f"Error in control_loop: {e}")


def main():
    """Run the gravity compensation controller."""
    rclpy.init()
    controller = None

    try:
        controller = GravityCompensationController()
        rclpy.spin(controller)

    except KeyboardInterrupt:
        if controller:
            controller.get_logger().info(
                "Shutting down gravity compensation controller"
            )

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Clean up CSV file
        if controller and controller.csv_file:
            controller.csv_file.close()
            print(f"CSV log file closed: {controller.csv_file.name}")
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
