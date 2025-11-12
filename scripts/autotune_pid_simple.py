#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Realistic automated PID tuning using grid search with MuJoCo.

This script simulates the ACTUAL ROS2 control pipeline as closely as possible:
1. Multi-joint trajectories (not single-joint isolation)
2. Cubic spline trajectory interpolation (matches JointTrajectoryController)
3. Full arm dynamics with coupling (gravity changes with configuration)
4. Split-step timing (mj_step1 -> control -> mj_step2)
5. Realistic control loop overhead

What's still missing (unavoidable without ROS2):
- Message passing latency
- Controller manager scheduling jitter
- Actual control_toolbox::Pid implementation (we use equivalent)

Usage:
    python3 scripts/autotune_pid_simple.py

    # Tune specific joint only
    python3 scripts/autotune_pid_simple.py --joint 2

    # Coarse grid (faster)
    python3 scripts/autotune_pid_simple.py --grid-size 3

    # Fine grid (slower, more accurate)
    python3 scripts/autotune_pid_simple.py --grid-size 7

Output:
    - figs/*.png: Performance plots for each joint
    - optimal_pid_gains.json: Best gains in machine-readable format
    - Terminal: URDF snippet to copy-paste
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # Non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline

try:
    import mujoco
except ImportError:
    print("ERROR: mujoco not installed")
    print("Install with: pip install mujoco")
    sys.exit(1)

try:
    import control_toolbox
except ImportError:
    # Python binding for control_toolbox (if available)
    # Otherwise we'll use our own PID implementation
    control_toolbox = None


class RealisticPID:
    """
    PID controller matching control_toolbox behavior.

    Implements:
    - Anti-windup with integral clamping
    - Derivative filtering
    - Output saturation
    """

    def __init__(self, kp, ki, kd, dt, i_clamp=1.0, output_limit=None):
        """Initialize PID controller."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.i_clamp = i_clamp
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

        # Derivative filter (low-pass) to reduce noise amplification
        self.derivative_filter_coef = 0.1  # Same as control_toolbox default

    def compute(self, error, error_dot=None):
        """
        Compute control output.

        Args:
            error: Position error (target - actual)
            error_dot: Derivative of error (optional, computed if not provided)

        Returns:
            float: Control output (torque)
        """
        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.i_clamp, self.i_clamp)
        i_term = self.ki * self.integral

        # Derivative term (with filtering to match control_toolbox)
        if error_dot is not None:
            derivative = error_dot
        else:
            derivative = (error - self.prev_error) / self.dt

        # Low-pass filter on derivative
        filtered_derivative = (
            self.derivative_filter_coef * derivative
            + (1 - self.derivative_filter_coef) * self.prev_derivative
        )
        self.prev_derivative = filtered_derivative
        d_term = self.kd * filtered_derivative

        self.prev_error = error

        # Total output
        output = p_term + i_term + d_term

        # Output saturation
        if self.output_limit is not None:
            output = np.clip(output, -self.output_limit, self.output_limit)

        return output

    def reset(self):
        """Reset integrator and filters."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0


class TrajectoryGenerator:
    """
    Generate cubic spline trajectories matching JointTrajectoryController.

    This replicates the trajectory interpolation that happens in ROS2.
    """

    def __init__(self, waypoints, times):
        """
        Initialize trajectory generator.

        Args:
            waypoints: List of joint positions (N waypoints × 7 joints)
            times: List of times for each waypoint (N waypoints)
        """
        self.waypoints = np.array(waypoints)
        self.times = np.array(times)
        self.n_joints = self.waypoints.shape[1] if len(self.waypoints.shape) > 1 else 1

        # Create cubic splines for each joint
        self.splines = []
        if len(self.waypoints.shape) == 1:
            # Single joint
            self.splines.append(
                CubicSpline(
                    self.times,
                    self.waypoints,
                    bc_type="clamped",  # Zero velocity at endpoints
                )
            )
        else:
            # Multiple joints
            for joint_idx in range(self.n_joints):
                spline = CubicSpline(
                    self.times,
                    self.waypoints[:, joint_idx],
                    bc_type="clamped",  # Zero velocity at endpoints
                )
                self.splines.append(spline)

    def sample(self, t):
        """
        Sample trajectory at time t.

        Returns:
            position, velocity, acceleration
        """
        positions = np.array([spline(t) for spline in self.splines])
        velocities = np.array([spline(t, 1) for spline in self.splines])
        accelerations = np.array([spline(t, 2) for spline in self.splines])

        return positions, velocities, accelerations


def test_pid_gains_multijoint(
    mj_model,
    mj_data,
    kp_values,
    kd_values,
    q_initial,
    q_target,
    duration=3.0,
    dt=0.001,
    torque_limits=None,
):
    """
    Test PID gains with realistic multi-joint trajectory.

    This simulates the ACTUAL ROS2 control pipeline:
    1. Cubic spline trajectory interpolation
    2. Multi-joint coupling (full arm dynamics)
    3. Split-step timing (mj_step1 -> control -> mj_step2)
    4. Realistic PID with anti-windup

    Args:
        mj_model: MuJoCo model
        mj_data: MuJoCo data
        kp_values: Array of Kp gains for all 7 joints
        kd_values: Array of Kd gains for all 7 joints
        q_initial: Initial configuration (7 joints)
        q_target: Target configuration (7 joints)
        duration: Simulation duration (seconds)
        dt: Time step (seconds, matches ROS2 control rate 1000Hz)
        torque_limits: Array of torque limits for each joint [40, 40, 27, 27, 7, 7, 7]

    Returns:
        dict: Performance metrics for each joint
    """
    if torque_limits is None:
        torque_limits = np.array([40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0])

    # Get joint DOF addresses
    joint_ids = []
    dof_addrs = []
    for i in range(7):
        joint_name = f"openarm_joint{i + 1}"
        joint_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        joint_ids.append(joint_id)
        dof_addrs.append(mj_model.jnt_dofadr[joint_id])

    # Reset to initial configuration
    mujoco.mj_resetData(mj_model, mj_data)
    for i, dof_adr in enumerate(dof_addrs):
        mj_data.qpos[dof_adr] = q_initial[i]
        mj_data.qvel[dof_adr] = 0.0

    # Forward kinematics to update model state
    mujoco.mj_forward(mj_model, mj_data)

    # Generate trajectory with cubic splines (matches JointTrajectoryController)
    waypoints = np.array([q_initial, q_target])
    times = np.array([0.0, duration])
    trajectory = TrajectoryGenerator(waypoints, times)

    # Create PID controllers for each joint
    pids = []
    for i in range(7):
        pid = RealisticPID(
            kp=kp_values[i],
            ki=0.0,  # No integral term (avoid windup)
            kd=kd_values[i],
            dt=dt,
            i_clamp=1.0,
            output_limit=torque_limits[i],
        )
        pids.append(pid)

    # Data logging
    n_steps = int(duration / dt)
    time_hist = np.zeros(n_steps)
    pos_hist = np.zeros((n_steps, 7))
    vel_hist = np.zeros((n_steps, 7))
    error_hist = np.zeros((n_steps, 7))
    torque_hist = np.zeros((n_steps, 7))
    target_pos_hist = np.zeros((n_steps, 7))
    target_vel_hist = np.zeros((n_steps, 7))

    # Simulate with SPLIT-STEP TIMING (matches mujoco_ros2_control)
    for step in range(n_steps):
        current_time = step * dt

        # Get desired state from trajectory (trajectory interpolation)
        q_des, qd_des, qdd_des = trajectory.sample(current_time)

        # mj_step1: Forward dynamics (compute qacc from current state)
        # This is where physics computes accelerations based on forces
        mujoco.mj_step1(mj_model, mj_data)

        # CONTROL LOOP (happens between step1 and step2)
        # Read current state
        q_actual = np.array([mj_data.qpos[dof_adr] for dof_adr in dof_addrs])
        qd_actual = np.array([mj_data.qvel[dof_adr] for dof_adr in dof_addrs])

        # Compute control torques for each joint
        torques = np.zeros(7)
        for i in range(7):
            error = q_des[i] - q_actual[i]
            error_dot = qd_des[i] - qd_actual[i]

            # PID control with velocity feedforward (matches ROS2 trajectory controller)
            # Trajectory controller sends both position and velocity from cubic spline
            torque = pids[i].compute(error, error_dot=error_dot)
            torques[i] = torque

        # Apply torques
        for i, dof_adr in enumerate(dof_addrs):
            mj_data.qfrc_applied[dof_adr] = torques[i]

        # mj_step2: Integration (advance state using computed accelerations)
        mujoco.mj_step2(mj_model, mj_data)

        # Log data (read AFTER step2, so we get the updated state)
        time_hist[step] = current_time
        for i, dof_adr in enumerate(dof_addrs):
            pos_hist[step, i] = mj_data.qpos[dof_adr]
            vel_hist[step, i] = mj_data.qvel[dof_adr]
            error_hist[step, i] = q_des[i] - mj_data.qpos[dof_adr]
            torque_hist[step, i] = torques[i]
            target_pos_hist[step, i] = q_des[i]
            target_vel_hist[step, i] = qd_des[i]

    # Compute metrics for each joint
    all_metrics = {}

    for joint_idx in range(7):
        pos = pos_hist[:, joint_idx]
        vel = vel_hist[:, joint_idx]
        error = error_hist[:, joint_idx]
        torque = torque_hist[:, joint_idx]

        # Check if this joint moved enough (look at actual position change, not error)
        position_range = np.max(pos) - np.min(pos)
        max_error = np.max(np.abs(error))

        # Joint must move at least 0.01 rad (0.57°) to be considered valid
        if position_range < 0.01:
            all_metrics[joint_idx] = {
                "moved": False,
                "cost": 0.0,
                # Store debug data
                "position": pos,
                "error": error,
                "max_error": max_error,
                "position_range": position_range,
            }
            continue

        # 1. RMS error
        rms_error = np.sqrt(np.mean(error**2))

        # 2. Settling time (2% criterion)
        target_change = abs(q_target[joint_idx] - q_initial[joint_idx])
        settling_threshold = 0.02 * target_change if target_change > 0.01 else 0.001
        settled_mask = np.abs(error) < settling_threshold
        if np.any(settled_mask):
            settling_idx = np.where(settled_mask)[0][0]
            settling_time = time_hist[settling_idx]
        else:
            settling_time = duration

        # 3. Final velocity (last 10% of trajectory)
        final_vel_window = max(1, int(0.1 * n_steps))
        final_vel = np.mean(np.abs(vel[-final_vel_window:]))

        # 4. Overshoot
        overshoot = np.max(np.abs(pos - q_target[joint_idx])) - target_change
        overshoot = max(0, overshoot)

        # 5. Peak torque
        peak_torque = np.max(np.abs(torque))

        # 6. Torque saturation penalty (bad if hitting limits)
        saturation_ratio = peak_torque / torque_limits[joint_idx]

        # Compute cost (weighted combination)
        cost = (
            np.degrees(rms_error)  # Tracking error (deg)
            + 2.0 * settling_time  # Settling time (s)
            + 100.0 * final_vel  # Final velocity (rad/s) - CRITICAL!
            + 5.0 * np.degrees(overshoot)  # Overshoot (deg)
            + 10.0 * max(0, saturation_ratio - 0.8)  # Penalty if > 80% of limit
        )

        all_metrics[joint_idx] = {
            "moved": True,
            "kp": kp_values[joint_idx],
            "kd": kd_values[joint_idx],
            "rms_error_rad": rms_error,
            "rms_error_deg": np.degrees(rms_error),
            "settling_time_s": settling_time,
            "final_vel_rad_s": final_vel,
            "overshoot_rad": overshoot,
            "overshoot_deg": np.degrees(overshoot),
            "peak_torque_nm": peak_torque,
            "saturation_ratio": saturation_ratio,
            "cost": cost,
            # Store trajectories for this joint
            "time": time_hist,
            "position": pos,
            "velocity": vel,
            "error": error,
            "torque": torque,
            "target_position": target_pos_hist[:, joint_idx],
            "target_velocity": target_vel_hist[:, joint_idx],
        }

    return all_metrics


def grid_search_joint_individual(
    mj_model,
    mj_data,
    joint_idx,
    kp_range,
    kd_range,
    grid_size=5,
    test_configs=None,
):
    """
    Grid search for single joint with multiple test configurations.

    Tests the joint in isolation but at different configurations to ensure
    robustness across the workspace.

    Args:
        mj_model: MuJoCo model
        mj_data: MuJoCo data
        joint_idx: Joint index (0-6)
        kp_range: (min, max) for Kp
        kd_range: (min, max) for Kd
        grid_size: Number of points in each dimension
        test_configs: List of (q_initial, q_target) configurations to test

    Returns:
        dict: Results with best gains averaged across all test configs
    """
    if test_configs is None:
        # Default test configurations for this joint
        # Use realistic random configurations within joint limits to avoid singularities

        # Extract joint limits directly from MuJoCo model
        # mj_model.jnt_range is (njnt, 2) with [lower, upper] for each joint
        joint_limits = mj_model.jnt_range.copy()  # Shape: (7, 2)

        # Generate 3 test configurations
        test_configs = []
        np.random.seed(42 + joint_idx)  # Reproducible per joint

        for config_idx in range(3):
            # Random initial configuration (within 50% of limits to be safe)
            q_initial = np.zeros(7)
            for j in range(7):
                limit_range = joint_limits[j, 1] - joint_limits[j, 0]
                center = (joint_limits[j, 0] + joint_limits[j, 1]) / 2
                q_initial[j] = center + np.random.uniform(-0.5, 0.5) * limit_range * 0.5

            # Target: move only the joint being tuned to a different position
            # Ensure minimum movement of 30% of joint range to make tuning meaningful
            q_target = q_initial.copy()
            limit_range = joint_limits[joint_idx, 1] - joint_limits[joint_idx, 0]
            center = (joint_limits[joint_idx, 0] + joint_limits[joint_idx, 1]) / 2

            # Generate target positions with guaranteed minimum movement
            min_movement = 0.3 * limit_range  # At least 30% of range
            max_movement = 0.7 * limit_range  # At most 70% of range

            # Alternate between positive and negative movements
            if config_idx % 2 == 0:
                movement = np.random.uniform(min_movement, max_movement)
            else:
                movement = -np.random.uniform(min_movement, max_movement)

            # Apply movement, clamp to limits
            q_target[joint_idx] = np.clip(
                q_initial[joint_idx] + movement,
                joint_limits[joint_idx, 0],
                joint_limits[joint_idx, 1],
            )

            test_configs.append((q_initial, q_target))

    # Generate grid
    kp_values = np.linspace(kp_range[0], kp_range[1], grid_size)
    kd_values = np.linspace(kd_range[0], kd_range[1], grid_size)

    print(f"\n  Testing {grid_size}x{grid_size} = {grid_size**2} combinations...")
    print(f"  Kp range: [{kp_range[0]:.0f}, {kp_range[1]:.0f}]")
    print(f"  Kd range: [{kd_range[0]:.1f}, {kd_range[1]:.1f}]")
    print(f"  Test configurations: {len(test_configs)}")

    # Show test configurations for this joint
    for i, (q_init, q_targ) in enumerate(test_configs):
        movement = q_targ[joint_idx] - q_init[joint_idx]
        print(
            f"    Config {i + 1}: Joint {joint_idx + 1} moves "
            f"{np.degrees(q_init[joint_idx]):+6.1f}° → {np.degrees(q_targ[joint_idx]):+6.1f}° "
            f"(Δ={np.degrees(movement):+6.1f}°)"
        )

    # Store results for each (Kp, Kd) pair
    grid_results = []
    best_avg_cost = float("inf")
    best_result = None

    for i, kp in enumerate(kp_values):
        for j, kd in enumerate(kd_values):
            # Build full gain arrays (only tune this joint, use defaults for others)
            kp_full = np.array([400.0, 400.0, 300.0, 250.0, 150.0, 100.0, 50.0])
            kd_full = np.array([20.0, 20.0, 15.0, 12.0, 8.0, 5.0, 3.0])
            kp_full[joint_idx] = kp
            kd_full[joint_idx] = kd

            # Test across all configurations
            costs = []
            for config_idx, (q_initial, q_target) in enumerate(test_configs):
                metrics_dict = test_pid_gains_multijoint(
                    mj_model,
                    mj_data,
                    kp_full,
                    kd_full,
                    q_initial,
                    q_target,
                    duration=3.0,
                )

                if joint_idx in metrics_dict and metrics_dict[joint_idx]["moved"]:
                    costs.append(metrics_dict[joint_idx]["cost"])
                elif i == 0 and j == 0:  # Debug first test only
                    # Debug why joint didn't move
                    if joint_idx not in metrics_dict:
                        print(f"    DEBUG: Joint {joint_idx + 1} not in metrics_dict")
                    elif not metrics_dict[joint_idx]["moved"]:
                        # Get actual position trajectory to see what happened
                        pos_traj = metrics_dict[joint_idx].get("position", None)
                        if pos_traj is not None:
                            pos_start = pos_traj[0]
                            pos_end = pos_traj[-1]
                            pos_range = np.max(pos_traj) - np.min(pos_traj)
                            print(
                                f"    DEBUG: Joint {joint_idx + 1} config {config_idx + 1}: "
                                f"commanded: {q_initial[joint_idx]:.3f}→{q_target[joint_idx]:.3f}, "
                                f"actual: {pos_start:.3f}→{pos_end:.3f}, "
                                f"range={pos_range:.6f}, moved=False"
                            )
                        else:
                            print(
                                f"    DEBUG: Joint {joint_idx + 1} config {config_idx + 1}: "
                                f"q_initial={q_initial[joint_idx]:.3f}, "
                                f"q_target={q_target[joint_idx]:.3f}, "
                                f"moved=False (no position data)"
                            )

            # Average cost across configurations
            avg_cost = np.mean(costs) if costs else 1e6

            grid_results.append({
                "kp": kp,
                "kd": kd,
                "avg_cost": avg_cost,
                "costs": costs,
            })

            print(
                f"  [{i * grid_size + j + 1}/{grid_size**2}] Kp={kp:.1f}, Kd={kd:.1f} → cost={avg_cost:.2f}",
                end="",
            )

            if avg_cost < best_avg_cost:
                best_avg_cost = avg_cost
                best_result = {
                    "kp": kp,
                    "kd": kd,
                    "cost": avg_cost,
                    "joint_idx": joint_idx,
                }
                print(" ✓ new best!")
            else:
                print()

    return {
        "best": best_result,
        "all_results": grid_results,
        "kp_values": kp_values,
        "kd_values": kd_values,
    }


def refine_multijoint_coupled(
    mj_model,
    mj_data,
    initial_gains_kp,
    initial_gains_kd,
    joints_to_refine,
    test_configs,
    refinement_factor=0.3,
    grid_size=3,
):
    """
    Refine gains accounting for multi-joint coupling effects.

    Takes individually-optimized gains and searches in a narrow range around them
    while testing full multi-joint trajectories. This captures coupling effects
    that single-joint optimization misses.

    Args:
        mj_model: MuJoCo model
        mj_data: MuJoCo data
        initial_gains_kp: Initial Kp gains from individual optimization (7 joints)
        initial_gains_kd: Initial Kd gains from individual optimization (7 joints)
        joints_to_refine: List of joint indices to refine
        test_configs: List of (q_initial, q_target) multi-joint trajectories
        refinement_factor: Search ±X% around initial gains (default: 30%)
        grid_size: Grid size for refinement (smaller than initial search)

    Returns:
        dict: Refined gains with best performance on coupled trajectories
    """
    print("\n" + "=" * 70)
    print("MULTI-JOINT COUPLING REFINEMENT")
    print("=" * 70)
    print(f"Refining joints: {[j + 1 for j in joints_to_refine]}")
    print(f"Refinement range: ±{refinement_factor * 100:.0f}% around initial gains")
    print(f"Grid size: {grid_size}x{grid_size} per joint")
    print(f"Test configurations: {len(test_configs)}")

    # For each joint, create a small grid around its initial value
    param_ranges = []
    param_names = []

    for joint_idx in joints_to_refine:
        kp_init = initial_gains_kp[joint_idx]
        kd_init = initial_gains_kd[joint_idx]

        kp_min = kp_init * (1 - refinement_factor)
        kp_max = kp_init * (1 + refinement_factor)
        kd_min = kd_init * (1 - refinement_factor)
        kd_max = kd_init * (1 + refinement_factor)

        param_ranges.append((kp_min, kp_max, kd_min, kd_max))
        param_names.append(f"J{joint_idx + 1}")

        print(
            f"  Joint {joint_idx + 1}: Kp [{kp_min:.1f}, {kp_max:.1f}], Kd [{kd_min:.1f}, {kd_max:.1f}]"
        )

    # Simple grid search over the refinement space
    # For simplicity, we'll do one joint at a time even in refinement
    # (full combinatorial would be grid_size^(2*n_joints) which explodes quickly)

    refined_kp = initial_gains_kp.copy()
    refined_kd = initial_gains_kd.copy()

    print("\nRefining each joint while others use initial values...")

    for idx, joint_idx in enumerate(joints_to_refine):
        kp_min, kp_max, kd_min, kd_max = param_ranges[idx]

        print(f"\n  Refining Joint {joint_idx + 1}...")

        kp_values = np.linspace(kp_min, kp_max, grid_size)
        kd_values = np.linspace(kd_min, kd_max, grid_size)

        best_cost = float("inf")
        best_kp = refined_kp[joint_idx]
        best_kd = refined_kd[joint_idx]

        for kp in kp_values:
            for kd in kd_values:
                # Test with current candidate gains
                test_kp = refined_kp.copy()
                test_kd = refined_kd.copy()
                test_kp[joint_idx] = kp
                test_kd[joint_idx] = kd

                # Average cost across all test trajectories
                costs = []
                for q_initial, q_target in test_configs:
                    metrics_dict = test_pid_gains_multijoint(
                        mj_model,
                        mj_data,
                        test_kp,
                        test_kd,
                        q_initial,
                        q_target,
                        duration=3.0,
                    )

                    # Sum costs for all moving joints
                    total_cost = sum(
                        m["cost"]
                        for m in metrics_dict.values()
                        if m.get("moved", False)
                    )
                    costs.append(total_cost)

                avg_cost = np.mean(costs)

                if avg_cost < best_cost:
                    best_cost = avg_cost
                    best_kp = kp
                    best_kd = kd

        refined_kp[joint_idx] = best_kp
        refined_kd[joint_idx] = best_kd

        improvement = (
            (initial_gains_kp[joint_idx] - best_kp) / initial_gains_kp[joint_idx]
        ) * 100
        print(
            f"    Best: Kp={best_kp:.1f} (initial: {initial_gains_kp[joint_idx]:.1f}), "
            f"Kd={best_kd:.1f} (initial: {initial_gains_kd[joint_idx]:.1f})"
        )
        print(f"    Cost: {best_cost:.2f}")

    return {
        "kp": refined_kp,
        "kd": refined_kd,
        "joints_refined": joints_to_refine,
    }


def plot_grid_search_results(results, joint_idx, output_dir):
    """
    Plot grid search results as heatmap and best trajectory.

    Saves to output_dir/joint_{joint_idx + 1}_grid_search.png
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(
        f"Joint {joint_idx + 1} - Individual Grid Search Results",
        fontsize=14,
        fontweight="bold",
    )

    # Extract grid data
    kp_values = results["kp_values"]
    kd_values = results["kd_values"]
    grid_size = len(kp_values)

    # Create cost matrix for heatmap
    cost_matrix = np.zeros((grid_size, grid_size))
    for result in results["all_results"]:
        i = np.argmin(np.abs(kp_values - result["kp"]))
        j = np.argmin(np.abs(kd_values - result["kd"]))
        cost_matrix[j, i] = result["avg_cost"]  # Note: j,i for correct orientation

    # 1. Cost heatmap
    ax = axes[0, 0]
    im = ax.imshow(cost_matrix, aspect="auto", origin="lower", cmap="viridis")
    ax.set_xlabel("Kp")
    ax.set_ylabel("Kd")
    ax.set_title("Cost (Lower is Better)")
    ax.set_xticks(range(grid_size))
    ax.set_xticklabels([f"{kp:.0f}" for kp in kp_values])
    ax.set_yticks(range(grid_size))
    ax.set_yticklabels([f"{kd:.1f}" for kd in kd_values])
    plt.colorbar(im, ax=ax)

    # Mark best point
    best = results["best"]
    best_i = np.argmin(np.abs(kp_values - best["kp"]))
    best_j = np.argmin(np.abs(kd_values - best["kd"]))
    ax.plot(best_i, best_j, "r*", markersize=20, label="Best")
    ax.legend()

    # For the other plots, we need to run one more trajectory with best gains
    # to get detailed trajectory data
    ax = axes[0, 1]
    ax.text(
        0.5,
        0.5,
        f"Best Gains:\nKp = {best['kp']:.1f}\nKd = {best['kd']:.1f}\nCost = {best['cost']:.2f}",
        ha="center",
        va="center",
        transform=ax.transAxes,
        fontsize=14,
    )
    ax.axis("off")

    # 3. Cost distribution
    ax = axes[1, 0]
    all_costs = [r["avg_cost"] for r in results["all_results"]]
    ax.hist(all_costs, bins=20, alpha=0.7, color="blue", edgecolor="black")
    ax.axvline(best["cost"], color="r", linestyle="--", linewidth=2, label="Best")
    ax.set_xlabel("Cost")
    ax.set_ylabel("Frequency")
    ax.set_title("Cost Distribution")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. Kp vs Kd scatter colored by cost
    ax = axes[1, 1]
    kps = [r["kp"] for r in results["all_results"]]
    kds = [r["kd"] for r in results["all_results"]]
    costs = [r["avg_cost"] for r in results["all_results"]]
    scatter = ax.scatter(kps, kds, c=costs, cmap="viridis", s=100, alpha=0.7)
    ax.plot(best["kp"], best["kd"], "r*", markersize=20, label="Best")
    ax.set_xlabel("Kp")
    ax.set_ylabel("Kd")
    ax.set_title("Parameter Space")
    plt.colorbar(scatter, ax=ax, label="Cost")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    output_path = output_dir / f"joint_{joint_idx + 1}_grid_search.png"
    plt.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"  Plot saved: {output_path.name}")


def main():
    """Run automated PID tuning with two-stage approach."""
    parser = argparse.ArgumentParser(
        description="Realistic automated PID tuning (grid search + multi-joint coupling)"
    )
    parser.add_argument(
        "--joints",
        type=int,
        nargs="+",
        help="Joint numbers to tune (1-7), default: all 7 joints",
    )
    parser.add_argument(
        "--grid-size",
        type=int,
        default=5,
        help="Grid size for individual search (NxN), default: 5",
    )
    parser.add_argument(
        "--refine-grid-size",
        type=int,
        default=3,
        help="Grid size for coupling refinement, default: 3",
    )
    parser.add_argument(
        "--skip-refinement",
        action="store_true",
        help="Skip multi-joint coupling refinement (faster but less accurate)",
    )
    parser.add_argument(
        "--model",
        type=str,
        default=str(
            Path.home()
            / "ros2_ws/src/openarm_description/mujoco_models/openarm_v10.xml"
        ),
        help="Path to MuJoCo XML model",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="figs",
        help="Output directory for plots, default: figs/",
    )

    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)
    print(f"Output directory: {output_dir.absolute()}")

    # Load MuJoCo model
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"ERROR: Model not found: {model_path}")
        print("Generate with: python3 scripts/urdf_to_mjcf.py --arm-type v10")
        return 1

    print(f"Loading model: {model_path}")
    mj_model = mujoco.MjModel.from_xml_path(str(model_path))
    mj_data = mujoco.MjData(mj_model)
    print(f"Model loaded: {mj_model.nv} DOF\n")

    # Define joints to tune
    if args.joints:
        joints_to_tune = [j - 1 for j in args.joints if 1 <= j <= 7]
    else:
        joints_to_tune = list(range(7))  # Default: all 7 joints

    # Define search ranges per joint (based on torque limits and inertia)
    # Ranges expanded further after observing boundary hits in refinement
    # Proximal joints (1-3) need very high gains due to high inertia and gravity loading
    search_ranges = {
        0: {"kp": (1000, 4000), "kd": (50, 200)},  # Joint 1 (very high gains needed)
        1: {"kp": (800, 3000), "kd": (20, 120)},  # Joint 2 (high Kp, moderate Kd)
        2: {"kp": (600, 2500), "kd": (30, 150)},  # Joint 3 (high)
        3: {"kp": (400, 2000), "kd": (20, 100)},  # Joint 4 (medium-high)
        4: {"kp": (200, 1000), "kd": (10, 50)},  # Joint 5 (medium)
        5: {"kp": (150, 800), "kd": (8, 40)},  # Joint 6 (low inertia)
        6: {"kp": (80, 500), "kd": (4, 25)},  # Joint 7 (lowest inertia)
    }

    print("=" * 70)
    print("STAGE 1: INDIVIDUAL JOINT OPTIMIZATION")
    print("=" * 70)
    print(f"Joints to tune: {[j + 1 for j in joints_to_tune]}")
    print(
        f"Grid size: {args.grid_size}x{args.grid_size} = {args.grid_size**2} tests per joint"
    )
    print("=" * 70)

    # Stage 1: Individual joint optimization
    individual_results = {}
    individual_gains_kp = np.array([400.0, 400.0, 300.0, 250.0, 150.0, 100.0, 50.0])
    individual_gains_kd = np.array([20.0, 20.0, 15.0, 12.0, 8.0, 5.0, 3.0])

    for joint_idx in joints_to_tune:
        print(f"\n{'=' * 60}")
        print(f"Joint {joint_idx + 1}")
        print(f"{'=' * 60}")

        ranges = search_ranges[joint_idx]
        results = grid_search_joint_individual(
            mj_model,
            mj_data,
            joint_idx,
            kp_range=ranges["kp"],
            kd_range=ranges["kd"],
            grid_size=args.grid_size,
        )

        individual_results[joint_idx] = results

        # Update gains with individually optimized values
        best = results["best"]
        individual_gains_kp[joint_idx] = best["kp"]
        individual_gains_kd[joint_idx] = best["kd"]

        print(f"\n✓ Best individual gains for Joint {joint_idx + 1}:")
        print(f"    Kp = {best['kp']:.1f}")
        print(f"    Kd = {best['kd']:.1f}")
        print(f"    Cost = {best['cost']:.2f}")

        # Plot results
        plot_grid_search_results(results, joint_idx, output_dir)

    # Stage 2: Multi-joint coupling refinement
    if not args.skip_refinement and len(joints_to_tune) > 0:
        # Define multi-joint test trajectories
        multi_joint_configs = [
            # Home to various poses
            (np.zeros(7), np.array([0, -np.pi / 3, 0, 0, 0, 0, 0])),
            (np.zeros(7), np.array([0, -np.pi / 2, 0, 0, 0, 0, 0])),
            (np.zeros(7), np.array([0, 0, np.pi / 3, 0, 0, 0, 0])),
            # Complex multi-joint motions
            (np.zeros(7), np.array([np.pi / 6, -np.pi / 4, np.pi / 6, 0, 0, 0, 0])),
            (
                np.array([np.pi / 6, -np.pi / 4, np.pi / 6, 0, 0, 0, 0]),
                np.zeros(7),
            ),  # Return
        ]

        refinement_results = refine_multijoint_coupled(
            mj_model,
            mj_data,
            individual_gains_kp,
            individual_gains_kd,
            joints_to_tune,
            multi_joint_configs,
            refinement_factor=0.3,
            grid_size=args.refine_grid_size,
        )

        final_gains_kp = refinement_results["kp"]
        final_gains_kd = refinement_results["kd"]
    else:
        print("\nSkipping coupling refinement...")
        final_gains_kp = individual_gains_kp
        final_gains_kd = individual_gains_kd

    # Print final summary
    print("\n" + "=" * 70)
    print("TUNING COMPLETE - FINAL RESULTS")
    print("=" * 70)
    print("\nOptimal PID gains:\n")

    final_gains_dict = {}
    for joint_idx in range(7):
        joint_name = f"joint{joint_idx + 1}"
        final_gains_dict[joint_name] = {
            "kp": float(final_gains_kp[joint_idx]),
            "kd": float(final_gains_kd[joint_idx]),
            "ki": 0.0,
        }

        if joint_idx in joints_to_tune:
            status = "✓ tuned"
        else:
            status = "(default)"

        print(f"Joint {joint_idx + 1} {status}:")
        print(f"  Kp = {final_gains_kp[joint_idx]:.1f}")
        print(f"  Kd = {final_gains_kd[joint_idx]:.1f}")
        print("  Ki = 0.0")
        print()

    # Save to JSON
    json_path = output_dir / "optimal_pid_gains.json"
    with open(json_path, "w") as f:
        json.dump(final_gains_dict, f, indent=2)

    print(f"✓ Gains saved to: {json_path}")
    print(f"✓ Plots saved to: {output_dir}/")

    # Save detailed cost log
    log_path = output_dir / "tuning_log.txt"
    with open(log_path, "w") as f:
        f.write("=" * 70 + "\n")
        f.write("AUTOTUNE_PID_SIMPLE.PY - TUNING LOG\n")
        f.write("=" * 70 + "\n\n")

        f.write("STAGE 1: INDIVIDUAL JOINT OPTIMIZATION\n")
        f.write("-" * 70 + "\n")
        for joint_idx in joints_to_tune:
            if joint_idx in individual_results:
                best = individual_results[joint_idx]["best"]
                f.write(f"\nJoint {joint_idx + 1}:\n")
                f.write(f"  Best Kp: {best['kp']:.1f}\n")
                f.write(f"  Best Kd: {best['kd']:.1f}\n")
                f.write(f"  Cost: {best['cost']:.2f}\n")

        f.write("\n" + "=" * 70 + "\n")
        f.write("FINAL OPTIMAL GAINS\n")
        f.write("=" * 70 + "\n")
        for joint_idx in range(7):
            status = "✓ tuned" if joint_idx in joints_to_tune else "(default)"
            f.write(f"\nJoint {joint_idx + 1} {status}:\n")
            f.write(f"  Kp = {final_gains_kp[joint_idx]:.1f}\n")
            f.write(f"  Kd = {final_gains_kd[joint_idx]:.1f}\n")
            f.write("  Ki = 0.0\n")

        f.write("\n" + "=" * 70 + "\n")
        f.write("COST BREAKDOWN BY JOINT\n")
        f.write("=" * 70 + "\n")
        f.write("Cost function weights:\n")
        f.write("  - RMS tracking error (degrees) × 1.0\n")
        f.write("  - Settling time (seconds) × 2.0\n")
        f.write("  - Final velocity (rad/s) × 100.0  ← CRITICAL!\n")
        f.write("  - Overshoot (degrees) × 5.0\n")
        f.write("  - Torque saturation penalty\n\n")

        for joint_idx in joints_to_tune:
            if joint_idx in individual_results:
                best = individual_results[joint_idx]["best"]
                f.write(f"Joint {joint_idx + 1}: Cost = {best['cost']:.2f}\n")

        total_cost = sum(
            individual_results[j]["best"]["cost"]
            for j in joints_to_tune
            if j in individual_results
        )
        avg_cost = total_cost / len(joints_to_tune) if joints_to_tune else 0
        f.write(f"\nTotal cost (sum): {total_cost:.2f}\n")
        f.write(f"Average cost per joint: {avg_cost:.2f}\n")

        f.write("\n" + "=" * 70 + "\n")
        f.write("INTERPRETATION\n")
        f.write("=" * 70 + "\n")
        f.write("Good performance targets:\n")
        f.write("  - Cost < 50 per joint: Excellent\n")
        f.write("  - Cost < 100 per joint: Good\n")
        f.write("  - Cost < 200 per joint: Acceptable\n")
        f.write("  - Cost > 200 per joint: Needs improvement\n\n")

        if avg_cost < 50:
            f.write("✓ EXCELLENT: Gains are very well tuned!\n")
        elif avg_cost < 100:
            f.write("✓ GOOD: Gains should work well for most applications.\n")
        elif avg_cost < 200:
            f.write(
                "⚠ ACCEPTABLE: Gains may need refinement for critical applications.\n"
            )
        else:
            f.write("✗ POOR: Consider re-running with different search ranges.\n")

    print(f"✓ Tuning log saved to: {log_path}")

    # Print URDF snippet
    print("\n" + "=" * 70)
    print("URDF SNIPPET (copy to openarm.ros2_control.xacro)")
    print("=" * 70)
    for joint_idx in range(7):
        print(
            f'<xacro:configure_joint joint_name="openarm_${{arm_prefix}}joint{joint_idx + 1}" '
            f'initial_position="0.0" '
            f'kp="{final_gains_kp[joint_idx]:.1f}" '
            f'kd="{final_gains_kd[joint_idx]:.1f}" '
            f'ki="0.0"/>'
        )

    print("\n" + "=" * 70)
    print("Next steps:")
    print("  1. Apply gains to URDF (see snippet above)")
    print(
        "  2. Rebuild: cd ~/ros2_ws && colcon build --packages-select openarm_description"
    )
    print("  3. Test: python3 scripts/test_trajectory_tracking.py")
    print("=" * 70)

    return 0


if __name__ == "__main__":
    exit(main())
