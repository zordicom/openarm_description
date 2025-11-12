#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Verify gravity compensation works with collisions disabled.

This script directly tests MuJoCo's qfrc_bias without ROS2:
1. Load the MJCF model
2. Set robot to random configurations within joint limits
3. Apply initial velocity perturbations
4. Apply qfrc_applied = +qfrc_bias
5. Verify qacc ≈ 0 and minimal drift over time
"""

import sys
from itertools import starmap
from pathlib import Path

import numpy as np

import mujoco

# Test thresholds
MAX_ACCELERATION_THRESHOLD = 1.0  # rad/s²
MAX_DRIFT_THRESHOLD = 2.0  # degrees per second


def main():
    """Test gravity compensation at various random configurations."""
    mjcf_path = Path(__file__).parent.parent / "mujoco_models" / "openarm_v10.xml"

    if not mjcf_path.exists():
        print(f"❌ MJCF file not found: {mjcf_path}")
        print("Run: python3 scripts/urdf_to_mjcf.py --arm-type v10")
        print("     --output mujoco_models/openarm_v10.xml")
        return False

    print("=" * 80)
    print("GRAVITY COMPENSATION VERIFICATION")
    print("=" * 80)

    # Load model
    mj_model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    mj_data = mujoco.MjData(mj_model)

    # Check for contacts at zero config
    mujoco.mj_forward(mj_model, mj_data)
    print(f"\nModel loaded: {mj_model.nv} DOF, {mj_model.njnt} joints")
    print(f"Contacts at zero config: {mj_data.ncon}")

    if mj_data.ncon > 0:
        print("⚠️  WARNING: Collisions are NOT disabled!")
        print("   Run: python3 scripts/urdf_to_mjcf.py --arm-type v10")
        print("        -o mujoco_models/openarm_v10.xml")
        return False

    # Get joint limits and DOF addresses
    joint_names = [f"openarm_joint{i + 1}" for i in range(7)]
    joint_limits = []
    dof_addresses = []

    for joint_name in joint_names:
        joint_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        dof_adr = mj_model.jnt_dofadr[joint_id]
        dof_addresses.append(dof_adr)

        # Get joint limits
        jnt_limited = mj_model.jnt_limited[joint_id]
        if jnt_limited:
            jnt_range = mj_model.jnt_range[joint_id]
            joint_limits.append((jnt_range[0], jnt_range[1]))
        else:
            # Default range if no limits specified
            joint_limits.append((-np.pi, np.pi))

    print("\nJoint Limits (radians):")
    for i, (lower, upper) in enumerate(joint_limits):
        print(f"  Joint {i + 1}: [{lower:6.3f}, {upper:6.3f}]")

    # Generate 30 random configurations
    rng = np.random.default_rng(seed=42)
    num_configs = 30

    print(f"\nGenerating {num_configs} random test configurations...")
    test_configs = []

    for config_idx in range(num_configs):
        # Sample random position within joint limits
        q_test = np.array(list(starmap(rng.uniform, joint_limits)))

        # Sample random initial velocity perturbations (±0.5 rad/s)
        qvel_test = rng.uniform(-0.5, 0.5, size=7)

        test_configs.append((f"Config {config_idx + 1:2d}", q_test, qvel_test))

    all_passed = True
    failures = []
    max_drifts = []
    max_accs = []

    for config_name, q_test, qvel_test in test_configs:
        # Reset to test configuration with velocity perturbations
        for i in range(7):
            dof_adr = dof_addresses[i]
            mj_data.qpos[dof_adr] = q_test[i]
            mj_data.qvel[dof_adr] = qvel_test[i]

        # Single step test
        mujoco.mj_forward(mj_model, mj_data)
        qfrc_bias_initial = mj_data.qfrc_bias.copy()

        # Apply +qfrc_bias
        for i in range(7):
            dof_adr = dof_addresses[i]
            mj_data.qfrc_applied[dof_adr] = +qfrc_bias_initial[dof_adr]

        mujoco.mj_step(mj_model, mj_data)

        # Check max acceleration after one step
        accs = [mj_data.qacc[dof_adr] for dof_adr in dof_addresses]
        max_acc = np.max(np.abs(accs))
        max_accs.append(max_acc)

        # Multi-step stability test (1 second)
        # Reset with perturbations
        for i in range(7):
            dof_adr = dof_addresses[i]
            mj_data.qpos[dof_adr] = q_test[i]
            mj_data.qvel[dof_adr] = qvel_test[i]

        initial_positions = np.array([
            mj_data.qpos[dof_adr] for dof_adr in dof_addresses
        ])

        for _ in range(1000):
            # Apply qfrc_bias continuously
            for i in range(7):
                dof_adr = dof_addresses[i]
                mj_data.qfrc_applied[dof_adr] = +mj_data.qfrc_bias[dof_adr]

            mujoco.mj_step(mj_model, mj_data)

        final_positions = np.array([mj_data.qpos[dof_adr] for dof_adr in dof_addresses])
        drift = np.degrees(final_positions - initial_positions)
        max_drift = np.max(np.abs(drift))
        max_drifts.append(max_drift)

        # Determine pass/fail for this configuration
        acc_pass = max_acc < MAX_ACCELERATION_THRESHOLD
        drift_pass = max_drift < MAX_DRIFT_THRESHOLD

        if acc_pass and drift_pass:
            # Passed - print compact success
            status = f"max_acc={max_acc:5.2f} rad/s², max_drift={max_drift:5.2f}°"
            print(f"✓ {config_name}: {status}")
        else:
            # Failed - print detailed info
            print(f"\n❌ {config_name} FAILED:")
            print(f"   Joint positions (deg): {np.degrees(q_test)}")
            print(f"   Initial velocities (rad/s): {qvel_test}")
            acc_status = "[FAIL]" if not acc_pass else "[PASS]"
            drift_status = "[FAIL]" if not drift_pass else "[PASS]"
            print(f"   Max acceleration: {max_acc:.3f} rad/s² {acc_status}")
            print(f"   Max drift: {max_drift:.3f}° {drift_status}")
            failures.append(config_name)
            all_passed = False

    # Summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)

    print(f"\nTotal configurations tested: {num_configs}")
    print(f"Passed: {num_configs - len(failures)}")
    print(f"Failed: {len(failures)}")

    print("\nPerformance Statistics:")
    min_acc, max_acc_val, mean_acc = (
        np.min(max_accs),
        np.max(max_accs),
        np.mean(max_accs),
    )
    min_drift, max_drift_val, mean_drift = (
        np.min(max_drifts),
        np.max(max_drifts),
        np.mean(max_drifts),
    )
    print("  Max acceleration (rad/s²):")
    print(f"    min={min_acc:.3f}, max={max_acc_val:.3f}, mean={mean_acc:.3f}")
    print("  Max drift (degrees):")
    print(f"    min={min_drift:.3f}, max={max_drift_val:.3f}, mean={mean_drift:.3f}")

    if all_passed:
        print("\n✅ ALL TESTS PASSED!")
        print("\nGravity compensation works robustly across random")
        print("configurations with velocity perturbations.")
        print("The MJCF model has collisions properly disabled.")
        print("\nThresholds used:")
        print(f"  - Max acceleration: < {MAX_ACCELERATION_THRESHOLD} rad/s²")
        print(f"  - Max drift: < {MAX_DRIFT_THRESHOLD}°/second")
        print("\nYou can now use the ROS2 workflow:")
        print("  1. ros2 launch openarm_description mujoco_sim.launch.py")
        print("  2. python3 scripts/gravity_compensation_controller.py")
        print("     --ros-args -p use_mujoco_qfrc_bias:=true")
        print("  3. ros2 control switch_controllers")
        print("     --activate effort_controller")
        print("     --deactivate joint_trajectory_controller")
        return True

    print("\n❌ SOME TESTS FAILED!")
    print(f"\nFailed configurations: {', '.join(failures)}")
    print("\nPossible issues:")
    print("  - Collisions not disabled in MJCF")
    print("  - MJCF file needs regeneration")
    print("  - Model configuration issue")
    print("  - Velocity perturbations too large for current damping")
    print("\nFix: python3 scripts/urdf_to_mjcf.py --arm-type v10")
    print("     -o mujoco_models/openarm_v10.xml")
    return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
