#!/usr/bin/env python3
"""
Verify gravity compensation works with collisions disabled.

This script directly tests MuJoCo's qfrc_bias without ROS2:
1. Load the MJCF model
2. Set robot to a configuration with gravity torque
3. Apply qfrc_applied = +qfrc_bias
4. Verify qacc = 0 and no drift over time
"""

from pathlib import Path

import numpy as np

import mujoco


def main():
    """Test gravity compensation at various configurations."""
    mjcf_path = Path(__file__).parent.parent / "mujoco_models" / "openarm_v10.xml"

    if not mjcf_path.exists():
        print(f"❌ MJCF file not found: {mjcf_path}")
        print(
            "Run: python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml"
        )
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
        print(
            "   Run: python3 scripts/urdf_to_mjcf.py --arm-type v10 -o mujoco_models/openarm_v10.xml"
        )
        return False

    # Test configurations
    test_configs = [
        ("Zero (home)", np.array([0, 0, 0, 0, 0, 0, 0])),
        ("J2=-60° (significant gravity)", np.array([0, -np.pi / 3, 0, 0, 0, 0, 0])),
        ("J2=-90° (horizontal)", np.array([0, -np.pi / 2, 0, 0, 0, 0, 0])),
    ]

    all_passed = True

    for config_name, q_test in test_configs:
        print("\n" + "-" * 80)
        print(f"Testing: {config_name}")
        print("-" * 80)

        # Reset to test configuration
        j2_joint_id = mujoco.mj_name2id(
            mj_model, mujoco.mjtObj.mjOBJ_JOINT, "openarm_joint2"
        )
        j2_dof = mj_model.jnt_dofadr[j2_joint_id]

        for i in range(7):
            joint_id = mujoco.mj_name2id(
                mj_model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_joint{i + 1}"
            )
            dof_adr = mj_model.jnt_dofadr[joint_id]
            mj_data.qpos[dof_adr] = q_test[i]
            mj_data.qvel[dof_adr] = 0.0

        # Single step test
        mujoco.mj_forward(mj_model, mj_data)
        qfrc_bias_initial = mj_data.qfrc_bias.copy()

        # Apply +qfrc_bias
        for i in range(7):
            joint_id = mujoco.mj_name2id(
                mj_model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_joint{i + 1}"
            )
            dof_adr = mj_model.jnt_dofadr[joint_id]
            mj_data.qfrc_applied[dof_adr] = +qfrc_bias_initial[dof_adr]

        mujoco.mj_step(mj_model, mj_data)

        print(f"  J2 qfrc_bias:  {qfrc_bias_initial[j2_dof]:8.4f} Nm")
        print(f"  J2 acceleration: {mj_data.qacc[j2_dof]:8.4f} rad/s²")

        if abs(mj_data.qacc[j2_dof]) < 0.1:
            print("  ✓ Single step: qacc ≈ 0")
        else:
            print(
                f"  ❌ Single step: qacc = {mj_data.qacc[j2_dof]:.4f} rad/s² (expected ~0)"
            )
            all_passed = False
            continue

        # Multi-step stability test (1 second)
        print(
            "\n  Running 1000 steps (1 second) with continuous gravity compensation..."
        )

        # Reset
        for i in range(7):
            joint_id = mujoco.mj_name2id(
                mj_model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_joint{i + 1}"
            )
            dof_adr = mj_model.jnt_dofadr[joint_id]
            mj_data.qpos[dof_adr] = q_test[i]
            mj_data.qvel[dof_adr] = 0.0

        initial_pos = mj_data.qpos[j2_dof].copy()

        for step in range(1000):
            # Apply qfrc_bias continuously
            for i in range(7):
                joint_id = mujoco.mj_name2id(
                    mj_model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_joint{i + 1}"
                )
                dof_adr = mj_model.jnt_dofadr[joint_id]
                mj_data.qfrc_applied[dof_adr] = +mj_data.qfrc_bias[dof_adr]

            mujoco.mj_step(mj_model, mj_data)

        final_pos = mj_data.qpos[j2_dof]
        drift_deg = np.degrees(final_pos - initial_pos)

        print(f"  Initial J2: {np.degrees(initial_pos):7.2f}°")
        print(f"  Final J2:   {np.degrees(final_pos):7.2f}°")
        print(f"  Drift:      {drift_deg:7.2f}° in 1 second")

        if abs(drift_deg) < 0.5:
            print("  ✓ Stable: drift < 0.5°")
        else:
            print(f"  ❌ Unstable: drift = {drift_deg:.2f}° (expected ~0)")
            all_passed = False

    # Summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)

    if all_passed:
        print("✅ ALL TESTS PASSED!")
        print("\nGravity compensation works perfectly with qfrc_applied = +qfrc_bias")
        print("The MJCF model has collisions properly disabled.")
        print("\nYou can now use the ROS2 workflow:")
        print("  1. ros2 launch openarm_description mujoco_sim.launch.py")
        print(
            "  2. python3 scripts/gravity_compensation_controller.py --ros-args -p use_mujoco_qfrc_bias:=true"
        )
        print(
            "  3. ros2 control switch_controllers --activate effort_controller --deactivate joint_trajectory_controller"
        )
        return True
    else:
        print("❌ TESTS FAILED!")
        print("\nPossible issues:")
        print("  - Collisions not disabled in MJCF")
        print("  - MJCF file needs regeneration")
        print("  - Model configuration issue")
        print(
            "\nFix: python3 scripts/urdf_to_mjcf.py --arm-type v10 -o mujoco_models/openarm_v10.xml"
        )
        return False


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
