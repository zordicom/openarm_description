# Gravity Compensation in MuJoCo

## Quick Start

1. **Launch MuJoCo simulation**:

```bash
ros2 launch openarm_description mujoco_sim.launch.py
```

2. **Start gravity compensation controller** (in another terminal):

```bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/gravity_compensation_controller.py
```

3. **Switch to effort control** (in a third terminal):

```bash
ros2 control switch_controllers --activate effort_controller --deactivate joint_trajectory_controller
```

The robot should now hold its position under gravity.

**Note**: By default, the controller computes gravity using **Pinocchio** (standard approach for real hardware). For perfect simulation testing, you can use MuJoCo's `qfrc_bias` directly with `-p use_mujoco_qfrc_bias:=true`.

---

## How It Works

The controller uses **Pinocchio** to compute gravity torques:

```python
tau_gravity = pinocchio.computeGeneralizedGravity(model, data, q)
tau_damping = -damping_gains * dq
tau_total = tau_gravity + tau_damping
```

This is the **standard robotics approach** - the same code you'd use on real hardware.

### Optional: MuJoCo qfrc_bias Mode

For sanity checking in simulation, you can use MuJoCo's internal `qfrc_bias` directly:

```bash
python3 scripts/gravity_compensation_controller.py --ros-args -p use_mujoco_qfrc_bias:=true
```

This bypasses Pinocchio and uses MuJoCo's own gravity computation, giving perfect compensation (useful for isolating model vs. control issues).

**Note**: MuJoCo uses a non-standard sign convention. When applying `qfrc_bias`, use **POSITIVE sign**: `cmd = +qfrc_bias` (not negative). See code comments for details.

---

## Important Requirements

### 1. Collisions Must Be Disabled

The MJCF model **must** have all collisions disabled (`contype=0 conaffinity=0`).

**Why?** Self-collision generates constraint forces that completely override the commanded torques.

The conversion script (`scripts/urdf_to_mjcf.py`) automatically disables collisions. If regenerating models, collisions are disabled by default.

### 2. Control Rate

All components run at **1000 Hz** to match MuJoCo's 1ms timestep:

- MuJoCo simulation: 1000 Hz
- ros2_control: 1000 Hz
- Python controller: 1000 Hz

---

## Key Parameters

- `use_mujoco_qfrc_bias`: Use MuJoCo's qfrc_bias directly (default: False)
  - `true`: Perfect compensation for testing/debugging
  - `false`: Use Pinocchio (default, real hardware approach)
- `add_damping`: Enable damping (default: False)
  - Disabled by default - pure gravity compensation works without damping
  - Enable only if you have friction/disturbances in your system
- `enable_plot`: Show live plots (default: True, set to `false` for better performance)
- `control_rate`: Control loop frequency in Hz (default: 1000)

## Using Pinocchio Mode (Default)

The controller computes gravity using Pinocchio:

```bash
python3 scripts/gravity_compensation_controller.py
```

This is the **standard robotics approach** - same code path as real hardware. The models now match (mass, COM, inertia) after ensuring `hand:=false` in xacro generation.

---

## Troubleshooting

### Robot falls when using MuJoCo qfrc_bias mode

Check that the MJCF model has collisions disabled:

```bash
grep "contype=" ~/ros2_ws/install/openarm_description/share/openarm_description/mujoco_models/openarm_v10.xml
```

Should show `contype="0"` everywhere. If not, regenerate:

```bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/urdf_to_mjcf.py --arm-type v10 -o mujoco_models/openarm_v10.xml
cp mujoco_models/openarm_v10.xml ~/ros2_ws/install/openarm_description/share/openarm_description/mujoco_models/
```

### Verify gravity compensation works

Run the verification script:

```bash
python3 scripts/verify_gravity_comp.py
```

Should show ✅ ALL TESTS PASSED with zero drift.

---

## Summary

**For MuJoCo simulation:**

- Use `+qfrc_bias` (POSITIVE sign - non-standard MuJoCo convention)
- Collisions must be disabled (`contype=0` in MJCF)
- Perfect gravity compensation with zero drift

**For real hardware:**

- Use Pinocchio to compute gravity with standard convention
- Apply with standard NEGATIVE sign: `τ_cmd = -τ_gravity`

**The key difference**: MuJoCo subtracts `qfrc_bias` internally (`M·q̈ = ... - qfrc_bias + qfrc_applied`), while standard frameworks add it (`M·q̈ = τ + τ_gravity`), requiring opposite signs for compensation.
