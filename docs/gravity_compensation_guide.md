# Gravity Compensation for OpenARM - Complete Guide

**Copyright 2025 Zordi, Inc. All rights reserved.**

## Executive Summary

This document provides a complete understanding of proper gravity compensation in MuJoCo simulation and how it translates to real hardware control.

### Key Findings

✅ **Your current approach is CORRECT!**

- Computing `g(q)` using Pinocchio (not reading MuJoCo's `qfrc_bias`)
- Sending computed torques to effort controller
- Same code path as real hardware

⚠️ **Do NOT extract `qfrc_bias` from MuJoCo**

- Defeats the purpose of testing your dynamics model
- Creates circular dependency (reading sim state to control sim)
- Won't work on real hardware where you don't have "ground truth"

✅ **Enhanced Implementation**

- Added optional Coriolis compensation for moving scenarios
- Improved test suite with systematic configuration testing
- Better logging and validation

---

## Understanding MuJoCo Dynamics

### The Forward Dynamics Equation

MuJoCo computes robot motion using:

```
M(q)·q̈ = qfrc_applied + qfrc_bias + qfrc_constraint

Where:
  M(q)           = Mass/inertia matrix (configuration-dependent)
  qfrc_applied   = External forces WE COMMAND (via effort controller)
  qfrc_bias      = Gravity + Coriolis + centrifugal (MuJoCo computes)
  qfrc_constraint= Contact forces, joint limits
```

### What is `qfrc_bias`?

`qfrc_bias` is MuJoCo's internal computation of:

```
qfrc_bias(q, q̇) = g(q) + C(q,q̇)·q̇

Components:
  g(q)       = Gravity torques (dominant for static holding)
  C(q,q̇)·q̇  = Coriolis + centrifugal forces (significant when moving)
```

**Critical Point:** `qfrc_bias` opposes motion (left side of equation), so you need to compute compensation torques independently.

---

## Proper Gravity Compensation Strategy

### What to Compute

For **static holding** (zero velocity):

```python
# Only gravity compensation needed
tau_cmd = g(q) + tau_damping

Where:
  g(q) = pin.computeGeneralizedGravity(model, data, q)
  tau_damping = -Kd * q̇  # For stability
```

For **moving scenarios** (non-zero velocity):

```python
# Add Coriolis compensation
tau_cmd = g(q) + C(q,q̇)·q̇ + tau_damping

Where:
  C(q,q̇)·q̇ = pin.computeCoriolisMatrix(model, data, q, q̇) @ q̇
```

For **high acceleration** (rapid motion):

```python
# Full inverse dynamics
tau_cmd = pin.rnea(model, data, q, q̇, q̈_desired)
        = M(q)·q̈ + C(q,q̇)·q̇ + g(q)
```

### What NOT to Do

❌ **Do NOT read `qfrc_bias` and send it as torque:**

```python
# WRONG APPROACH - defeats the purpose!
tau_cmd = mj_data.qfrc_bias  # ❌ Don't do this!
```

**Why this is wrong:**

1. Only works in simulation (real robot doesn't have this)
2. Doesn't test your dynamics model accuracy
3. Creates circular dependency
4. Hides model calibration issues

---

## Implementation Details

### Current Hardware Interface

From `mujoco_ros2_control/src/mujoco_system.cpp` line 194-195:

```cpp
// Effort commands write directly to qfrc_applied
mj_data_->qfrc_applied[joint_state.mj_vel_adr] =
    clamp(joint_state.effort_command, min_eff, max_eff);
```

**What this means:**

- Your torque commands are the ONLY thing in `qfrc_applied`
- MuJoCo adds `qfrc_bias` automatically in forward dynamics
- You must compute compensation torques to cancel gravity

### Sign Convention

**Gravity compensation torques are POSITIVE** (same sign as `qfrc_bias`):

```
If gravity pulls down with force F:
  → MuJoCo computes qfrc_bias = +F (opposes acceleration)
  → You command qfrc_applied = +F (to cancel)
  → Result: M·q̈ = +F + (-F) + 0 = 0 → no motion ✓
```

Do NOT negate gravity torques!

---

## Testing Strategy

### Real-World Test Procedure

On real hardware, you would:

1. **Enable gravity compensation controller**
2. **Manually push/drag the arm** to a new position
3. **Release and observe:**
   - ✅ Arm stays at new position (good compensation)
   - ❌ Arm drifts down (insufficient compensation)
   - ❌ Arm springs back (too much compensation)

### Simulation Test Procedure

Since we can't physically push the robot in sim, we:

1. **Test static holding** at various configurations
2. **Monitor position drift** over time
3. **Verify stability** at challenging poses (extended arm, etc.)
4. **Check velocity damping** prevents oscillation

**What the test script does:**

```python
# Test 1: Current position (wherever robot starts)
test_configuration("Current Position", current_q, duration=10s)

# Test 2: Zero configuration (vertical, low gravity torque)
test_configuration("Zero Config", [0, 0, 0, 0, 0, 0, 0], duration=10s)

# Test 3: Extended arm (horizontal, high gravity torque)
test_configuration("Extended", [0, 1.0, 0, 1.5, 0, 0, 0], duration=15s)

# Test 4: Varied configuration (mixed joint angles)
test_configuration("Varied", [0.5, 0.5, 0.3, 1.0, 0.2, 0.1, 0], duration=10s)
```

**Acceptance criteria:**

- Position error < 0.05 rad (2.9°) for 10+ seconds
- RMS velocity < 0.01 rad/s (minimal oscillation)
- Stable at all tested configurations

### External Force Application (Now Available!)

**Test with programmatic force injection** via ROS2 service:

```python
# Apply external force to link (via MuJoCo's xfrc_applied)
apply_external_wrench(
    body_name="openarm_link7",
    force_xyz=[0, 0, -10],  # 10N downward
    torque_xyz=[0, 0, 0],
    duration=1.0,
    in_world_frame=True
)

# Check recovery
time.sleep(2.0)
assert position_error < threshold  # Should return to target
```

The `ApplyExternalWrench` service is now implemented in `mujoco_ros2_control` and used by `test_gravity_compensation.py`.

---

## Validation: Does Your Model Match Reality?

### How to Check Model Accuracy

**Method 1: Compare computed vs. expected gravity torques**

For a horizontal arm (joint2 = 90°), joint2 gravity torque should be approximately:

```
τ_gravity ≈ m * g * L_cm

Where:
  m = link mass (kg)
  g = 9.81 m/s²
  L_cm = distance from joint to center of mass (m)

Example for OpenARM link2:
  m ≈ 1.0 kg, L_cm ≈ 0.2 m
  τ ≈ 1.0 * 9.81 * 0.2 ≈ 2.0 N·m
```

Check if Pinocchio's computed value matches this rough estimate.

**Method 2: Tune gravity_scale parameter**

If robot drifts despite gravity compensation:

```yaml
# If arm sags (insufficient compensation)
gravity_scale: 1.1  # Increase torques 10%

# If arm rises (too much compensation)
gravity_scale: 0.9  # Decrease torques 10%
```

Ideal value should be 1.0 if URDF masses/inertias are accurate.

**Method 3: Log and compare (advanced)**

Add logging to compare Pinocchio vs. MuJoCo:

```python
# In gravity_compensation_controller.py
tau_gravity_pinocchio = pin.computeGeneralizedGravity(model, data, q)

# Read MuJoCo's qfrc_bias via custom message (would need plugin support)
tau_bias_mujoco = get_mujoco_qfrc_bias()  # Hypothetical

# Compare
error = tau_gravity_pinocchio - tau_bias_mujoco
log.info(f"Model error: {error}")  # Should be small!
```

---

## Usage Guide

### Running Gravity Compensation Tests

**Terminal 1: Start simulation (headless for automated testing)**

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch openarm_description mujoco_sim.launch.py use_rviz:=false headless:=true
```

**Terminal 2: Switch to effort controller**

```bash
ros2 control switch_controllers \
  --activate effort_controller \
  --deactivate joint_trajectory_controller
```

**Terminal 3: Run gravity compensation controller**

```bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/gravity_compensation_controller.py --ros-args -p enable_plot:=false
```

**Terminal 4: Run automated tests (includes external force perturbations)**

```bash
python3 scripts/test_gravity_compensation.py
```

The test suite now includes:

- Static position holding at multiple configurations
- **External force application** (10N downward on end-effector)
- Recovery validation after disturbance removal

### Expected Output

**Good gravity compensation:**

```
=== Stability Test Results ===
Target:   [ 0.000,  0.500,  0.000,  1.000,  0.000,  0.000,  0.000] rad
Mean:     [ 0.001,  0.498,  0.001,  0.999,  0.000,  0.001,  0.000] rad
Std Dev:  [0.0012, 0.0023, 0.0015, 0.0031, 0.0008, 0.0010, 0.0005] rad
Max Err:  [0.0045, 0.0089, 0.0053, 0.0124, 0.0032, 0.0041, 0.0021] rad
RMS Vel:  [0.0021, 0.0034, 0.0019, 0.0028, 0.0012, 0.0015, 0.0008] rad/s
✅ PASS: Position held within 0.050 rad
```

**Poor gravity compensation (drifting):**

```
=== Stability Test Results ===
Max Err:  [0.0234, 0.1523, 0.0412, 0.2156, 0.0156, 0.0234, 0.0089] rad
❌ FAIL: Joints exceeded threshold: ['openarm_joint2', 'openarm_joint4']
```

---

## Parameter Tuning

### Gravity Scale

```python
# Default: 1.0 (assumes accurate URDF)
gravity_scale: 1.0

# If robot sags: increase
gravity_scale: 1.05  # 5% more compensation

# If robot drifts up: decrease
gravity_scale: 0.95  # 5% less compensation
```

### Damping Gains

```python
# Conservative (stable but may oscillate)
damping_gains: [1.0, 1.0, 0.8, 0.8, 0.3, 0.3, 0.3]

# Aggressive (faster settling, may overshoot)
damping_gains: [5.0, 5.0, 3.0, 3.0, 1.0, 1.0, 1.0]

# Per-joint tuning based on inertia
# Larger joints (1-2): higher damping
# Smaller joints (5-7): lower damping
```

### Coriolis Compensation

```python
# Static holding: not needed
add_coriolis: false

# Slow motion (< 1 rad/s): usually not needed
add_coriolis: false

# Fast motion (> 2 rad/s): recommended
add_coriolis: true

# High-speed coordinated motion: essential
add_coriolis: true
```

---

## Troubleshooting

### Robot Falls Despite Gravity Compensation

**Possible causes:**

1. **Gravity compensation not running**

   ```bash
   # Check if controller is publishing
   ros2 topic hz /effort_controller/commands
   # Should show ~100 Hz
   ```

2. **Effort controller not active**

   ```bash
   ros2 control list_controllers
   # effort_controller should be [active]
   ```

3. **URDF masses inaccurate**
   - Try increasing `gravity_scale` to 1.1-1.2
   - Check URDF link masses match real hardware

4. **Torque limits exceeded**

   ```
   Joint 1-2: ±40 N·m
   Joint 3-4: ±27 N·m
   Joint 5-7: ±7 N·m
   ```

   Check if computed torques exceed these limits

### Robot Oscillates

**Possible causes:**

1. **Insufficient damping**
   - Increase `damping_gains` by 20-50%

2. **Control rate too low**
   - Default 100 Hz should be sufficient
   - Try 200 Hz: `control_rate: 200.0`

3. **PID interference**
   - Make sure position controller is deactivated
   - Only effort controller should be active

### Robot Drifts Slowly

**Possible causes:**

1. **Model inaccuracy (most likely)**
   - Tune `gravity_scale` parameter
   - Check link masses in URDF

2. **Missing friction compensation**
   - Joint friction creates small steady-state error
   - Can add simple friction model:

     ```python
     tau_friction = -K_friction * np.sign(dq)
     ```

3. **Numerical integration issues**
   - Usually not the problem with MuJoCo
   - Check timestep in MJCF (should be 0.001s)

---

## Integration with CartesianController

The gravity compensation demonstrated here is the same feedforward computation that `crisp_controllers/CartesianController` performs internally.

**CartesianController computes:**

```cpp
tau_d = tau_task           // Cartesian impedance control
      + tau_nullspace      // Elbow/posture control
      + tau_gravity        // g(q) - same as your controller!
      + tau_coriolis       // C(q,q̇)·q̇ - same as your controller!
      + tau_friction       // Joint friction compensation
      + tau_joint_limits   // Soft limit repulsion
      + tau_wrench;        // External force application
```

Your standalone gravity compensation controller is essentially:

```cpp
tau_d = tau_gravity + tau_damping (+ optional tau_coriolis)
```

This validates that your dynamics model is working correctly before using the full Cartesian controller.

---

## Next Steps

### Immediate

1. ✅ Run the test suite: `python3 scripts/test_gravity_compensation.py`
2. ✅ Verify all tests pass with default parameters
3. ✅ If tests fail, tune `gravity_scale` and `damping_gains`

### Short-term

1. Test with Coriolis compensation during motion:

   ```bash
   python3 scripts/gravity_compensation_controller.py \
     --ros-args -p add_coriolis:=true
   ```

2. Add friction compensation if you observe steady-state drift

3. Validate against real hardware (when available)

### Long-term

1. ✅ **External force application** - Now implemented via `ApplyExternalWrench` service

2. **Log and compare** Pinocchio vs. MuJoCo dynamics:
   - Export qfrc_bias via custom topic
   - Compare with Pinocchio computation
   - Quantify model accuracy

3. **System identification**:
   - Measure real robot link masses/inertias
   - Update URDF with accurate values
   - Minimize `gravity_scale` deviation from 1.0

---

## References

### Internal Documentation

- **MuJoCo Integration**: [`mujoco_status.md`](mujoco_status.md)
- **Torque vs. Actuators**: [`torque_vs_actuator_control.md`](torque_vs_actuator_control.md)
- **Controller Architecture**: [`controllers.md`](controllers.md)
- **Setup Guide**: [`ZORDI_README.md`](ZORDI_README.md)

### External Resources

- **MuJoCo Documentation**: <https://mujoco.readthedocs.io/en/stable/>
- **Pinocchio**: <https://stack-of-tasks.github.io/pinocchio/>
- **ROS2 Control**: <https://control.ros.org/>

### Academic References

- Featherstone, "Rigid Body Dynamics Algorithms" (2008) - Inverse dynamics
- Siciliano et al., "Robotics: Modelling, Planning and Control" (2009) - Robot dynamics
- Slotine & Li, "Applied Nonlinear Control" (1991) - Computed torque control

---

## Summary

### Key Takeaways

1. ✅ **Your approach is correct**: Compute `g(q)` with Pinocchio, not from MuJoCo
2. ✅ **This matches real hardware**: Same code works on real robot
3. ✅ **Enhanced implementation**: Now supports Coriolis, better testing
4. ⚠️ **Never read `qfrc_bias`**: Defeats purpose of model validation
5. 📊 **Test systematically**: Use provided test suite for validation

### What You've Achieved

- ✅ Proper gravity compensation implementation
- ✅ Model-based approach (not sim-dependent)
- ✅ Systematic test suite with external force perturbations
- ✅ Hardware-ready control strategy
- ✅ Headless testing capability

### What's Next

- Run tests and validate performance
- Tune parameters if needed
- Test with interactive viewer perturbations (Ctrl+Right-click)
- Integrate with CartesianController for full dynamics control

---

**Copyright 2025 Zordi, Inc. All rights reserved.**
