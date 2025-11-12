# Direct Torque Control vs MuJoCo Actuators

**Purpose**: Comprehensive guide on the two approaches to controlling robots in MuJoCo simulation

**For OpenARM**: We use direct torque control (`qfrc_applied`), not MuJoCo actuators (`ctrl`)

---

## Table of Contents

1. [Quick Summary](#quick-summary)
2. [Detailed Comparison](#detailed-comparison)
3. [Why We Chose Direct Torque Control](#why-we-chose-direct-torque-control)
4. [When to Use MuJoCo Actuators](#when-to-use-mujoco-actuators)
5. [Implementation Details](#implementation-details)
6. [Hybrid Approach](#hybrid-approach)

---

## Quick Summary

### Primary: Direct Torque Application (qfrc_applied) - MIT Mode

```cpp
// In mujoco_ros2_control - MIT mode
mj_data->qfrc_applied[joint_idx] = computed_torque;
```

- Write torques directly to MuJoCo's generalized force array
- Used for full MIT mode control
- Force enters directly into forward dynamics equation

### Secondary: MJCF Actuator Inputs (ctrl) - Position Servo Mode

```cpp
// In mujoco_ros2_control - position_servo mode
mj_data->ctrl[actuator_idx] = position_command;
```

- Uses MuJoCo's built-in position actuators (kp=50000)
- Triggered when joint_trajectory_controller active
- Provides comparison with firmware-controlled Position Mode

**We use BOTH approaches with dynamic switching!**

### Key Point

**Generated MJCF files INCLUDE `<actuator>` section** - position actuators with kp=50000 for position_servo mode. We use BOTH qfrc_applied (MIT mode) and ctrl (position_servo mode) with dynamic switching.

---

## Detailed Comparison

### Option 1: Direct Torque Application (qfrc_applied) ✅ **Current Implementation**

#### How it Works

1. Compute desired torque externally (PID, gravity comp, etc.)
2. Write directly to `qfrc_applied[]`
3. MuJoCo forward dynamics uses this as external force
4. No actuator model in between

**MuJoCo Equation:**

```
M(q)·q̈ = qfrc_applied + qfrc_bias + qfrc_constraint
```

#### Pros

| Advantage | Description |
|-----------|-------------|
| **Transparent Control** | What you command is what you get - no hidden dynamics |
| **Simplified Debugging** | Direct cause-and-effect relationship |
| **Hardware Matching** | OpenARM Damiao motors use torque control mode |
| **No Parameter Tuning** | Don't need actuator gain, bias, transmission parameters |
| **Performance** | Skips actuator computation (minor speedup) |
| **Flexible Control** | Implement any control law externally (PID, impedance, etc.) |

#### Cons

| Limitation | Description |
|------------|-------------|
| **No Actuator Realism** | Doesn't model motor dynamics (back-EMF, saturation) |
| **Manual Clamping** | Must clamp torques to limits in code |
| **Less Modularity** | Control logic in C++, not declarative MJCF |
| **No Actuator Sensors** | Can't use MuJoCo's actuator force/velocity sensors |

#### When to Use

- ✅ Real hardware uses direct torque control (like OpenARM)
- ✅ Implementing custom control algorithms
- ✅ Want predictable, transparent behavior
- ✅ Debugging control issues
- ✅ Simplicity is a priority

---

### Option 2: MJCF Actuator Inputs (ctrl) ❌ **Not Currently Used**

#### How it Works

1. Write command to `ctrl[]` array
2. MuJoCo actuator model processes command
3. Actuator generates forces: `force = gain * (ctrl - bias) * transmission`
4. Force enters forward dynamics equation

**Force Generation:**

```
qfrc_actuator = transmission^T * gain * (ctrl - bias) * activation
```

#### Available Actuator Types

| Type | Description | Use Case |
|------|-------------|----------|
| `motor` | Simple torque actuator | Direct drive motors |
| `position` | Position servo (PD control) | Servo motors |
| `velocity` | Velocity servo (PI control) | Speed-controlled motors |
| `intvelocity` | Integrated velocity | Smooth velocity control |
| `damper` | Damping actuator | Passive damping |
| `cylinder` | Pneumatic/hydraulic | Air/fluid cylinders |
| `muscle` | Hill-type muscle model | Biological systems |

#### Pros

| Advantage | Description |
|-----------|-------------|
| **Built-in Dynamics** | Models realistic motor behavior (saturation, activation) |
| **Declarative Config** | Actuator parameters in MJCF, no recompile needed |
| **Advanced Types** | Servos, muscles, hydraulics available |
| **Auto Clamping** | `ctrlrange` enforces limits automatically |
| **Actuator Sensors** | Can use actuatorpos, actuatorvel, actuatorfrc |
| **Modularity** | Swap actuator models without changing code |

#### Cons

| Limitation | Description |
|------------|-------------|
| **Added Complexity** | Need to tune gain, bias, dynprm parameters |
| **Indirect Control** | Command ≠ applied force (transfer function) |
| **Debugging Difficulty** | Extra layer between command and effect |
| **Parameter Uncertainty** | Real motor parameters may not be known |
| **Performance Overhead** | Actuator computation adds cost (minor) |
| **Hardware Mismatch** | If hardware uses torque control, simulation differs |

#### When to Use

- Complex actuator dynamics matter (muscles, pneumatics)
- Want built-in position/velocity servo behavior
- Studying actuator model effects (motor saturation)
- Need declarative, MJCF-based configuration
- System has non-trivial actuator dynamics

---

## Why We Chose Direct Torque Control

### Reason 1: Hardware Matching

**Real OpenARM Hardware:**

```
Controller → τ_cmd → Damiao Motor (torque mode) → τ_actual
```

**Simulation Should Match:**

```
Controller → τ_cmd → qfrc_applied → MuJoCo dynamics
```

OpenARM uses Damiao motors in **torque control mode**, not position/velocity servos. Direct torque application in simulation matches this behavior.

### Reason 2: Control Architecture

```
High-Level Controller (crisp_controllers, CartesianController)
    ↓
Computes desired torques (PID, gravity comp, impedance, etc.)
    ↓
ros2_control command interface
    ↓
mujoco_ros2_control writes to qfrc_applied
```

Our controllers **already** handle:

- PID control
- Gravity compensation
- Impedance control
- Dynamics compensation

**No need for MuJoCo to add another control layer.**

### Reason 3: Transparency

During development and debugging:

- Direct relationship: `commanded_torque → applied_force`
- No hidden actuator dynamics to account for
- Easy to understand cause and effect
- Simplified troubleshooting

### Reason 4: Simplicity

**Don't need to identify motor parameters:**

- Motor constant (Kt)
- Back-EMF constant (Kv)
- Resistance
- Inductance
- Gear ratios
- Saturation curves

**Fewer sources of model mismatch** between simulation and reality.

---

## When to Use MuJoCo Actuators

### Scenario 1: Built-in Servo Control

If you want MuJoCo to handle PD position control:

```xml
<actuator>
  <position name="joint1_servo" joint="openarm_joint1"
            kp="100" kv="10" ctrlrange="-1.4 3.5" />
</actuator>
```

Then command desired positions to `ctrl[]` and MuJoCo does the control.

**Advantage**: No need to implement position controller externally.

### Scenario 2: Studying Actuator Dynamics

If motor saturation, back-EMF, or activation dynamics are important:

```xml
<actuator>
  <motor name="joint1_motor" joint="openarm_joint1"
         gear="100" dynprm="1 0 0"
         ctrlrange="-24 24" />  <!-- Voltage limits -->
</actuator>
```

Model realistic motor transfer function and study effects of actuator dynamics on control performance.

### Scenario 3: Biological Systems

For muscle-actuated robots:

```xml
<actuator>
  <muscle name="biceps" joint="elbow"
          tausmooth="0.01 0.5" />
</actuator>
```

Muscles have activation buildup and Hill-type force-velocity curves - crucial for biological realism.

### Scenario 4: Hydraulic/Pneumatic Systems

For systems with fluid actuators:

```xml
<actuator>
  <cylinder name="hydraulic_ram" joint="knee"
            area="0.01" diameter="0.05" />
</actuator>
```

Pneumatic and hydraulic actuators have very different dynamics than electric motors.

### Scenario 5: Hardware Uses Servos

If your real robot uses position or velocity servos (not torque control):

```xml
<actuator>
  <velocity name="joint1_servo" joint="openarm_joint1"
            kv="50" ctrlrange="-10 10" />
</actuator>
```

Match simulation to hardware control mode.

---

## Implementation Details

### Auto-Generated Actuators Are Removed

The `urdf2mjcf` converter automatically generates `<actuator>` elements for all joints:

```xml
<!-- Auto-generated by urdf2mjcf -->
<actuator>
  <motor name="openarm_joint1" joint="openarm_joint1"
         ctrllimited="true" ctrlrange="-40 40" gear="1" />
  <motor name="openarm_joint2" joint="openarm_joint2"
         ctrllimited="true" ctrlrange="-40 40" gear="1" />
  <!-- ... 7 motors total -->
</actuator>
```

**Our conversion script automatically removes these** during post-processing.

### Why Remove Auto-Generated Actuators?

- ✅ **Clarity** - Makes it clear we use `qfrc_applied`, not `ctrl[]`
- ✅ **Cleaner MJCF** - Generated files match our control approach
- ✅ **Avoid Confusion** - No unused actuator elements
- ✅ **No URDF Changes** - Actuators come from converter, not source URDF

### Implementation in `urdf_to_mjcf.py`

```python
def add_position_actuators(root, joint_limits, kp=50000.0, kv=200.0):
    """Add high-gain position actuators for position_servo mode."""
    actuator_elem = ET.SubElement(root, "actuator")

    for joint_name, limits in joint_limits.items():
        ET.SubElement(actuator_elem, "position",
            name=f"actuator_openarm_{joint_name}",
            joint=f"openarm_{joint_name}",
            kp=str(kp),    # High stiffness (firmware-like)
            kv=str(kv),    # Damping
            ctrlrange=f"{limits['lower']} {limits['upper']}",
            forcerange=f"-{limits['effort']} {limits['effort']}"
        )
```

### Result

When you run `python3 scripts/urdf_to_mjcf.py`, the output MJCF has:

- ✅ Position actuators with kp=50000 (high stiffness)
- ✅ Proper force limits per joint (40/27/7 Nm)
- ✅ Supports both direct torque (MIT) and actuators (position_servo)

### Regenerate Clean MJCF Files

```bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml

# You'll see:
# ✓ Removed auto-generated actuators (using qfrc_applied instead)
```

---

## Hybrid Approach

### Using Both Methods Simultaneously

It's possible to use **both** `qfrc_applied` and `ctrl`:

**Example:**

```cpp
// Primary control via direct torque
for (int i = 0; i < 7; i++) {
    mj_data->qfrc_applied[arm_joint_idx[i]] = arm_torques[i];
}

// Gripper via MuJoCo actuator
mj_data->ctrl[gripper_actuator_idx] = desired_gripper_position;
```

**Use Case:**

- Arm joints: Direct torque control (transparent, debuggable)
- Gripper: MuJoCo position actuator (simple position control)

### Mode-Based Switching

Switch between methods based on control mode:

```cpp
if (control_mode == "effort") {
    // Direct torque control
    mj_data->qfrc_applied[i] = effort_command;
} else {
    // Use MuJoCo actuators for position/velocity
    mj_data->ctrl[actuator_idx] = position_command;
}
```

**Trade-off**: Increased complexity for potentially more realistic simulation.

---

## Technical Note: Actuator Force Generation

### MuJoCo Actuator Force Equation

```
qfrc_actuator = transmission^T * gain * (ctrl - bias) * activation

Where:
  transmission: Maps actuator space to joint space (gear ratio, moment arms)
  gain:         Actuator strength / motor constant
  bias:         Neutral point offset
  activation:   Dynamics state (0 to 1, muscles have activation buildup)
  ctrl:         Control input (what you command)
```

### With qfrc_applied

You **directly set the end result**, bypassing this entire computation:

```cpp
// Skip all actuator dynamics, directly set the force
mj_data->qfrc_applied[joint_idx] = desired_torque;
```

### Forward Dynamics Integration

Both approaches feed into the same forward dynamics:

```
M(q)·q̈ = qfrc_applied + qfrc_actuator + qfrc_bias + qfrc_constraint

Where:
  M(q):             Mass/inertia matrix
  qfrc_applied:     External forces (our direct torques)
  qfrc_actuator:    Forces from MJCF actuators
  qfrc_bias:        Gravity + Coriolis + centrifugal
  qfrc_constraint:  Contact and joint limit forces
```

---

## Recommendation

### For OpenARM (Current Use Case)

✅ **Keep using qfrc_applied**

- Matches real hardware torque control
- Implement control externally (Pinocchio, crisp_controllers)
- Simple, transparent, debuggable
- No actuator parameter uncertainty

### For Future Robots

**Consider MuJoCo actuators if:**

- Hardware uses position/velocity servos (not torque control)
- Studying motor dynamics is critical to research goals
- System has significant actuator dynamics (muscles, hydraulics)
- Want declarative configuration without code changes

**Stick with qfrc_applied if:**

- Hardware uses torque control
- Simplicity and transparency are priorities
- Implementing custom control algorithms externally
- Want to minimize simulation/reality mismatch

---

## References

- **MuJoCo Documentation**: <https://mujoco.readthedocs.io/en/stable/modeling.html#actuator>
- **OpenARM Integration**: `docs/mujoco_status.md`
- **Conversion Script**: `scripts/urdf_to_mjcf.py`

---

**Copyright 2025 Zordi, Inc. All rights reserved.**
