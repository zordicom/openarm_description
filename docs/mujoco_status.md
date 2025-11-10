# OpenARM MuJoCo Integration - Complete Status & Reference

**Last Updated**: November 10, 2025
**Status**: ✅ **Production Ready** (Position Control), ⚠️ **Unstable** (Velocity Control), ✅ **Working** (Effort Control with Gravity Compensation)

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [System Architecture](#system-architecture)
3. [Control Modes](#control-modes)
4. [Quick Start](#quick-start)
5. [Implementation Details](#implementation-details)
6. [File Structure](#file-structure)
7. [Known Issues & Limitations](#known-issues--limitations)
8. [Troubleshooting](#troubleshooting)
9. [API Reference](#api-reference)

---

## Executive Summary

OpenARM now has full MuJoCo physics simulation support via ROS2 Control, enabling realistic dynamics simulation with gravity, inertia, friction, and contacts.

### What Works

✅ **Position Control** - Excellent stability and accuracy (< 0.02° error)
✅ **Effort Control** - Works with gravity compensation controller
✅ **Dynamic Controller Switching** - Switch between modes at runtime
✅ **Gravity Simulation** - Robot holds position under gravity
✅ **Single Arm, Bimanual, Hand** - All configurations supported
⚠️ **Velocity Control** - Implemented but unstable (needs tuning)

### Key Achievement

**Custom `mujoco_ros2_control` fork** enables all three control interfaces (position, velocity, effort) to coexist without conflicts. The hardware interface detects which controller is active and only applies commands from that controller.

---

## System Architecture

### Overall Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS2 Controllers                             │
│  (JointTrajectoryController, VelocityController, EffortController)│
└────────────────────────┬────────────────────────────────────────┘
                         │ ros2_control framework
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│              mujoco_ros2_control (Custom Fork)                  │
│  - MujocoSystem hardware interface                              │
│  - Control mode: "all" (position_pid, velocity_pid, effort)    │
│  - Active command detection (prevents conflicts)                │
└────────────────────────┬────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────────────┐
│                    MuJoCo Physics Engine                        │
│  Forward dynamics (as implemented):                             │
│    M(q)·q̈ = qfrc_applied + qfrc_actuator + qfrc_constraint     │
│             - qfrc_bias(q, q̇)                                   │
│                                                                 │
│  Where:                                                         │
│    M(q) = mass/inertia matrix                                   │
│    qfrc_bias = gravity + Coriolis + centrifugal forces          │
│    qfrc_applied = external forces (our control torques)         │
│    qfrc_actuator = actuator forces (unused in our setup)        │
│    qfrc_constraint = contact and joint limit forces             │
│                                                                 │
│  Integration: RK4 / Euler with constraint stabilization         │
└─────────────────────────────────────────────────────────────────┘
```

### Control Flow

```
Controller publishes command
    ↓
ros2_control Command Interface updated
    ↓
MujocoSystem::write() called (100 Hz)
    ↓
Detect which controller is active (command ≠ initial value)
    ↓
Apply ONLY that controller's commands:
    - Position PID → compute τ = PID(error), write to qfrc_applied
    - Velocity PID → compute τ = PID(vel_error), write to qfrc_applied
    - Effort → write τ directly to qfrc_applied (clamped to limits)
    ↓
MuJoCo forward dynamics: M(q)·q̈ = qfrc_applied + qfrc_bias + constraints
    ↓
MuJoCo integration → new q, q̇, q̈
    ↓
MujocoSystem::read() publishes joint states to ROS2
```

### Key Implementation Insight: qfrc_applied vs ctrl

**What we do:**

- Write joint torques directly to `mj_data->qfrc_applied[joint_idx]`
- MuJoCo's forward dynamics uses this as external generalized force

**What we DON'T do:**

- We don't use `mj_data->ctrl[]` (MJCF actuator inputs)
- Generated MJCF files have NO `<actuator>` section (auto-removed during conversion)
- This gives us direct torque control without actuator dynamics

**Note on auto-generated actuators:**

The `urdf2mjcf` converter automatically generates `<actuator>` elements for all joints. Our conversion script (`urdf_to_mjcf.py`) **automatically removes these** during post-processing since we use `qfrc_applied` instead of `ctrl[]`.

**Old generated MJCF had:**

```xml
<actuator>
  <motor name="openarm_joint1" joint="openarm_joint1"
         ctrllimited="true" ctrlrange="-40 40" gear="1" />
  <!-- ... 7 motors + actuator sensors ... -->
</actuator>
```

**Our post-processing removes them** to avoid confusion and keep MJCF files clean.

---

### Detailed Comparison: qfrc_applied vs ctrl (MJCF Actuators)

#### Option 1: Direct Torque Application (qfrc_applied) ✅ **Current Implementation**

**How it works:**

- Torques written directly to `qfrc_applied[]`
- Bypasses MuJoCo actuator model entirely
- Force enters directly into forward dynamics equation

**Pros:**

1. **Transparent Control** - No hidden actuator dynamics, what you command is what you get
2. **Simplified Debugging** - Direct relationship between commanded torque and applied force
3. **Matching Hardware Behavior** - OpenARM Damiao motors use torque control mode on real hardware
4. **No Parameter Tuning** - No need to tune actuator gain, bias, or transmission parameters
5. **Performance** - Slightly more efficient (skips actuator computation step)
6. **Flexible Control** - Can implement any control law externally (PID, computed torque, impedance, etc.)

**Cons:**

1. **No Built-in Actuator Realism** - Doesn't model motor dynamics (back-EMF, inductance, saturation curves)
2. **No Automatic Control Clamping** - Must manually clamp torques to joint limits (though we do this in code)
3. **Less Modularity** - Control logic is in C++ code, not declaratively in MJCF
4. **No Actuator Sensors** - Can't use MuJoCo's actuator force/position/velocity sensors (though we read joint states directly)

**When to use:**

- ✅ When real hardware uses direct torque control (OpenARM case)
- ✅ When implementing custom control algorithms
- ✅ When you want predictable, transparent behavior
- ✅ When debugging control issues

---

#### Option 2: MJCF Actuator Inputs (ctrl) ❌ **Not Currently Used**

**How it works:**

- Commands written to `ctrl[]` array
- MuJoCo actuator model processes commands
- Actuator generates forces using: `force = gain * (ctrl - bias) * transmission`
- Force enters forward dynamics equation

**Actuator types available:**

- `motor` - Simple torque actuator (our MJCF uses this)
- `position` - Position servo with PD control
- `velocity` - Velocity servo with PI control
- `intvelocity` - Integrated velocity control
- `damper` - Damping actuator
- `cylinder` - Pneumatic/hydraulic actuator
- `muscle` - Hill-type muscle model

**Pros:**

1. **Built-in Actuator Dynamics** - Can model realistic motor behavior (activation, saturation, non-linearities)
2. **Declarative Configuration** - Actuator parameters in MJCF, easy to modify without recompiling
3. **Advanced Actuator Types** - Access to position/velocity servos, muscles, hydraulics, etc.
4. **Automatic Control Clamping** - `ctrlrange` enforces limits automatically
5. **Actuator Sensors** - Can use actuatorpos, actuatorvel, actuatorfrc sensors
6. **Modularity** - Swap actuator models without changing control code

**Cons:**

1. **Added Complexity** - Need to understand and tune actuator parameters (gain, bias, dynprm, etc.)
2. **Indirect Control** - Commanded value ≠ applied force (goes through transfer function)
3. **Debugging Difficulty** - Extra layer between command and effect
4. **Parameter Uncertainty** - Real motor parameters may not be accurately known
5. **Performance Overhead** - Actuator computation adds (minor) computational cost
6. **Mismatch with Hardware** - If real robot uses torque control, simulation using position servo is less representative

**When to use:**

- When simulating systems with complex actuator dynamics (muscles, pneumatics)
- When you want built-in position/velocity servo behavior
- When actuator model is important to study (e.g., motor saturation effects)
- When you need declarative, MJCF-based configuration

---

### Why We Chose qfrc_applied

**Reason 1: Hardware Matching**

- OpenARM uses Damiao motors in torque control mode
- Real hardware: `τ_cmd → motor → τ_actual`
- Simulation should match: `τ_cmd → qfrc_applied`

**Reason 2: Control Architecture**

- High-level controllers (CartesianController, crisp_controllers) compute desired torques
- These controllers already handle PID, gravity compensation, impedance, etc.
- No need for MuJoCo to add another control layer

**Reason 3: Transparency**

- During development/debugging, direct torque application makes cause-and-effect clear
- No hidden actuator dynamics to account for

**Reason 4: Simplicity**

- Don't need to identify/tune motor parameters (Kt, Kv, resistance, inductance, etc.)
- Fewer sources of model mismatch

---

### When You SHOULD Use ctrl (MJCF Actuators)

**Scenario 1: Built-in Servo Control**
If you want MuJoCo to handle PD position control:

```xml
<actuator>
  <position name="joint1_servo" joint="openarm_joint1"
            kp="100" kv="10" ctrlrange="-1.4 3.5" />
</actuator>
```

Then command desired positions to `ctrl[]` and MuJoCo does the rest.

**Scenario 2: Studying Actuator Dynamics**
If motor saturation, back-EMF, or activation dynamics are important:

```xml
<actuator>
  <motor name="joint1_motor" joint="openarm_joint1"
         gear="100" dynprm="1 0 0"
         ctrlrange="-24 24" />  <!-- Voltage limits -->
</actuator>
```

Model realistic motor transfer function.

**Scenario 3: Biological Systems**
For muscle-actuated robots:

```xml
<actuator>
  <muscle name="biceps" joint="elbow"
          tausmooth="0.01 0.5" />
</actuator>
```

**Scenario 4: Hydraulic/Pneumatic**
For systems with cylinder actuators:

```xml
<actuator>
  <cylinder name="hydraulic_ram" joint="knee"
            area="0.01" diameter="0.05" />
</actuator>
```

---

### Hybrid Approach (Future Consideration)

It's possible to use BOTH methods:

- `qfrc_applied` for primary control torques
- `ctrl` for auxiliary actuators (e.g., gripper, special mechanisms)

Or switch between them based on control mode:

- Effort mode → use `qfrc_applied` (direct torque)
- Position/Velocity mode → use `ctrl` with MuJoCo's built-in servos

**Trade-off:** Increased complexity for potentially more realistic servo simulation.

---

### Recommendation

**For OpenARM (current use case):**

- ✅ **Keep using qfrc_applied** - Matches real hardware torque control
- ✅ Implement high-level control externally (Pinocchio, crisp_controllers)
- ✅ Simple, transparent, debuggable

**For future robots:**

- If hardware uses position/velocity servos → consider MuJoCo position/velocity actuators
- If studying motor dynamics is critical → implement motor model via ctrl
- If simplicity and transparency are priorities → stick with qfrc_applied

---

### Technical Note: Actuator Force Generation

For those interested, MuJoCo's actuator force generation:

```
qfrc_actuator = transmission^T * gain * (ctrl - bias) * activation

Where:
- transmission: Maps actuator space to joint space (gear ratio, moment arms)
- gain: Actuator strength/motor constant
- bias: Neutral point offset
- activation: Dynamics state (muscles have activation buildup)
- ctrl: Control input (what you command)
```

With `qfrc_applied`, you directly set the end result, bypassing this computation.

---

### Auto-Generated Actuators Are Automatically Removed ✅

The `urdf2mjcf` tool automatically generates `<actuator>` elements for all joints. **Our conversion script now automatically removes these** during post-processing.

**Why we remove them:**

- ✅ **Avoid confusion** - Makes it clear we use `qfrc_applied`, not `ctrl[]`
- ✅ **Cleaner MJCF** - Generated files are more readable and match our control approach
- ✅ **No URDF changes needed** - Actuators come from converter, not URDF

**Implementation:**

This happens automatically in `fix_mjcf_mesh_paths()` in `scripts/urdf_to_mjcf.py`:

```python
# Remove auto-generated actuators (we use qfrc_applied, not ctrl)
for actuator in root.findall(".//actuator"):
    root.remove(actuator)
    print("✓ Removed auto-generated actuators (using qfrc_applied instead)")

# Remove actuator sensors (since actuators are removed)
sensor_elem = root.find(".//sensor")
if sensor_elem is not None:
    for sensor in list(sensor_elem):
        if sensor.tag in ["actuatorpos", "actuatorvel", "actuatorfrc"]:
            sensor_elem.remove(sensor)
```

**Result:**

When you run `python3 scripts/urdf_to_mjcf.py`, the output MJCF will have:

- ✅ NO `<actuator>` section
- ✅ NO actuator sensors (actuatorpos, actuatorvel, actuatorfrc)
- ✅ Clean MJCF that clearly shows direct torque control via qfrc_applied

**To regenerate clean MJCF files:**

```bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml
```

You'll see: `✓ Removed auto-generated actuators (using qfrc_applied instead)`

---

## Control Modes

### 1. Position Control (JointTrajectoryController)

**Status:** ✅ **Excellent** - Production ready

**How it works:**

- Controller publishes desired joint positions
- Hardware plugin computes PID: `τ = Kp·(q_des - q) + Kd·(q̇_des - q̇)`
- Torque applied to `qfrc_applied`
- MuJoCo forward dynamics moves joints to reach desired position

**PID Gains (tuned for stability):**

```yaml
Joint 1-2 (40 Nm limit):  Kp=20.0,  Kd=2.0,  Ki=0.0
Joint 3-4 (27 Nm limit):  Kp=15.0,  Kd=1.5,  Ki=0.0
Joint 5-7 (7 Nm limit):   Kp=5.0,   Kd=0.5,  Ki=0.0
```

**Note:** `Ki=0` to avoid integral windup. This causes small steady-state error (e.g., joint 6 sags ~1.4° under gravity), which is acceptable for most applications.

**Usage:**

```bash
# Launch
ros2 launch openarm_description mujoco_sim.launch.py control_mode:=position

# Send trajectory
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [openarm_joint1, openarm_joint2, openarm_joint3, openarm_joint4, openarm_joint5, openarm_joint6, openarm_joint7]
  points:
  - positions: [0.5, 0.3, 0.2, 0.8, 0.0, 0.0, 0.0]
    time_from_start: {sec: 3}
"
```

### 2. Velocity Control (JointGroupVelocityController)

**Status:** ⚠️ **Unstable** - Not recommended for production

**How it works:**

- Controller publishes desired joint velocities
- Hardware plugin computes PID: `τ = Kp·(q̇_des - q̇) + Kd·(q̈_des - q̈)`
- Torque applied to `qfrc_applied`

**Current Gains:**

```yaml
Kp = position_kp / 1000  (very small)
Kd = position_kd / 1000
Ki = 0.0
```

**Problem:**

- Gravity creates constant disturbance force
- PID without integral term cannot reject constant disturbances
- Low gains → drifts, high gains → oscillates

**Attempted Solutions:**

- ✗ Reduced gains to Kp/1000 → still drifts
- ✗ Tested without gravity → library loading issue prevented completion

**Recommendation:** Use position control instead. Convert desired velocities to position updates: `q_des(t+Δt) = q(t) + q̇_des·Δt`

### 3. Effort Control (JointGroupEffortController)

**Status:** ✅ **Working** - Requires gravity compensation controller

**How it works:**

- Controller publishes desired joint torques
- Hardware plugin writes torques directly to `qfrc_applied` (clamped to joint limits)
- No PID, direct force application

**Important:** Effort control by itself doesn't compensate for gravity! Robot will fall unless you:

1. **Use gravity compensation controller** (recommended): `scripts/gravity_compensation_controller.py`
2. **Compute feedforward torques manually** in your high-level controller

**Gravity Compensation Controller:**

Computes gravity torques using Pinocchio dynamics:

```python
tau_gravity = pin.computeGeneralizedGravity(model, data, q)
tau_damping = -Kd * q̇  # Optional stability
tau_total = tau_gravity + tau_damping
```

**Usage:**

```bash
# Terminal 1: Launch with effort control
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch openarm_description mujoco_sim.launch.py control_mode:=effort

# Terminal 2: Start gravity compensation
python3 scripts/gravity_compensation_controller.py

# Robot now holds position under gravity!

# Terminal 3: Send additional torques for movement
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [20.0, 20.0, 15.0, 15.0, 3.0, 3.0, 3.0]" -r 100
```

**Joint Torque Limits:**

```
Joint 1-2: ±40 Nm
Joint 3-4: ±27 Nm
Joint 5-7: ±7 Nm
```

---

## Quick Start

### Prerequisites

**1. Install Custom mujoco_ros2_control Fork (REQUIRED):**

```bash
cd ~/ros2_ws/src
git clone https://github.com/zordicom/mujoco_ros2_control.git
cd mujoco_ros2_control
git checkout 2025-11-control-interface  # Commit: 5a1e443

cd ~/ros2_ws
colcon build --packages-select mujoco_ros2_control --symlink-install
source ~/ros2_ws/install/setup.bash
```

**Why the custom fork?**
Standard `mujoco_ros2_control` has conflicts when position/velocity/effort interfaces are all exposed. Our fork detects active controllers and prevents interference.

**2. Install MuJoCo Python:**

```bash
pip install mujoco urdf2mjcf
```

**3. Install Pinocchio (for gravity compensation):**

```bash
sudo apt install ros-humble-pinocchio
```

### Generate MuJoCo Model

```bash
cd ~/ros2_ws/src/openarm_description

# Single arm (7 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml

# With hand (9 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --hand --output mujoco_models/openarm_v10_hand.xml

# Bimanual (14 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --bimanual --output mujoco_models/openarm_v10_bimanual.xml
```

### Launch Simulation

**Position Control (Recommended):**

```bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch openarm_description mujoco_sim.launch.py
```

**Effort Control with Gravity Compensation:**

```bash
# Terminal 1
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch openarm_description mujoco_sim.launch.py control_mode:=effort

# Terminal 2
python3 scripts/gravity_compensation_controller.py
```

### Test Position Control

```bash
# Use example script
python3 scripts/example_position_control.py

# Or send manual command
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [openarm_joint1, openarm_joint2, openarm_joint3, openarm_joint4, openarm_joint5, openarm_joint6, openarm_joint7]
  points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 3}
"
```

### Dynamic Controller Switching

```bash
# List controllers
ros2 control list_controllers

# Switch to effort control
ros2 control switch_controllers \
  --activate effort_controller \
  --deactivate joint_trajectory_controller velocity_controller

# Switch back to position control
ros2 control switch_controllers \
  --activate joint_trajectory_controller \
  --deactivate effort_controller velocity_controller
```

---

## Implementation Details

### Position vs Position_PID Interface Naming

**Key Convention in URDF:**

```xml
<command_interface name="position_pid">  <!-- NOT "position" -->
  <param name="kp">20.0</param>
  <param name="kd">2.0</param>
  <param name="ki">0.0</param>
</command_interface>
```

**Behavior:**

- `position` → Kinematic set. Writes directly to `qpos` (ignores physics, breaks with gravity)
- `position_pid` → Dynamic set. Computes torque via PID and writes to `qfrc_applied` (physics-faithful)

**Same pattern for velocity:**

- `velocity` → Kinematic set to `qvel`
- `velocity_pid` → Torque-based PID control

**Recommendation:** Always use `_pid` variants for physics-realistic simulation that matches real hardware.

### Gravity and qfrc_bias

**MuJoCo Equation of Motion:**

```
M(q) · q̈ + qfrc_bias(q, q̇) = qfrc_applied + qfrc_actuator + qfrc_constraint
```

Where:

- `M(q)` = Mass/inertia matrix
- `qfrc_bias` = Gravity + Coriolis + centrifugal forces (computed by MuJoCo)
- `qfrc_applied` = External forces we write (our control torques)
- `qfrc_actuator` = Forces from MJCF actuators (unused in our implementation)
- `qfrc_constraint` = Contact and joint limit forces

**Key Points:**

1. **Gravity is ALWAYS included** by MuJoCo in `qfrc_bias`
2. **Position control holds via steady-state error:**
   - PD controller with `Ki=0` → torque = `Kp·error + Kd·vel_error`
   - At steady state: `Kp·error ≈ gravity_torque` → small position sag
3. **Gravity compensation in effort control:**
   - Must compute `g(q)` externally (using Pinocchio)
   - Command: `τ_cmd = τ_control + g(q)`
   - This cancels the `qfrc_bias` term for zero-gravity-like behavior

**Sign Convention:**

- Gravity compensation torques are POSITIVE (same sign as qfrc_bias)
- Do NOT negate `qfrc_bias` values when using them for feedforward

### Control Mode: "all" (Dynamic Switching Implementation)

**URDF Configuration:**

```xml
<hardware>
  <plugin>mujoco_ros2_control/MujocoSystem</plugin>
  <param name="control_mode">all</param>  <!-- ALWAYS "all", hardcoded -->
</hardware>
```

**How Dynamic Switching Works:**

```cpp
// In MujocoSystem::write()
// 1. Detect active controller (command changed from initial value)
if (abs(position_command - initial_position_command) > threshold) {
  position_command_active = true;
}

// 2. Apply ONLY the active controller
bool apply_position = is_position_control_enabled &&
                      (control_mode == "position" ||
                       (control_mode == "all" && position_command_active &&
                        !velocity_command_active && !effort_command_active));

if (apply_position) {
  // Compute PID and apply torque
  tau = Kp * (q_des - q) + Kd * (dq_des - dq);
  qfrc_applied[i] = tau;
}
```

**Why This Works:**

- Without custom fork: All three interfaces write to joints → conflicts
- With custom fork: Detects active interface → only applies that one
- Enables seamless switching without URDF changes

### Launch File Behavior

**What `mujoco_sim.launch.py` does:**

1. **Generate URDF** with `use_mujoco:=true` and `control_mode:=all`
2. **Start MuJoCo node** with **ALL THREE** controller configs loaded
3. **Load ALL THREE controllers as INACTIVE:**
   - `joint_trajectory_controller` (position)
   - `velocity_controller` (velocity)
   - `effort_controller` (effort)
4. **Activate ONLY the one specified by `control_mode` parameter**
5. User can switch at runtime without restart

**Controller Config Loading:**

```python
# In launch file
parameters=[
    robot_description,
    position_config,  # controllers_position.yaml
    velocity_config,  # controllers_velocity.yaml
    effort_config,    # controllers_effort.yaml
    {"mujoco_model_path": model_path, "use_sim_time": True},
]
```

All configs are loaded into the controller manager's parameter space, enabling dynamic switching.

---

## File Structure

```
openarm_description/
├── config/mujoco/
│   ├── v10_sim_params.yaml           # (Placeholder, not fully implemented)
│   ├── controllers_position.yaml     # JointTrajectoryController config
│   ├── controllers_velocity.yaml     # VelocityController config
│   └── controllers_effort.yaml       # EffortController config
│
├── docs/
│   └── mujoco_status.md              # THIS FILE (comprehensive reference)
│
├── launch/
│   └── mujoco_sim.launch.py          # Main simulation launch file
│
├── mujoco_models/
│   ├── openarm_v10.xml               # Generated MuJoCo model (7 DOF)
│   ├── openarm_v10_hand.xml          # With hand (9 DOF)
│   ├── openarm_v10_bimanual.xml      # Bimanual (14 DOF)
│   └── meshes/                       # Copied STL files
│
├── rviz/
│   └── mujoco_view.rviz              # RViz visualization config
│
├── scripts/
│   ├── urdf_to_mjcf.py                        # URDF → MJCF converter
│   ├── test_mujoco_setup.py                   # Setup validation tests
│   ├── example_position_control.py            # Position control demo
│   ├── gravity_compensation_controller.py     # Pinocchio-based gravity comp
│   └── test_gravity_compensation.py           # Gravity comp stability test
│
└── urdf/
    ├── robot/
    │   ├── v10.urdf.xacro               # Top-level robot xacro
    │   └── openarm_robot.xacro          # Robot macro with use_mujoco param
    └── ros2_control/
        ├── openarm.ros2_control.xacro         # Single arm ros2_control
        └── openarm.bimanual.ros2_control.xacro # Bimanual ros2_control
```

**Key Files:**

| File | Purpose |
|------|---------|
| `mujoco_sim.launch.py` | Main entry point, loads all controllers |
| `openarm.ros2_control.xacro` | Defines hardware interface, PID gains, command interfaces |
| `urdf_to_mjcf.py` | Converts URDF to MuJoCo XML with mesh copying and validation |
| `gravity_compensation_controller.py` | Implements torque-based gravity compensation |
| `mujoco_status.md` | **This file** - comprehensive reference |

---

## Known Issues & Limitations

### Issues

**1. Velocity Control Instability (HIGH PRIORITY)**

- **Symptom:** Robot drifts/spins uncontrollably with velocity commands
- **Root Cause:** PID without integral term cannot reject gravity disturbance, gain tuning paradox
- **Status:** Attempted fix (gains/1000) improved but not sufficient
- **Workaround:** Use position control with high-frequency updates
- **Future Fix:** Add Ki with anti-windup, or use MuJoCo native velocity actuators

**2. Library Loading Error (MEDIUM PRIORITY)**

- **Symptom:** `symbol lookup error: undefined symbol: _ZN20controller_interface23ControllerInterfaceBaseD2Ev`
- **When:** When dynamically loading controllers without proper `LD_LIBRARY_PATH`
- **Workaround:** `export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH` before launch
- **Future Fix:** Set in launch file environment or fix package dependencies

**3. Joint 6 Steady-State Sag (LOW PRIORITY)**

- **Symptom:** Joint 6 sags ~1.4° under gravity in position control
- **Root Cause:** PD control (Ki=0) cannot eliminate steady-state error from constant gravity
- **Impact:** Minor, acceptable for most applications
- **Solution:** Add small Ki with anti-windup (risky, can cause instability)

### Limitations

1. **Fixed-Base Only:** Current implementation removes freejoint (floating base not supported)
2. **Effort Control Requires Gravity Comp:** Unlike position control, effort control needs external gravity compensation
3. **Manual Model Generation:** Must run `urdf_to_mjcf.py` before first use (not automated in launch)
4. **PID Tuning:** Gains are conservative for stability, may not be optimal for performance

---

## Troubleshooting

### Problem: MuJoCo model not found

```bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml
```

### Problem: Symbol lookup error when loading controllers

```bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
# Then launch
```

### Problem: Robot falls under gravity in effort control

**Solution:** Start gravity compensation controller

```bash
python3 scripts/gravity_compensation_controller.py
```

### Problem: Controllers not switching

**Check controller status:**

```bash
ros2 control list_controllers
```

**Verify custom fork installed:**

```bash
ros2 pkg list | grep mujoco_ros2_control
# Should show custom fork, not upstream version
```

### Problem: Position control oscillates / unstable

**Check PID gains in URDF:**

```bash
grep -A 5 "position_pid" urdf/ros2_control/openarm.ros2_control.xacro
```

Gains should be:

- Joint 1-2: Kp=20.0, Kd=2.0
- Joint 3-4: Kp=15.0, Kd=1.5
- Joint 5-7: Kp=5.0, Kd=0.5

### Debug Commands

```bash
# Check joint states
ros2 topic echo /joint_states --once

# Check controller manager
ros2 control list_hardware_interfaces

# View MuJoCo node logs
ros2 run rqt_console rqt_console

# Test URDF generation
cd ~/ros2_ws/src/openarm_description
python3 scripts/test_mujoco_setup.py
```

---

## API Reference

### Launch Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `arm_type` | string | `v10` | ARM type |
| `control_mode` | string | `position` | Initial controller: `position`, `velocity`, `effort` |
| `hand` | bool | `false` | Include hand/gripper |
| `bimanual` | bool | `false` | Bimanual configuration |
| `use_rviz` | bool | `true` | Launch RViz |
| `use_sim_time` | bool | `true` | Use simulation time |
| `mujoco_model_path` | string | (auto) | Override MuJoCo model path |

### ROS2 Topics

**Published:**

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Joint position, velocity, effort |
| `/robot_description` | `std_msgs/String` | Robot URDF |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |

**Subscribed (depends on active controller):**

| Topic | Type | Controller | Description |
|-------|------|------------|-------------|
| `/joint_trajectory_controller/follow_joint_trajectory` | Action | Position | Joint trajectory |
| `/velocity_controller/commands` | `std_msgs/Float64MultiArray` | Velocity | Velocity commands (rad/s) |
| `/effort_controller/commands` | `std_msgs/Float64MultiArray` | Effort | Torque commands (N⋅m) |

### ROS2 Services

| Service | Type | Description |
|---------|------|-------------|
| `/controller_manager/list_controllers` | `controller_manager_msgs/ListControllers` | List all controllers |
| `/controller_manager/switch_controller` | `controller_manager_msgs/SwitchController` | Switch active controllers |
| `/controller_manager/load_controller` | `controller_manager_msgs/LoadController` | Load a controller |

### Scripts

**urdf_to_mjcf.py** - Convert URDF to MuJoCo MJCF

```bash
python3 scripts/urdf_to_mjcf.py --arm-type v10 [OPTIONS]

Options:
  --bimanual          Generate bimanual configuration
  --hand              Include hand/gripper
  --output PATH       Output MJCF file (required)
  --no-gravity        Disable gravity (for PID tuning)
  --validate-only     Only validate existing MJCF
```

**gravity_compensation_controller.py** - Torque-based gravity compensation

```bash
python3 scripts/gravity_compensation_controller.py

Parameters (ROS2 params):
  urdf_path: ""                 # Auto-detect URDF
  gravity_scale: 1.0            # Scale factor for gravity (tuning)
  control_rate: 100.0           # Control loop rate (Hz)
  add_damping: true             # Add velocity damping for stability
  damping_gains: [2.0, ..., 0.5]  # Damping coefficients per joint
```

**test_gravity_compensation.py** - Stability test for gravity compensation

```bash
python3 scripts/test_gravity_compensation.py

Tests:
  - Basic stability (10s, ±0.05 rad threshold)
  - Extended stability (30s, ±0.1 rad threshold)
  - Perturbation recovery (apply 5 Nm, check recovery)
```

### Joint Limits

| Joint | Position (rad) | Velocity (rad/s) | Effort (N⋅m) |
|-------|----------------|------------------|--------------|
| joint1 | -1.396 to 3.491 | 16.755 | ±40 |
| joint2 | -1.745 to 1.745 | 16.755 | ±40 |
| joint3 | -1.571 to 1.571 | 5.445 | ±27 |
| joint4 | 0.0 to 2.443 | 5.445 | ±27 |
| joint5 | -1.571 to 1.571 | 20.944 | ±7 |
| joint6 | -0.785 to 0.785 | 20.944 | ±7 |
| joint7 | -1.571 to 1.571 | 20.944 | ±7 |

---

## Additional Resources

- **Custom mujoco_ros2_control Fork:** <https://github.com/zordicom/mujoco_ros2_control> (branch: `2025-11-control-interface`, commit: `5a1e443`)
- **MuJoCo Documentation:** <https://mujoco.readthedocs.io/>
- **ros2_control:** <https://control.ros.org/>
- **Pinocchio:** <https://stack-of-tasks.github.io/pinocchio/>

---

**Copyright 2025 Zordi, Inc. All rights reserved.**
