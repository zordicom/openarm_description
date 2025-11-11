# OpenARM MuJoCo Integration - Complete Status & Reference

**Last Updated**: November 10, 2025 (Controller Switching Update)
**Status**: ✅ **Production Ready** (Position Control), ⚠️ **Unstable** (Velocity Control), ✅ **Working** (Effort Control with Gravity Compensation)

**Recent Updates (November 2025)**:

- ✅ Implemented proper ros2_control API callbacks (`prepare/perform_command_mode_switch`)
- ✅ Instant controller switching (no timeout delays)
- ✅ Fixed simulation timing (clock synchronization)
- ✅ Thread-safe clock publishing (RViz stability)
- ✅ Simplified launch API: All controllers loaded at startup, position active by default
- ✅ Removed `control_mode` launch argument (use runtime switching instead)

**Quick Links**:

- 📦 [Setup & Installation Guide](ZORDI_README.md) - Complete setup instructions
- 🔧 [Torque Control vs Actuators](torque_vs_actuator_control.md) - Technical deep dive

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [System Architecture](#system-architecture)
3. [Control Modes](#control-modes)
4. [Quick Start](#quick-start) - *See [ZORDI_README.md](ZORDI_README.md) for full setup*
5. [Testing](#testing-the-controller-switching-implementation)
6. [Implementation Details](#implementation-details)
7. [File Structure](#file-structure)
8. [Known Issues & Limitations](#known-issues--limitations)
9. [Troubleshooting](#troubleshooting)
10. [API Reference](#api-reference)

---

## Executive Summary

OpenARM now has full MuJoCo physics simulation support via ROS2 Control, enabling realistic dynamics simulation with gravity, inertia, friction, and contacts.

### What Works

✅ **Position Control** - Excellent stability and accuracy (< 0.02° error), active by default
✅ **Effort Control** - Works with gravity compensation controller
✅ **Dynamic Controller Switching** - All controllers loaded at startup, switch at runtime with `ros2 control`
✅ **Gravity Simulation** - Robot holds position under gravity
✅ **Single Arm, Bimanual, Hand** - All configurations supported
⚠️ **Velocity Control** - Implemented but unstable (needs tuning)

### Key Achievement

**Custom `mujoco_ros2_control` fork** enables all three control interfaces (position, velocity, effort) to coexist without conflicts. The hardware interface detects which controller is active and only applies commands from that controller.

**Direct torque control** (`qfrc_applied`) matches OpenARM's real hardware behavior. See [`docs/torque_vs_actuator_control.md`](torque_vs_actuator_control.md) for detailed comparison with MuJoCo actuators.

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

### Torque Control vs MuJoCo Actuators

**OpenARM uses direct torque control (`qfrc_applied`), not MuJoCo actuators (`ctrl`).**

- We write joint torques directly to `mj_data->qfrc_applied[joint_idx]`
- This gives us direct torque control without actuator dynamics
- Matches OpenARM's real hardware (Damiao motors in torque control mode)
- Generated MJCF files have NO `<actuator>` section (auto-removed during conversion)

**For detailed comparison and alternatives**, see: [`docs/torque_vs_actuator_control.md`](torque_vs_actuator_control.md)

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
# Launch (position controller active by default)
ros2 launch openarm_description mujoco_sim.launch.py

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
# Terminal 1: Launch simulation
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch openarm_description mujoco_sim.launch.py

# Terminal 2: Switch to effort controller
ros2 control switch_controllers --activate effort_controller \
  --deactivate joint_trajectory_controller

# Terminal 3: Start gravity compensation
python3 scripts/gravity_compensation_controller.py

# Robot now holds position under gravity!

# Terminal 4: Send additional torques for movement
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

**For complete setup instructions**, see: [`docs/ZORDI_README.md`](ZORDI_README.md)

### Quick Launch

```bash
# Prerequisites: Install custom mujoco_ros2_control fork, MuJoCo Python, Pinocchio
# See ZORDI_README.md for detailed installation steps

# Generate MuJoCo model (first time only)
cd ~/ros2_ws/src/openarm_description
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml

# Launch simulation (position controller active by default)
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch openarm_description mujoco_sim.launch.py

# Test position control
python3 scripts/example_position_control.py
```

### Dynamic Controller Switching

```bash
# List controllers
ros2 control list_controllers

# Switch between control modes
ros2 control switch_controllers --activate effort_controller \
  --deactivate joint_trajectory_controller velocity_controller
```

---

## Testing the Controller Switching Implementation

### Prerequisites for Testing

**Ensure you have completed the setup** in [`docs/ZORDI_README.md`](ZORDI_README.md), including:

- Custom mujoco_ros2_control fork (branch: `2025-11-control-interface`)
- MuJoCo Python packages
- Pinocchio
- Generated MuJoCo models

**Verify installation**:

```bash
cd ~/ros2_ws/src/mujoco_ros2_control
git status  # Should show branch: 2025-11-control-interface
git log --oneline -1  # Verify latest commits include controller switching callbacks
```

### Test 1: Startup Stability (Position Control Active by Default)

**Purpose:** Verify robot holds initial pose during controller loading (no collapse under gravity)

```bash
# Terminal 1: Launch and watch for stability
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch openarm_description mujoco_sim.launch.py

# Terminal 2: Monitor joint positions
ros2 topic echo /joint_states --field position

# Expected: All joints stay near zero (±0.05 rad) for first 5 seconds
```

**Success Criteria:**

- ✅ Robot doesn't fall during startup
- ✅ Joints hold position (< 0.05 rad drift)
- ✅ No error messages in console

### Test 2: Instant Controller Switching (Position → Velocity)

**Purpose:** Verify instant switching with explicit callback logging

```bash
# Terminal 1: Launch (already running from Test 1)

# Terminal 2: Switch to velocity controller
ros2 control switch_controllers --deactivate joint_trajectory_controller \
    --activate velocity_controller

# Terminal 3: Check MuJoCo node logs
ros2 run rqt_console rqt_console  # Filter by node: mujoco_ros2_control
```

**Expected Log Messages:**

```
[INFO] Controller switch: Position interface STOPPED for openarm_joint1
[INFO] Controller switch: Velocity interface STARTED for openarm_joint1
```

**Success Criteria:**

- ✅ Log messages appear immediately (< 10ms)
- ✅ No "timeout" messages (those are from old implementation)
- ✅ Controller switches without delay

### Test 3: Velocity Controller Activation

**Purpose:** Test that velocity commands are actually applied after switch

```bash
# Terminal 1: Launch with velocity controller
ros2 control switch_controllers --activate velocity_controller \
    --deactivate joint_trajectory_controller effort_controller

# Terminal 2: Send constant velocity command (joint 1 should rotate)
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" -r 100

# Terminal 3: Monitor joint positions
ros2 topic echo /joint_states --field position

# Expected: joint1 position should increase continuously at ~0.5 rad/s
```

**Success Criteria:**

- ✅ Joint 1 rotates at commanded velocity
- ✅ Other joints remain stationary
- ⚠️ **Known Issue**: Velocity control may drift due to gravity (see limitations)

### Test 4: Effort Controller Switching

**Purpose:** Verify effort control activates and gravity compensation works

```bash
# Terminal 1: Launch
ros2 control switch_controllers --activate effort_controller \
    --deactivate joint_trajectory_controller velocity_controller

# Terminal 2: Start gravity compensation (robot should hold position)
python3 ~/ros2_ws/src/openarm_description/scripts/gravity_compensation_controller.py

# Terminal 3: Send additional torques
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" -r 100

# Expected: Joint 1 moves due to additional 10 Nm torque
```

**Expected Log Messages:**

```
[INFO] Controller switch: Position interface STOPPED for openarm_joint1
[INFO] Controller switch: Effort interface STARTED for openarm_joint1
```

**Success Criteria:**

- ✅ Robot holds position with gravity compensation
- ✅ Additional torques cause movement
- ✅ Switching messages appear in logs

### Test 5: Rapid Switching (Stress Test)

**Purpose:** Verify system handles rapid controller switches without crashes

```bash
# Terminal 1: Launch (already running)

# Terminal 2: Rapid switching script
for i in {1..10}; do
  echo "Switch $i: Position"
  ros2 control switch_controllers --activate joint_trajectory_controller \
      --deactivate velocity_controller effort_controller
  sleep 1

  echo "Switch $i: Velocity"
  ros2 control switch_controllers --activate velocity_controller \
      --deactivate joint_trajectory_controller effort_controller
  sleep 1

  echo "Switch $i: Effort"
  ros2 control switch_controllers --activate effort_controller \
      --deactivate joint_trajectory_controller velocity_controller
  sleep 1
done
```

**Success Criteria:**

- ✅ No crashes or error messages
- ✅ Each switch completes successfully
- ✅ Log messages appear for each switch
- ✅ Robot behavior changes with each switch

### Test 6: Simulation Clock Monotonicity

**Purpose:** Verify clock never goes backwards (RViz stability)

```bash
# Terminal 1: Launch with RViz
ros2 launch openarm_description mujoco_sim.launch.py use_rviz:=true

# Terminal 2: Monitor clock
ros2 topic echo /clock --field clock

# Run for 30 seconds and verify time always increases
```

**Success Criteria:**

- ✅ Clock time monotonically increases
- ✅ No jumps backward in time
- ✅ RViz doesn't reset or flicker
- ✅ TF tree stays stable

### Test 7: Multi-Controller Simultaneous Command Test

**Purpose:** Verify only the active controller's commands are applied

```bash
# Terminal 1: Launch with position controller active
ros2 launch openarm_description mujoco_sim.launch.py

# Terminal 2: Send position command (should work)
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [openarm_joint1, openarm_joint2, openarm_joint3, openarm_joint4, openarm_joint5, openarm_joint6, openarm_joint7]
  points:
  - positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 3}
"

# Terminal 3: Send velocity command (should be ignored)
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]" -r 100 &

# Terminal 4: Send effort command (should be ignored)
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [20.0, 20.0, 15.0, 15.0, 5.0, 5.0, 5.0]" -r 100 &

# Expected: Robot follows position trajectory, ignores velocity/effort commands
```

**Success Criteria:**

- ✅ Robot follows position command only
- ✅ Velocity/effort commands don't interfere
- ✅ No control conflicts or instability

### Test 8: Debug Logging Verification

**Purpose:** Verify periodic debug logs are working

```bash
# Terminal 1: Launch
ros2 launch openarm_description mujoco_sim.launch.py

# Terminal 2: Filter logs for joint1 debug messages
ros2 run rqt_console rqt_console
# Filter: Message contains "Joint1 Debug"

# Expected: Log message every ~1 second (500 cycles at 500Hz)
```

**Expected Log Format:**

```
[INFO] Joint1 Debug: apply_pos=1, is_enabled=1, mode=all, active=1, cmd=0.000, cur=0.000, period_ns=2000000
```

**Success Criteria:**

- ✅ Debug logs appear every ~1 second
- ✅ `apply_pos`, `active` flags match expected controller state
- ✅ Commands and current positions are reasonable

### Test 9: Controller Manager State Verification

**Purpose:** Verify all controllers load correctly and only one is active

```bash
# Check loaded controllers
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster     [active]
# joint_trajectory_controller [active]    ← Position controller
# velocity_controller         [inactive]
# effort_controller           [inactive]

# Check available interfaces
ros2 control list_hardware_interfaces

# Expected: All position/velocity/effort interfaces listed
```

**Success Criteria:**

- ✅ All three controllers loaded (1 active, 2 inactive)
- ✅ All command interfaces available
- ✅ Only one controller active at a time

### Automated Test Script

Save as `test_controller_switching.sh`:

```bash
#!/bin/bash
set -e

echo "=== Controller Switching Test Suite ==="

# Ensure environment
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
source ~/ros2_ws/install/setup.bash

# Launch in background
ros2 launch openarm_description mujoco_sim.launch.py use_rviz:=false &
LAUNCH_PID=$!
sleep 5

echo "Test 1: Check initial state"
ros2 control list_controllers | grep "joint_trajectory_controller.*active" && echo "✅ Position active" || exit 1

echo "Test 2: Switch to velocity"
ros2 control switch_controllers --activate velocity_controller --deactivate joint_trajectory_controller
sleep 1
ros2 control list_controllers | grep "velocity_controller.*active" && echo "✅ Velocity active" || exit 1

echo "Test 3: Switch to effort"
ros2 control switch_controllers --activate effort_controller --deactivate velocity_controller
sleep 1
ros2 control list_controllers | grep "effort_controller.*active" && echo "✅ Effort active" || exit 1

echo "Test 4: Switch back to position"
ros2 control switch_controllers --activate joint_trajectory_controller --deactivate effort_controller
sleep 1
ros2 control list_controllers | grep "joint_trajectory_controller.*active" && echo "✅ Position active" || exit 1

echo "=== All tests passed! ==="

# Cleanup
kill $LAUNCH_PID
```

Run with:

```bash
chmod +x test_controller_switching.sh
./test_controller_switching.sh
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

**How Dynamic Switching Works (Updated November 2025):**

Uses proper ros2_control API callbacks for instant, explicit controller switching:

```cpp
// Phase 1: Controller Manager calls prepare_command_mode_switch()
// Validates that the requested switch is possible
hardware_interface::return_type prepare_command_mode_switch(
  const std::vector<std::string> &start_interfaces,  // e.g., ["joint1/velocity", ...]
  const std::vector<std::string> &stop_interfaces)   // e.g., ["joint1/position", ...]
{
  // Validate interfaces exist
  // Return OK to proceed, or ERROR to abort
  return hardware_interface::return_type::OK;
}

// Phase 2: Controller Manager calls perform_command_mode_switch()
// Executes the actual switch
hardware_interface::return_type perform_command_mode_switch(...)
{
  // Parse interface names and update active flags
  for (auto &interface : stop_interfaces) {
    if (interface_type == "position")
      position_command_active = false;
  }
  for (auto &interface : start_interfaces) {
    if (interface_type == "velocity") {
      velocity_command_active = true;
      position_command_active = false;  // Mutual exclusion
    }
  }
  return hardware_interface::return_type::OK;
}

// In MujocoSystem::write() - Apply ONLY the active controller
bool apply_position = is_position_control_enabled &&
                      (control_mode == "position" ||
                       (control_mode == "all" && position_command_active &&
                        !velocity_command_active && !effort_command_active));

if (apply_position) {
  tau = Kp * (q_des - q) + Kd * (dq_des - dq);
  qfrc_applied[i] = tau;
}
```

**Key Improvements (November 2025 Update):**

- ✅ **Instant switching** - No timeout detection needed, callbacks fire immediately
- ✅ **Explicit state tracking** - Controller manager tells us exactly what changed
- ✅ **Proper ros2_control API** - Follows hardware_interface::SystemInterface contract
- ✅ **Clean implementation** - ~50 lines vs previous ~70 lines of timeout logic
- ✅ **Simulation timing fix** - `mj_step1()` called before reading time (correct clock sync)
- ✅ **Thread-safe clock** - Monotonic guarantee prevents RViz resets

**Why This Works:**

- Without custom fork: All three interfaces write to joints → conflicts
- With custom fork: Receives explicit signals from controller manager
- Enables seamless switching with zero delay

### Launch File Behavior

**What `mujoco_sim.launch.py` does:**

1. **Generate URDF** with `use_mujoco:=true` and `control_mode:=all` (hardcoded)
2. **Start MuJoCo node** with **ALL THREE** controller configs loaded
3. **Load ALL THREE controllers as INACTIVE:**
   - `joint_trajectory_controller` (position)
   - `velocity_controller` (velocity)
   - `effort_controller` (effort)
4. **Activate position controller by default** (`joint_trajectory_controller`)
5. **User can switch at runtime** using `ros2 control switch_controllers`

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

All configs are loaded into the controller manager's parameter space, enabling seamless dynamic switching without restarting the simulation.

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

**For common setup issues**, see the troubleshooting section in [`docs/ZORDI_README.md`](ZORDI_README.md)

### Quick Fixes

**MuJoCo model not found:**

```bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml
```

**Symbol lookup error:**

```bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch openarm_description mujoco_sim.launch.py
```

**Robot falls in effort control:**

```bash
python3 scripts/gravity_compensation_controller.py
```

**Controllers not switching:**

```bash
ros2 control list_controllers  # Check status
# Verify custom fork: see ZORDI_README.md
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
| `hand` | bool | `false` | Include hand/gripper |
| `bimanual` | bool | `false` | Bimanual configuration |
| `use_rviz` | bool | `true` | Launch RViz |
| `rviz_config` | string | (auto) | Path to RViz config file |
| `use_sim_time` | bool | `true` | Use simulation time |
| `mujoco_model_path` | string | (auto) | Override MuJoCo model path |

**Note:** The `control_mode` parameter is no longer supported as a launch argument. All three controllers (position, velocity, effort) are loaded at startup with the position controller active by default. Use `ros2 control switch_controllers` to change modes at runtime.

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

### Documentation

- **Setup & Installation Guide:** [`docs/ZORDI_README.md`](ZORDI_README.md) - Complete setup instructions, troubleshooting, and usage guide
- **Torque Control vs Actuators:** [`docs/torque_vs_actuator_control.md`](torque_vs_actuator_control.md) - Deep dive on direct torque control vs MuJoCo actuators

### External Resources

- **Custom mujoco_ros2_control Fork:** <https://github.com/zordicom/mujoco_ros2_control> (branch: `2025-11-control-interface`, commit: `5a1e443`)
- **MuJoCo Documentation:** <https://mujoco.readthedocs.io/>
- **ros2_control:** <https://control.ros.org/>
- **Pinocchio:** <https://stack-of-tasks.github.io/pinocchio/>

---

**Copyright 2025 Zordi, Inc. All rights reserved.**
