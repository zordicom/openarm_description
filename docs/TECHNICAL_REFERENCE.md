# Technical Reference: Actuator-Centric Control System

**Last Updated**: November 13, 2025
**System**: OpenARM with MuJoCo + ROS2 Control
**Status**: Production Ready

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Control Modes](#control-modes)
3. [MuJoCo Actuator Configuration](#mujoco-actuator-configuration)
4. [ROS2 Controllers](#ros2-controllers)
5. [Hardware Interface Implementation](#hardware-interface-implementation)
6. [Code Structure](#code-structure)
7. [Configuration Reference](#configuration-reference)

---

## System Architecture

### Overview

The actuator-centric control system uses a **three-layer architecture**:

```
┌──────────────────────────────────────────────────────────┐
│                  Application Layer                        │
│  • ROS2 Controllers (zordi_mit_controller, JTC, etc.)   │
│  • Computes desired states: q_cmd, qd_cmd, τ_ff        │
└────────────────────┬─────────────────────────────────────┘
                     │
                     ▼
┌──────────────────────────────────────────────────────────┐
│              Hardware Interface Layer                     │
│           (mujoco_system.cpp)                            │
│  • Detects active command interfaces                     │
│  • Applies control mode logic                            │
│  • Composes MIT mode PD: τ = Kp*e_p + Kd*e_v + τ_ff    │
└────────────────────┬─────────────────────────────────────┘
                     │
                     ▼
┌──────────────────────────────────────────────────────────┐
│                  Physics Layer                            │
│              (MuJoCo Engine)                             │
│  • Three actuators per joint: position, velocity, torque │
│  • Simulates robot dynamics                              │
│  • Returns: q, qd, τ                                     │
└──────────────────────────────────────────────────────────┘
```

### Key Principles

1. **Actuator-Centric Logic**
   - Drive actuators based on active command interfaces
   - Neutralize inactive actuators (set to current state)
   - No global control mode state

2. **Mode Detection**
   - Automatic based on which interfaces are active
   - Per-joint, per-cycle decision
   - Flags: `position_command_active`, `velocity_command_active`, `effort_command_active`

3. **Separation of Concerns**
   - **Controllers**: Compute desired states (what to do)
   - **Hardware Interface**: Execute control strategy (how to do it)
   - **Physics Engine**: Simulate dynamics (ground truth)

---

## Control Modes

### Mode Matrix

| Mode | Pos | Vel | Eff | Active Actuators | Use Case |
|------|-----|-----|-----|-----------------|----------|
| 1 | ✓ | - | - | Position only | Joint positioning |
| 2 | - | ✓ | - | Velocity only | Velocity tracking |
| 3 | - | - | ✓ | Torque only | Force control |
| 4 | ✓ | ✓ | - | Position + Velocity | Trajectory tracking |
| 5 | ✓ | - | ✓ | MIT Mode (P control) | Position with feedforward |
| 6 | - | ✓ | ✓ | MIT Mode (D control) | Velocity with feedforward |
| 7 | ✓ | ✓ | ✓ | MIT Mode (PD control) | Full state control |

### Control Equations

#### Mode 1: Position-Only

```
Position actuator: ctrl_pos = q_cmd
Velocity actuator: ctrl_vel = qd (neutralized)
Torque actuator: ctrl_tau = 0 (neutralized)
```

#### Mode 2: Velocity-Only

```
Position actuator: ctrl_pos = q (neutralized)
Velocity actuator: ctrl_vel = qd_cmd
Torque actuator: ctrl_tau = 0 (neutralized)
```

#### Mode 3: Torque-Only

```
Position actuator: ctrl_pos = q (neutralized)
Velocity actuator: ctrl_vel = qd (neutralized)
Torque actuator: ctrl_tau = τ_cmd
```

#### Mode 4: Position + Velocity

```
Position actuator: ctrl_pos = q_cmd
Velocity actuator: ctrl_vel = qd_cmd
Torque actuator: ctrl_tau = 0 (neutralized)
```

#### Mode 5-7: MIT Mode (Position and/or Velocity + Effort)

**Core equation**:

```
τ = Kp*(q_cmd - q) + Kd*(qd_cmd - qd) + τ_ff
```

Where:

- `Kp`, `Kd`: PID gains from URDF
- `q_cmd`, `qd_cmd`: Commanded position/velocity from controller
- `q`, `qd`: Current position/velocity from sensors
- `τ_ff`: Feedforward torque from controller (e.g., gravity compensation)

**Actuator behavior**:

```
Position actuator: ctrl_pos = q (neutralized)
Velocity actuator: ctrl_vel = qd (neutralized)
Torque actuator: ctrl_tau = τ (computed via PD composition)
```

**Variants**:

- **Mode 5** (P only): If `qd_cmd` not active, `Kd*e_v` term is zero
- **Mode 6** (D only): If `q_cmd` not active, `Kp*e_p` term is zero
- **Mode 7** (PD): Both P and D terms active

---

## MuJoCo Actuator Configuration

### Actuator Naming Convention

Each joint has **three actuators** with specific naming:

```xml
<!-- Position actuator -->
<position name="act_pos_{joint_name}" joint="{joint_name}"
          kp="..." kv="..." ctrlrange="..." forcerange="..."/>

<!-- Velocity actuator -->
<velocity name="act_vel_{joint_name}" joint="{joint_name}"
          kv="..." ctrlrange="..." forcerange="..."/>

<!-- Torque actuator -->
<motor name="act_tau_{joint_name}" joint="{joint_name}"
       ctrlrange="..." forcerange="..."/>
```

**Example** (OpenARM joint 1):

```xml
<actuator>
  <position name="act_pos_openarm_joint1" joint="openarm_joint1"
            kp="20.0" kv="0.0"
            ctrlrange="-3.14 3.14" forcerange="-87 87"/>

  <velocity name="act_vel_openarm_joint1" joint="openarm_joint1"
            kv="2.0"
            ctrlrange="-2.0 2.0" forcerange="-87 87"/>

  <motor name="act_tau_openarm_joint1" joint="openarm_joint1"
         ctrlrange="-87 87" forcerange="-87 87"/>
</actuator>
```

### Actuator Types

#### 1. Position Actuator

- **MuJoCo type**: `<position>`
- **Control law**: `τ = kp*(ctrl - q) - kv*qd`
- **When used**: Direct position control (Mode 1, 4)
- **When neutralized**: Set `ctrl = q` (generates zero torque **only if kv=0**)

**Parameters**:

- `kp`: Position gain (stiffness)
- `kv`: Damping gain (**MUST be 0.0 for MIT mode**)
- `ctrlrange`: Control signal limits (position range)
- `forcerange`: Torque limits

**⚠️ IMPORTANT FOR MIT MODE**: Position actuators **must have `kv=0.0`** when used with MIT mode (effort + position/velocity control).

**When is non-zero kv OK?**

- ✅ When using **pure position control** (Mode 1, 4): `kv` provides desirable damping
- ✅ Control law: `τ = kp*(pos_cmd - q) - kv*qd` (kv term is intentional)

**When is non-zero kv problematic?**

- ❌ When position actuator is **neutralized** (not actively used):
  - Startup (no controller active yet)
  - MIT mode (torque actuator in use, position actuator neutralized)
- ❌ Control law becomes: `τ = kp*(q - q) - kv*qd = -kv*qd`
- ❌ This produces unwanted velocity damping that interferes with torque control

The system will warn if `kv ≠ 0` when position and effort interfaces are both exposed (MIT mode configuration).

#### 2. Velocity Actuator

- **MuJoCo type**: `<velocity>`
- **Control law**: `τ = -kv*(qd - ctrl)`
- **When used**: Direct velocity control (Mode 2, 4)
- **When neutralized**: Set `ctrl = qd` (generates zero torque)

**Parameters**:

- `kv`: Velocity gain
- `ctrlrange`: Control signal limits (velocity range)
- `forcerange`: Torque limits

#### 3. Torque Actuator

- **MuJoCo type**: `<motor>`
- **Control law**: `τ = ctrl` (direct torque)
- **When used**: Torque control (Mode 3), MIT mode (Mode 5-7)
- **When neutralized**: Set `ctrl = 0`

**Parameters**:

- `ctrlrange`: Torque signal limits
- `forcerange`: Torque limits (usually same as ctrlrange)

### Actuator Interaction

When multiple actuators act on the same joint, their torques **sum**:

```
τ_total = τ_position + τ_velocity + τ_motor
```

**Neutralization Strategy**:

- Inactive actuators set to generate `τ = 0`
- Prevents interference with active actuators
- Enables seamless mode switching

---

## ROS2 Controllers

### zordi_mit_controller

**Type**: `zordi_mit_controller/ZordiMITController`
**Purpose**: Full state control with gravity compensation

#### Configuration

```yaml
zordi_mit_controller:
  ros__parameters:
    joints:
      - openarm_joint1
      - openarm_joint2
      - openarm_joint3
      - openarm_joint4
      - openarm_joint5
      - openarm_joint6

    command_interfaces:
      - position
      - velocity
      - effort

    state_interfaces:
      - position
      - velocity

    use_gravity_compensation: true

    action_monitor_rate: 20.0  # Hz
```

#### Features

- **Gravity Compensation**: Via Pinocchio `computeGeneralizedGravity()`
- **Trajectory Interface**: Action and topic subscribers
- **Hold Mode**: Applies gravity comp even when idle
- **MIT Mode**: Automatically triggered when all three interfaces claimed

#### Interfaces

- **Action**: `/zordi_mit_controller/follow_joint_trajectory`
- **Topic**: `/zordi_mit_controller/joint_trajectory`

#### Implementation Details

**Gravity Compensation**:

```cpp
std::vector<double> ZordiMITController::compute_effort_feedforward(
    const std::vector<double>& positions,
    const std::vector<double>& velocities)
{
  // Update Pinocchio model
  Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
    positions.data(), positions.size());

  pinocchio::forwardKinematics(model_, data_, q);

  // Compute gravity torques
  Eigen::VectorXd tau_g = pinocchio::computeGeneralizedGravity(
    model_, data_);

  // Convert to std::vector
  std::vector<double> gravity_torques(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    gravity_torques[i] = tau_g[i];
  }

  return gravity_torques;
}
```

**Hold Mode**:

```cpp
controller_interface::return_type ZordiMITController::update(...) {
  if (!trajectory_active_) {
    // No trajectory - hold with gravity compensation
    std::vector<double> current_pos = get_current_positions();
    std::vector<double> current_vel = get_current_velocities();
    std::vector<double> tau_gravity = compute_effort_feedforward(
      current_pos, current_vel);

    // Send commands to hold position against gravity
    for (size_t i = 0; i < num_joints; ++i) {
      position_cmd[i] = current_pos[i];  // Hold position
      velocity_cmd[i] = 0.0;             // Zero velocity
      effort_cmd[i] = tau_gravity[i];    // Gravity compensation
    }
  }
  // ... trajectory execution logic ...
}
```

---

### joint_trajectory_controller

**Type**: `joint_trajectory_controller/JointTrajectoryController`
**Purpose**: Standard ROS2 trajectory execution

#### Configuration

```yaml
joint_trajectory_controller:
  ros__parameters:
    joints:
      - openarm_joint1
      # ... etc

    command_interfaces:
      - position
      - velocity

    state_interfaces:
      - position
      - velocity
```

#### Features

- **Standard ROS2 controller**
- **Claims two interfaces**: Position + Velocity
- **No MIT mode**: Does not claim effort interface
- **Well-tested**: Part of ROS2 Control ecosystem

---

### effort_controller

**Type**: `effort_controllers/JointGroupEffortController`
**Purpose**: Direct torque control

#### Configuration

```yaml
effort_controller:
  ros__parameters:
    joints:
      - openarm_joint1
      # ... etc
```

#### Features

- **Single interface**: Claims only `effort`
- **Direct control**: Forwards commands to torque actuator
- **Low-level**: For impedance/force control

---

## Hardware Interface Implementation

### File: `mujoco_system.cpp`

#### Key Functions

##### 1. `init()` - Initialize Hardware Interface

```cpp
hardware_interface::CallbackReturn MujocoSystem::on_init(...) {
  // For each joint:
  // 1. Register state interfaces (position, velocity, effort)
  // 2. Register command interfaces (position, velocity, effort)
  // 3. Resolve actuator IDs from MuJoCo model
  // 4. Load PID gains from URDF
  // 5. Load joint limits
  // 6. Validate kv=0 for MIT mode configurations (warns if kv≠0 with effort+position)
}
```

**Actuator ID Resolution**:

```cpp
std::string pos_name = "act_pos_" + joint.name;
std::string vel_name = "act_vel_" + joint.name;
std::string tau_name = "act_tau_" + joint.name;

joint_state.mj_pos_actuator_id = mj_name2id(
  mj_model_, mjOBJ_ACTUATOR, pos_name.c_str());
joint_state.mj_vel_actuator_id = mj_name2id(
  mj_model_, mjOBJ_ACTUATOR, vel_name.c_str());
joint_state.mj_tau_actuator_id = mj_name2id(
  mj_model_, mjOBJ_ACTUATOR, tau_name.c_str());
```

**PID Gain Loading**:

```cpp
// IMPORTANT: Use underscores, not dots!
const std::string PARAM_KP[] = {"_kp"};
const std::string PARAM_KI[] = {"_ki"};
const std::string PARAM_KD[] = {"_kd"};

// Load position PID
hardware_interface::ComponentInfo::get_parameter(
  joint.parameters, "position" + PARAM_KP[0],
  joint_state.position_pid.p_gain_);

// Load velocity PID (Kd is stored in D gain)
hardware_interface::ComponentInfo::get_parameter(
  joint.parameters, "velocity" + PARAM_KD[0],
  joint_state.velocity_pid.d_gain_);
```

##### 2. `prepare_command_mode_switch()` - Pre-Switch Validation

```cpp
hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {

  // For each joint, track which interfaces will be active after switch
  // Return OK if valid, ERROR if conflicting

  return hardware_interface::return_type::OK;
}
```

##### 3. `perform_command_mode_switch()` - Update Active Flags

```cpp
hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {

  // Update per-joint flags
  for (auto& joint_state : joint_states_) {
    joint_state.position_command_active = /* check if in start_interfaces */;
    joint_state.velocity_command_active = /* check if in start_interfaces */;
    joint_state.effort_command_active = /* check if in start_interfaces */;
  }

  return hardware_interface::return_type::OK;
}
```

##### 4. `read()` - Read State from MuJoCo

```cpp
hardware_interface::return_type MujocoSystem::read(...) {
  // For each joint:
  // 1. Read position from mj_data_->qpos
  // 2. Read velocity from mj_data_->qvel
  // 3. Read effort from mj_data_->qfrc_actuator
  // 4. Update state interface values

  return hardware_interface::return_type::OK;
}
```

##### 5. `write()` - Apply Control Commands

**Core logic** (per joint):

```cpp
hardware_interface::return_type MujocoSystem::write(...) {
  for (auto& joint_state : joint_states_) {
    double q = mj_data_->qpos[joint_state.mj_pos_adr];
    double qd = mj_data_->qvel[joint_state.mj_vel_adr];

    // 1. Position actuator
    if (joint_state.mj_pos_actuator_id >= 0) {
      if (joint_state.position_command_active) {
        // Drive position
        double pos_cmd = clamp(joint_state.position_command,
                               limits.min, limits.max);
        mj_data_->ctrl[joint_state.mj_pos_actuator_id] = pos_cmd;
      } else {
        // Neutralize (warns once if kv≠0 during neutralization)
        mj_data_->ctrl[joint_state.mj_pos_actuator_id] = q;
      }
    }

    // 2. Velocity actuator
    if (joint_state.mj_vel_actuator_id >= 0) {
      if (joint_state.velocity_command_active) {
        // Drive velocity
        mj_data_->ctrl[joint_state.mj_vel_actuator_id] =
          joint_state.velocity_command;
      } else {
        // Neutralize
        mj_data_->ctrl[joint_state.mj_vel_actuator_id] = qd;
      }
    }

    // 3. Torque actuator (MIT mode if effort + pos/vel active)
    if (joint_state.mj_tau_actuator_id >= 0) {
      if (joint_state.effort_command_active) {
        double tau_total = joint_state.effort_command;

        // MIT mode: Add PD terms if position/velocity also active
        if (joint_state.position_command_active ||
            joint_state.velocity_command_active) {

          double tau_pd = 0.0;

          // P term
          if (joint_state.position_command_active) {
            double e_pos = joint_state.position_command - q;
            tau_pd += joint_state.position_pid.p_gain_ * e_pos;
          }

          // D term
          if (joint_state.velocity_command_active) {
            double e_vel = joint_state.velocity_command - qd;
            tau_pd += joint_state.velocity_pid.d_gain_ * e_vel;
          }

          tau_total += tau_pd;
        }

        // Apply with limits
        double limit = joint_state.joint_limits.max_effort;
        mj_data_->ctrl[joint_state.mj_tau_actuator_id] =
          clamp(tau_total, -limit, limit);
      } else {
        // Neutralize
        mj_data_->ctrl[joint_state.mj_tau_actuator_id] = 0.0;
      }
    }
  }

  return hardware_interface::return_type::OK;
}
```

---

## Code Structure

### Directory Layout

```
mujoco_ros2_control/
├── include/mujoco_ros2_control/
│   ├── mujoco_system.hpp          # Hardware interface header
│   └── mujoco_ros2_control.hpp    # Main node header
├── src/
│   ├── mujoco_system.cpp          # Hardware interface implementation
│   ├── mujoco_ros2_control.cpp    # Main node implementation
│   └── mujoco_ros2_control_node.cpp  # Node entry point
└── ...

zordi_mit_controller/
├── include/zordi_mit_controller/
│   └── zordi_mit_controller.hpp   # Controller header
├── src/
│   └── zordi_mit_controller.cpp   # Controller implementation
└── ...

openarm_description/
├── urdf/
│   └── openarm_v10.urdf           # Robot URDF (with PID gains)
├── mujoco_models/
│   └── openarm_v10.xml            # MuJoCo model (actuators)
├── launch/
│   └── single_arm.launch.py       # Main launch file
└── docs/
    ├── README.md                  # This quick start guide
    ├── PROJECT_HISTORY.md         # Development history
    └── TECHNICAL_REFERENCE.md     # This document
```

### Key Data Structures

#### JointState (mujoco_system.hpp)

```cpp
struct JointState {
  // Joint identification
  std::string name;
  int mj_joint_id;
  int mj_pos_adr;
  int mj_vel_adr;

  // Actuator IDs
  int mj_pos_actuator_id;
  int mj_vel_actuator_id;
  int mj_tau_actuator_id;

  // Command values
  double position_command;
  double velocity_command;
  double effort_command;

  // State values
  double position;
  double velocity;
  double effort;

  // Active flags
  bool position_command_active;
  bool velocity_command_active;
  bool effort_command_active;

  // PID gains
  control_toolbox::Pid position_pid;
  control_toolbox::Pid velocity_pid;
  bool is_pid_enabled;

  // Joint limits
  JointLimits joint_limits;
};
```

---

## Configuration Reference

### URDF Joint Configuration

```xml
<ros2_control name="MujocoSystem" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystem</plugin>
  </hardware>

  <joint name="openarm_joint1">
    <!-- Command interfaces (what controller can write) -->
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>

    <!-- State interfaces (what controller can read) -->
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>

    <!-- MIT mode PID gains (IMPORTANT: use underscores!) -->
    <param name="position_kp">20.0</param>
    <param name="position_ki">0.0</param>
    <param name="position_kd">0.0</param>

    <param name="velocity_kp">0.0</param>
    <param name="velocity_ki">0.0</param>
    <param name="velocity_kd">2.0</param>
  </joint>

  <!-- Repeat for other joints -->
</ros2_control>
```

### MuJoCo Model Configuration

```xml
<mujoco model="openarm">
  <compiler angle="radian" meshdir="." autolimits="true"/>
  <option timestep="0.001" gravity="0 0 -9.81"/>

  <worldbody>
    <body name="base_link">
      <!-- ... robot definition ... -->
    </body>
  </worldbody>

  <actuator>
    <!-- For each joint, three actuators -->
    <position name="act_pos_openarm_joint1" joint="openarm_joint1"
              kp="20.0" kv="0.0"
              ctrlrange="-3.14 3.14" forcerange="-87 87"/>

    <velocity name="act_vel_openarm_joint1" joint="openarm_joint1"
              kv="2.0"
              ctrlrange="-2.0 2.0" forcerange="-87 87"/>

    <motor name="act_tau_openarm_joint1" joint="openarm_joint1"
           ctrlrange="-87 87" forcerange="-87 87"/>

    <!-- Repeat for other joints -->
  </actuator>
</mujoco>
```

### Controller Configuration (YAML)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

zordi_mit_controller:
  ros__parameters:
    joints:
      - openarm_joint1
      - openarm_joint2
      - openarm_joint3
      - openarm_joint4
      - openarm_joint5
      - openarm_joint6

    command_interfaces:
      - position
      - velocity
      - effort

    state_interfaces:
      - position
      - velocity

    use_gravity_compensation: true
    action_monitor_rate: 20.0

joint_state_broadcaster:
  ros__parameters:
    joints:
      - openarm_joint1
      - openarm_joint2
      - openarm_joint3
      - openarm_joint4
      - openarm_joint5
      - openarm_joint6
```

---

## Performance Tuning

### PID Gain Selection

**General Guidelines**:

| System | Kp | Kd | Reasoning |
|--------|----|----|-----------|
| 1-DOF test | 100 | 10 | High gains for validation |
| 6-DOF (unloaded) | 20 | 2 | Lower for stability |
| 6-DOF (loaded) | 30-50 | 3-5 | Higher for tracking |

**Tuning Process**:

1. Start with low gains (Kp=10, Kd=1)
2. Increase Kp until oscillations appear
3. Reduce Kp to 50% of oscillation threshold
4. Increase Kd to add damping
5. Test with gravity compensation enabled

**Signs of Bad Tuning**:

- ❌ Oscillations → Kp too high or Kd too low
- ❌ Sluggish response → Kp too low
- ❌ Overshoot → Kd too low
- ❌ Vibration at rest → Kd too high

### Timestep Selection

**Current**: 0.001s (1kHz)

**Considerations**:

- Smaller timestep = Better accuracy, slower simulation
- Larger timestep = Faster simulation, risk of instability
- Rule of thumb: Timestep < 1/(10 * max_natural_frequency)

---

## Troubleshooting Guide

### Problem: PID gains are 0.0

**Symptoms**: Robot doesn't track, logs show gains = 0.0

**Cause**: URDF uses dots instead of underscores

**Solution**: Check URDF parameter naming:

```xml
<!-- WRONG -->
<param name="position.kp">20.0</param>

<!-- CORRECT -->
<param name="position_kp">20.0</param>
```

### Problem: Actuator not found

**Symptoms**: Error "Actuator 'act_pos_openarm_joint1' not found"

**Cause**: Mismatch between URDF joint names and MuJoCo actuator names

**Solution**: Ensure actuator naming follows convention:

- Position: `act_pos_{joint_name}`
- Velocity: `act_vel_{joint_name}`
- Torque: `act_tau_{joint_name}`

### Problem: MIT mode not activating

**Symptoms**: Only position control works, no PD composition

**Cause**: Controller not claiming all three interfaces

**Solution**: Check controller YAML:

```yaml
command_interfaces:
  - position
  - velocity
  - effort  # ← This must be present!
```

### Problem: Robot drifts under gravity

**Symptoms**: Robot slowly falls after trajectory

**Cause**: No gravity compensation or not applied in hold mode

**Solution**:

1. Enable gravity comp in YAML: `use_gravity_compensation: true`
2. Ensure controller computes gravity in hold mode (see zordi_mit_controller implementation)

### Problem: Unwanted damping in MIT mode

**Symptoms**:

- Robot exhibits unexpected velocity damping when using effort-only control
- Warning message: "Position actuator has kv=X.X but position interface is not active"

**Cause**: Position actuator has non-zero `kv` when it should be neutralized

**Explanation**:
When the position actuator is **actively used** for position control:

```
τ = kp*(pos_cmd - q) - kv*qd  ← kv provides desirable damping
```

But when **neutralized** (MIT mode or no active controller):

```
ctrl = q  →  τ = kp*(q - q) - kv*qd = -kv*qd  ← unwanted damping!
```

This velocity-dependent torque interferes with torque control.

**Solution**:

- If using **only position control**: Non-zero `kv` is OK and beneficial
- If using **MIT mode** (position + effort): Set `kv=0.0` in MuJoCo model:

  ```xml
  <position name="act_pos_openarm_joint1" joint="openarm_joint1"
            kp="100.0" kv="0.0" ... />
  ```

---

## API Reference

### Hardware Interface Callbacks

```cpp
// Initialization
hardware_interface::CallbackReturn on_init(
  const hardware_interface::HardwareInfo& info);

// Activation/Deactivation
hardware_interface::CallbackReturn on_activate(
  const rclcpp_lifecycle::State& previous_state);
hardware_interface::CallbackReturn on_deactivate(
  const rclcpp_lifecycle::State& previous_state);

// Control loop
hardware_interface::return_type read(
  const rclcpp::Time& time, const rclcpp::Duration& period);
hardware_interface::return_type write(
  const rclcpp::Time& time, const rclcpp::Duration& period);

// Mode switching
hardware_interface::return_type prepare_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces);
hardware_interface::return_type perform_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces);
```

### Controller Interface

```cpp
// Initialization
controller_interface::CallbackReturn on_init();
controller_interface::CallbackReturn on_configure(
  const rclcpp_lifecycle::State& previous_state);

// Activation
controller_interface::CallbackReturn on_activate(
  const rclcpp_lifecycle::State& previous_state);
controller_interface::CallbackReturn on_deactivate(
  const rclcpp_lifecycle::State& previous_state);

// Control loop
controller_interface::return_type update(
  const rclcpp::Time& time, const rclcpp::Duration& period);

// Interface management
controller_interface::InterfaceConfiguration command_interface_configuration();
controller_interface::InterfaceConfiguration state_interface_configuration();
```

---

## Glossary

**Actuator-Centric Control**: Control strategy where actuators are driven based on active command interfaces, with inactive actuators neutralized.

**MIT Mode**: Control mode where torque is computed as PD control with feedforward: `τ = Kp*(q_cmd - q) + Kd*(qd_cmd - qd) + τ_ff`.

**Neutralization**: Setting actuator control signal to generate zero torque (e.g., `ctrl_pos = q` for position actuator).

**Hardware Interface**: ROS2 Control component that interfaces between controllers and physical/simulated hardware.

**Command Interface**: ROS2 Control interface for sending commands (position, velocity, effort).

**State Interface**: ROS2 Control interface for reading state (position, velocity, effort).

**Feedforward Torque**: Torque computed from system model (e.g., gravity compensation) added to feedback control.

---

**Document Version**: 1.0
**Last Updated**: November 13, 2025
**Status**: Production Ready
