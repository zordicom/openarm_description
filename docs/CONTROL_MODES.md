# MuJoCo Control Modes and Dynamic Switching

**Copyright 2025 Zordi, Inc. All rights reserved.**

## Overview

OpenArm MuJoCo simulation features **dynamic control mode switching** that automatically adapts based on the active controller, matching real DAMIAO motor behavior.

### Two Control Modes

| Mode | Triggered When | Implementation | Real Hardware Analog |
|------|---------------|----------------|---------------------|
| **position_servo** | Trajectory controller (pos+vel, no effort) | MuJoCo actuators (kp=50000) | DAMIAO Position Mode |
| **mit** (default) | Any other combination | Manual PID: τ = Kp*(p-p_cmd) + Kd*(v-v_cmd) + τ_ff | DAMIAO MIT Mode |

**Mode switching is automatic** - happens when you switch controllers via ros2_control.

---

## Architecture

### Three Layers

```
Controllers (ROS2):
  ├─ joint_trajectory_controller → Claims: position + velocity
  ├─ effort_controller → Claims: effort
  └─ zordi_mit_controller → Claims: position + velocity + effort

Hardware Interfaces (Always Exposed):
  ├─ position (read/write)
  ├─ velocity (read/write)
  └─ effort (read/write)

Control Modes (Dynamic, Internal):
  ├─ position_servo → MuJoCo actuators
  └─ mit → Manual PID
```

**Key:** Interfaces are always the same. Mode determines HOW hardware uses them.

---

## How Mode Switching Works

### Automatic Detection

**In mujoco_system.cpp `perform_command_mode_switch()`:**

```cpp
if (has_position && has_velocity && !has_effort)
  current_motor_mode_ = "position_servo";
else
  current_motor_mode_ = "mit";  // Default for everything else
```

### Mode Behaviors

**position_servo mode:**

```cpp
// Uses MuJoCo position actuators
mj_data->ctrl[actuator_id] = position_command;
// MuJoCo computes: τ = kp*(ctrl - q) + kv*(0 - v)
// Auto-compensates gravity with high stiffness (kp=50000)
```

**mit mode:**

```cpp
// Combines all active components
τ = 0;
if (is_position_enabled) τ += Kp * (pos_cmd - pos);
if (is_velocity_enabled) τ += Kd * (vel_cmd - vel);
if (is_effort_enabled) τ += effort_cmd;
mj_data->qfrc_applied[joint] = τ;
```

---

## Controller Suite

### 1. joint_trajectory_controller (Standard)

**Type:** ros2_controllers/JointTrajectoryController
**Interfaces:** position + velocity
**Mode:** → position_servo (stiff, actuator-based)
**Use:** Basic trajectory tracking

### 2. effort_controller (Standard)

**Type:** effort_controllers/JointGroupEffortController
**Interfaces:** effort only
**Mode:** → mit (Kp=0, Kd=0, pure torque)
**Use:** Gravity compensation testing

### 3. zordi_mit_controller (Custom) ⭐

**Type:** zordi_mit_controller/ZordiMITController
**Interfaces:** position + velocity + effort (configurable!)
**Mode:** → mit (full capability)
**Features:**

- ✅ Configurable interfaces (like JointTrajectoryController)
- ✅ Trajectory following with effort feedforward
- ✅ Gravity compensation (Pinocchio)
- ✅ Additive feedforward (computed + trajectory)

### 4. CartesianController (CRISP)

**Type:** crisp_controllers/CartesianController
**Interfaces:** position + velocity + effort
**Mode:** → mit (full)
**Use:** Cartesian space control with full dynamics

---

## Configuration

### controllers_all.yaml

```yaml
controller_manager:
  ros__parameters:
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    effort_controller:
      type: effort_controllers/JointGroupEffortController
    zordi_mit_controller:
      type: zordi_mit_controller/ZordiMITController

# Standard trajectory controller
joint_trajectory_controller:
  command_interfaces: [position, velocity]  # No effort
  # Triggers: position_servo mode

# Effort-only controller
effort_controller:
  interface_name: effort
  # Triggers: mit mode (Kp=0, Kd=0)

# Full MIT mode controller
zordi_mit_controller:
  command_interfaces: [position, velocity, effort]  # All three!
  use_gravity_compensation: true
  # Triggers: mit mode (full)
```

### PID Gains (URDF)

```xml
<!-- urdf/ros2_control/openarm.ros2_control.xacro -->
<param name="position_kp">20.0</param>  <!-- Hardware applies these -->
<param name="position_kd">2.0</param>
```

**Gains are hardware-specific, not controller config!**

---

## Usage

### Launch

```bash
ros2 launch openarm_description single_arm.launch.py

# Or start with specific controller:
ros2 launch openarm_description single_arm.launch.py \
  default_controller:=zordi_mit_controller
```

### Switch Controllers

```bash
# To full MIT mode
ros2 control switch_controllers --activate zordi_mit_controller

# To position servo
ros2 control switch_controllers --activate joint_trajectory_controller

# To effort only
ros2 control switch_controllers --activate effort_controller
```

### Send Trajectories

**To zordi_mit_controller (with gravity comp):**

```python
trajectory.points = [
  JointTrajectoryPoint(
    positions=[0, 0, 0, 0, 0, 0, 0],
    velocities=[0, 0, 0, 0, 0, 0, 0],
    effort=[0, 0, 5, 0, 0, 0, 0],  # Additional force (adds to gravity!)
    time_from_start=Duration(sec=0)
  ),
  ...
]
```

**Computed effort = gravity + trajectory effort (additive!)**

---

## Key Changes from Main Branch

### mujoco_ros2_control

**1. Dynamic Mode Switching**

- Added `current_motor_mode_` variable
- Implemented `perform_command_mode_switch()` to detect interface patterns
- Automatic switching between position_servo and mit modes

**2. Full MIT Mode Support**

- Position + velocity + effort work together (not mutually exclusive)
- Combines all three: τ = Kp*pos_err + Kd*vel_err + effort_cmd
- Matches real DAMIAO MIT mode exactly

**3. Position Actuator Support**

- Added `mj_actuator_id` field to JointState
- Maps joints to MuJoCo position actuators
- Uses `ctrl` commands in position_servo mode

**Changes:** +134 lines, -169 lines (net: -35, simplified!)

### openarm_description

**1. Standard Interface Names**

- Always exposes: position, velocity, effort (same as real hardware)
- Removed `_pid` suffix (was breaking ros2_control compatibility)

**2. Simplified Configuration**

- Single `controllers_all.yaml` with all controllers
- Removed 4 redundant config files
- Dynamic spawning based on `default_controller` parameter

**3. Position Actuator Generation**

- Updated `urdf_to_mjcf.py` to generate actuators with kp=50000
- High stiffness mimics firmware-controlled Position Mode

**4. Launch File Improvements**

- Removed `position_control_mode` parameter
- Added `default_controller` parameter
- Simplified spawner logic

---

## For More Details

- **Controller Feedforward:** See `controllers.md`
- **Setup Instructions:** See `setup.md`
- **Torque vs Actuators:** See `torque_vs_actuator_control.md`

---

**Copyright 2025 Zordi, Inc. All rights reserved.**
