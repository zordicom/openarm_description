# OpenARM Multimode Testing Setup

**Copyright 2025 Zordi, Inc. All rights reserved.**

**Status**: ✅ Complete
**Date**: 2025-11-14

---

## Overview

This setup provides comprehensive testing of multiple control modes for the OpenARM 7-DOF robot arm.

### Controllers Available

| Controller | Type | Interfaces | Description |
|-----------|------|------------|-------------|
| **joint_trajectory_controller** | Standard ROS2 | pos + vel | Position servo mode (MuJoCo actuators) |
| **zordi_hardware_pd_controller** | ZordiMITController | pos + vel + eff | Hardware PD (MIT mode in mujoco_system.cpp) |
| **zordi_software_pd_controller** | ZordiMITController | eff only | Software PD (PD computed in controller) |
| **zordi_grav_comp_controller** | ZordiMITController | eff only | Pure gravity comp (backdrivable) |

---

## Quick Start

```bash
# Launch with all controllers loaded (inactive)
ros2 launch openarm_description test_openarm_multimode.launch.py

# Check controllers
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster[...] active
# joint_trajectory_controller[...] inactive
# zordi_hardware_pd_controller[...] inactive
# zordi_software_pd_controller[...] inactive
# zordi_grav_comp_controller[...] inactive
```

---

## Controller Details

### 1. Joint Trajectory Controller

**Standard ROS2 position+velocity control** (no effort interface)

```bash
# Activate
ros2 control set_controller_state joint_trajectory_controller active

# Send trajectory
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['openarm_joint1', 'openarm_joint2', 'openarm_joint3',
                  'openarm_joint4', 'openarm_joint5', 'openarm_joint6', 'openarm_joint7'],
    points: [{positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"
```

**Behavior**:

- Uses position and velocity actuators in MuJoCo
- No gravity compensation
- May drift under gravity after trajectory completion

---

### 2. Hardware PD Controller (MIT Mode)

**Claims all three interfaces** - Triggers MIT mode in hardware (mujoco_system.cpp)

```bash
# Activate
ros2 control set_controller_state zordi_hardware_pd_controller active

# Send trajectory (via action)
ros2 action send_goal /zordi_hardware_pd_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: ['openarm_joint1', 'openarm_joint2', 'openarm_joint3',
                  'openarm_joint4', 'openarm_joint5', 'openarm_joint6', 'openarm_joint7'],
    points: [{positions: [0.5, 0.5, -0.5, 1.0, 0.5, -0.5, 0.5], time_from_start: {sec: 3}}]
  }}" --feedback
```

**Behavior**:

- Position/velocity actuators neutralized
- Torque actuator receives: `τ = Kp*(q_cmd - q) + Kd*(qd_cmd - qd) + τ_gravity`
- PD gains from URDF (position_kp, velocity_kd)
- Gravity compensation from controller
- Holds position after trajectory with PD + gravity comp

---

### 3. Software PD Controller

**Claims effort only** - Computes PD internally in controller

```bash
# Activate
ros2 control set_controller_state zordi_software_pd_controller active

# Send trajectory
ros2 action send_goal /zordi_software_pd_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: ['openarm_joint1', 'openarm_joint2', 'openarm_joint3',
                  'openarm_joint4', 'openarm_joint5', 'openarm_joint6', 'openarm_joint7'],
    points: [{positions: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]
  }}"
```

**Behavior**:

- Controller computes: `τ = Kp*(q_cmd - q) + Kd*(qd_cmd - qd) + τ_gravity`
- PD gains from config file (gains section)
- Only torque actuator used in MuJoCo
- Works with any torque-controlled robot (not just MIT-capable hardware)
- Holds position after trajectory

---

### 4. Gravity Compensation Controller

**Claims effort only** - Pure gravity compensation (backdrivable)

```bash
# Activate
ros2 control set_controller_state zordi_grav_comp_controller active

# Apply external wrench to test backdrivability
ros2 service call /apply_external_wrench \
  mujoco_ros2_control_msgs/srv/ApplyExternalWrench \
  "{body_name: 'openarm_link3',
    wrench: {torque: {y: 5.0}},
    duration: 3.0}"
```

**Behavior**:

- Outputs ONLY gravity compensation torques (no PD control)
- Robot is **fully backdrivable** (can be moved by hand/wrenches)
- No trajectory tracking capability
- Maintains zero drift under gravity
- Settles at new position after external forces

---

## Switching Controllers

```bash
# Switch from one to another
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{
    activate_controllers: ['zordi_software_pd_controller'],
    deactivate_controllers: ['zordi_hardware_pd_controller'],
    strictness: 2,
    start_asap: true
  }"
```

---

## Testing Workflow

### Comprehensive Test Sequence

```bash
# 1. Launch (starts paused at home keyframe)
ros2 launch openarm_description test_openarm_multimode.launch.py

# 2. Activate hardware PD controller
ros2 control set_controller_state zordi_hardware_pd_controller active

# 3. Unpause simulation
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"

# 4. Send trajectory
ros2 action send_goal /zordi_hardware_pd_controller/follow_joint_trajectory ...

# 5. Test disturbance rejection
ros2 service call /apply_external_wrench ...
# Should resist and return to commanded position

# 6. Switch to JTC
ros2 service call /controller_manager/switch_controller ...

# 7. Send trajectory via JTC
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory ...

# 8. Switch to software PD
ros2 service call /controller_manager/switch_controller ...

# 9. Send trajectory via software PD
ros2 action send_goal /zordi_software_pd_controller/follow_joint_trajectory ...

# 10. Switch to gravity comp only
ros2 service call /controller_manager/switch_controller \
  "{activate_controllers: ['zordi_grav_comp_controller'],
    deactivate_controllers: ['zordi_software_pd_controller'], strictness: 2}"

# 11. Test backdrivability
ros2 service call /apply_external_wrench ...
# Should move slowly, no return to position
```

---

## Configuration Files

### Controller Config

**File**: `config/mujoco/controllers_multimode_test.yaml`

Contains all four controller configurations with detailed comments.

### Launch File

**File**: `launch/test_openarm_multimode.launch.py`

Loads all controllers in inactive state, starts simulation paused.

### MuJoCo Model

**File**: `mujoco_models/openarm_v10.xml`

Has three actuators per joint + keyframes (home, pose1).

---

## Comparison Table

| Feature | JTC | Hardware PD | Software PD | Grav Comp |
|---------|-----|-------------|-------------|-----------|
| **Trajectory Tracking** | ✅ Yes | ✅ Yes | ✅ Yes | ❌ No |
| **Gravity Comp** | ❌ No | ✅ Yes | ✅ Yes | ✅ Yes |
| **Hold After Trajectory** | ⚠️ Actuator gains | ✅ Excellent | ✅ Excellent | N/A |
| **Backdrivable** | ❌ No | ❌ No | ❌ No | ✅ Yes |
| **Disturbance Response** | Resists | Resists, returns | Resists, returns | Moves slowly |
| **PD Computation** | MuJoCo actuators | Hardware (mujoco_system) | Controller | None |
| **Interfaces** | pos + vel | pos + vel + eff | eff only | eff only |
| **Hardware Requirements** | Any | MIT-capable | Any torque robot | Any torque robot |

---

## Documentation References

- **Testing Guide**: `docs/MANUAL_OPENARM_MULTIMODE_TEST.md`
- **zordi_mit_controller**: `~/ros2_ws/src/zordi_mit_controller/README.md`
- **MuJoCo Architecture**: `~/ros2_ws/src/mujoco_ros2_control/doc/mujoco_ros2_control_updates.md`

---

## Troubleshooting

### Controller Won't Load

```bash
# Check controller manager is running
ros2 control list_controller_types

# Rebuild zordi_mit_controller
cd ~/ros2_ws
colcon build --packages-select zordi_mit_controller
source install/setup.bash
```

### Gains Not Loading (Software PD)

Check `controllers_multimode_test.yaml` - gains section should have all joints defined.

### No Gravity Compensation

```bash
# Check parameter
ros2 param get /zordi_software_pd_controller use_gravity_compensation
# Should return: Boolean value is: True

# Check Pinocchio is installed
python3 -c "import pinocchio; print('OK')"
```

---

**Status**: Production Ready ✅
**Last Updated**: November 14, 2025

**Copyright 2025 Zordi, Inc. All rights reserved.**
