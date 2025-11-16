# MuJoCo Keyframe Migration Summary

**Date**: 2025-11-14
**Status**: ✅ Complete

---

## Overview

Migrated OpenARM system from YAML-based initial pose configuration to native MuJoCo XML keyframes, aligning with the latest `mujoco_ros2_control` architecture (§7).

---

## Changes Made

### 1. ✅ MuJoCo XML Models Updated

**Files modified**:

- `mujoco_models/openarm_v10.xml`
- `mujoco_models/openarm_v10_hand.xml`

**Changes**:

- Added `<keyframe>` section with two keyframes:
  - `home`: All joints at zero (default)
  - `pose1`: Test configuration [1.0, 1.5, -1.0, 2.0, 1.0, -1.5, 1.5]
- Reformatted actuator section for readability
- **Verified**: All three actuators per joint present with correct naming

**Removed**: `stable_hanging` pose (simplified to just home/pose1)

---

### 2. ✅ Launch Files Updated

**Files modified**:

- `launch/single_arm.launch.py`
- `launch/test_openarm_multimode.launch.py`

**Changes**:

- Replaced `initial_pose` parameter with `initial_keyframe`
- Removed `initial_pose_config` YAML path parameter
- Updated default from `stable_hanging` to `home`
- Updated docstrings and usage examples
- Pass `initial_keyframe` to mujoco_ros2_control node

**New usage**:

```bash
# Default (home keyframe)
ros2 launch openarm_description single_arm.launch.py

# Specific keyframe
ros2 launch openarm_description single_arm.launch.py initial_keyframe:=pose1
```

---

### 3. ✅ Documentation Updated

**Files modified**:

- `docs/README.md`

**Changes**:

- Updated launch examples to use `initial_keyframe`
- Added keyframe section with available options
- Added runtime reset service documentation
- Removed `stable_hanging` references
- Updated troubleshooting section

**New capabilities documented**:

```bash
# Runtime keyframe reset
ros2 service call /mujoco_ros2_control/reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'home'}"

# Simulation control
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'reset'}"
```

---

## Benefits

### ✅ **Alignment with Latest mujoco_ros2_control**

Now uses native MuJoCo keyframes (§7 feature) instead of custom YAML system.

### ✅ **Runtime Reset Capability**

Can reset to any keyframe during runtime via service call.

### ✅ **Simplified Configuration**

- Removed dependency on separate YAML config file
- Keyframes defined once in MuJoCo XML
- Single source of truth for initial poses

### ✅ **Better Integration**

- Works with simulation control service (§13)
- Compatible with `reset` command
- Leverages MuJoCo's native keyframe mechanism

---

## Migration Guide (For Users)

### Old Way (YAML-based)

```bash
# Old launch
ros2 launch openarm_description single_arm.launch.py \
  initial_pose:=stable_hanging

# Required separate YAML file
config/mujoco/initial_poses.yaml
```

### New Way (XML Keyframes)

```bash
# New launch
ros2 launch openarm_description single_arm.launch.py \
  initial_keyframe:=home

# Keyframes defined in MuJoCo XML
<keyframe>
  <key name="home" qpos="0 0 0 0 0 0 0" />
  <key name="pose1" qpos="1.0 1.5 -1.0 2.0 1.0 -1.5 1.5" />
</keyframe>
```

---

## Available Keyframes

| Keyframe | Description | Joint Positions (rad) |
|----------|-------------|----------------------|
| `home` | Zero configuration (default) | [0, 0, 0, 0, 0, 0, 0] |
| `pose1` | Test configuration | [1.0, 1.5, -1.0, 2.0, 1.0, -1.5, 1.5] |

**Note**: With hand model, gripper joints default to [0, 0].

---

## Runtime Services

### Reset to Keyframe

```bash
ros2 service call /reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'home'}"
```

### Simulation Control

```bash
# Reset to initial keyframe and pause
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'reset'}"

# Status check
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'status'}"
```

---

## Verification

### ✅ Actuators

All models have **three actuators per joint**:

- `act_pos_openarm_joint{1-7}` with `kv="0.0"` (MIT mode compatible)
- `act_vel_openarm_joint{1-7}` with `kv="10.0"`
- `act_tau_openarm_joint{1-7}` (motor)

### ✅ Keyframes

Both models have native XML keyframes:

- `openarm_v10.xml`: 7 DOF (arm only)
- `openarm_v10_hand.xml`: 9 DOF (arm + 2 gripper joints)

### ✅ Launch Files

Both launch files use `initial_keyframe` parameter:

- `single_arm.launch.py`
- `test_openarm_multimode.launch.py`

### ✅ Documentation

README updated with keyframe terminology and examples.

---

## Testing

### Test Launch

```bash
# Default home keyframe
ros2 launch openarm_description single_arm.launch.py

# Verify robot starts at zero configuration
ros2 topic echo /joint_states --once

# Test pose1 keyframe
ros2 launch openarm_description single_arm.launch.py initial_keyframe:=pose1

# Test runtime reset
ros2 service call /reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'home'}"
```

---

## Future Enhancements

### Additional Keyframes

Add more keyframes to XML models for common poses:

```xml
<keyframe>
  <key name="home" qpos="0 0 0 0 0 0 0" />
  <key name="pose1" qpos="1.0 1.5 -1.0 2.0 1.0 -1.5 1.5" />
  <key name="ready" qpos="0 -0.5 0 -1.5 0 1.0 0" />
  <key name="stow" qpos="0 1.5 0 1.5 0 0 0" />
</keyframe>
```

### Integration with MoveIt

Use keyframes as named poses in MoveIt configuration.

---

## References

- **mujoco_ros2_control Updates**: `mujoco_ros2_control/doc/mujoco_ros2_control_updates.md` (§7)
- **MuJoCo Keyframe Documentation**: <https://mujoco.readthedocs.io/en/stable/XMLreference.html#keyframe>
- **Simulation Control**: `mujoco_ros2_control/doc/SIMULATION_CONTROL.md` (§13)

---

**Copyright 2025 Zordi, Inc. All rights reserved.**
