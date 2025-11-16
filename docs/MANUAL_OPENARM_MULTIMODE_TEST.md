# OpenARM Multimode Control Testing Guide

**Copyright 2025 Zordi, Inc. All rights reserved.**

**Purpose**: Comprehensive testing of OpenARM 7-DOF robot arm with multiple control modes and gravity compensation

**System**: OpenARM v10 7-DOF arm (joint1-joint7)
**Controllers**: `joint_trajectory_controller`, `zordi_mit_controller`, `zordi_grav_comp_controller`
**Test Focus**: Controller switching, gravity compensation, disturbance rejection, MIT mode validation

---

## Prerequisites

```bash
# Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Verify Pinocchio is installed
python3 -c "import pinocchio; print('Pinocchio OK')"
```

---

## Quick Start

### Terminal 1: Launch Simulation

```bash
ros2 launch openarm_description test_openarm_multimode.launch.py
```

**Expected**:

- MuJoCo viewer window opens
- OpenARM at home keyframe (all joints at 0.0 rad)
- **Simulation starts PAUSED** (no physics running yet)
- Console shows: `joint_state_broadcaster` active, other controllers inactive

**Note**: The system follows the latest mujoco_ros2_control architecture - simulation starts paused to allow controller setup.

---

## Core Testing Workflow

This workflow tests all major features: keyframes, simulation control, multiple controller modes, gravity compensation, and disturbance rejection.

### Step 1: Verify Initial State

```bash
# Terminal 2: Check system status
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'status'}"

# Expected: current_state: "PAUSED"

# Check controllers
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive
# zordi_mit_controller[zordi_mit_controller/ZordiMITController] inactive
```

### Step 2: Load Initial Pose (Using Keyframes)

```bash
# Robot is already at 'home' keyframe (default)
# Let's move to test pose1 keyframe
ros2 service call /reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'pose1'}"

# Verify position changed
ros2 topic echo /joint_states --once
# Should show: positions: [1.0, 1.5, -1.0, 2.0, 1.0, -1.5, 1.5]
```

### Step 3: Activate Gravity Compensation Controller

```bash
# Start with pure gravity compensation (effort-only mode)
ros2 control set_controller_state zordi_grav_comp_controller active

# Verify it's active
ros2 control list_controllers
# zordi_mit_controller should show "active"
```

### Step 4: Unpause Simulation

```bash
# Now start physics
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"

# Observe in MuJoCo viewer:
# - Robot should hold steady at pose1
# - No drift visible
# - Controller applying gravity compensation
```

**Monitor hold performance (15 seconds):**

```bash
ros2 topic echo /joint_states | grep -A1 "position:"
```

**Expected**: All joints stable within ±0.01 rad

### Step 5: Switch to Joint Trajectory Controller

```bash
# Switch to position+velocity control (no effort)
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{
    activate_controllers: ['joint_trajectory_controller'],
    deactivate_controllers: ['zordi_mit_controller','zordi_grav_comp_controller'],
    strictness: 2,
    start_asap: true
  }"
```

### Step 6: Send Trajectory (JTC)

```bash
# Move to home position
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{
    joint_names: ['openarm_joint1', 'openarm_joint2', 'openarm_joint3',
                  'openarm_joint4', 'openarm_joint5', 'openarm_joint6', 'openarm_joint7'],
    points: [
      {
        positions: [0.0, 1.5, -1.5, 0.0, 1.5, 0.0, 0.0],
        velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 3, nanosec: 0}
      }
    ]
  }"
```

**Observe**: Smooth motion to this pose over 3 seconds

### Step 7: Switch to MIT Controller (Hardware PD)

This mode uses all three interfaces (position + velocity + effort) with PD gains applied in hardware (mujoco_system.cpp).

```bash
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{
    activate_controllers: ['zordi_mit_controller'],
    deactivate_controllers: ['joint_trajectory_controller'],
    strictness: 2,
    start_asap: true
  }"
```

**What happens in MIT mode**:

- Position and velocity actuators neutralized
- Torque actuator receives: `τ = Kp*(q_cmd - q) + Kd*(qd_cmd - qd) + τ_ff`
- PD gains from URDF, feedforward from controller
- Automatic gravity compensation when idle

### Step 8: Send Trajectory (MIT Mode)

```bash
ros2 action send_goal /zordi_software_pd_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['openarm_joint1', 'openarm_joint2', 'openarm_joint3',
                    'openarm_joint4', 'openarm_joint5', 'openarm_joint6', 'openarm_joint7'],
      points: [
        {
          positions: [0.5, 0.5, -0.5, 1.0, 0.5, -0.5, 0.5],
          velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 3, nanosec: 0}
        }
      ]
    }
  }" --feedback
```

**Observe**:

- Smooth trajectory execution with gravity compensation
- **Automatic hold** after completion (no drift)
- Controller continues applying gravity torques

### Step 9: Test Disturbance Rejection (Holding)

While zordi_mit_controller is active and holding position:

```bash
# Apply 10 Nm torque to joint 3 for 2 seconds
ros2 service call /apply_external_wrench \
  mujoco_ros2_control_msgs/srv/ApplyExternalWrench \
  "{
    body_name: 'openarm_link3',
    wrench: {
      force: {x: 0.0, y: 0.0, z: 0.0},
      torque: {x: 0.0, y: 10.0, z: 0.0}
    },
    duration: 2.0
  }"
```

**Expected**:

- Small deviation during wrench application
- Quick recovery after wrench expires
- Returns to commanded position

### Step 10: Test Backdrivability (Pure Gravity Comp)

Now test pure gravity compensation mode (effort-only, no PD control).

**Note**: If you have a dedicated `zordi_grav_comp_controller`, use it. Otherwise, we'll temporarily remove position/velocity interfaces from zordi_mit_controller configuration.

**For testing with existing controllers:**

The zordi_mit_controller with `use_gravity_compensation: true` already provides gravity comp. When no trajectory is active, it holds position with gravity compensation. To test backdrivability, we need a pure effort controller.

**If you have zordi_grav_comp_controller loaded:**

```bash
# Switch to pure gravity comp (no PD)
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{
    activate_controllers: ['zordi_grav_comp_controller'],
    deactivate_controllers: ['zordi_mit_controller'],
    strictness: 2
  }"

# Apply wrench
ros2 service call /apply_external_wrench \
  mujoco_ros2_control_msgs/srv/ApplyExternalWrench \
  "{
    body_name: 'openarm_link3',
    wrench: {
      force: {x: 0.0, y: 0.0, z: 0.0},
      torque: {x: 0.0, y: 5.0, z: 0.0}
    },
    duration: 3.0
  }"
```

**Expected (Pure Gravity Comp)**:

- Robot moves slowly in response to wrench
- **Fully backdrivable** (no position control)
- Maintains gravity compensation (no collapse)
- Settles at new position after wrench expires

**Comparison**:

| Mode | PD Control | Gravity Comp | Disturbance Response |
|------|-----------|--------------|---------------------|
| MIT Mode | ✅ Yes (Kp, Kd) | ✅ Yes | Resists and returns |
| Pure Gravity Comp | ❌ No | ✅ Yes | Backdrivable, moves slowly |
| JTC | ✅ Yes (actuator gains) | ❌ No | Resists, may drift |

---

## Additional Testing Scenarios

### Multi-Point Trajectory (MIT Mode)

```bash
ros2 action send_goal /zordi_mit_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['openarm_joint1', 'openarm_joint2', 'openarm_joint3',
                    'openarm_joint4', 'openarm_joint5', 'openarm_joint6', 'openarm_joint7'],
      points: [
        {positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}},
        {positions: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 4}},
        {positions: [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 6}},
        {positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 8}}
      ]
    }
  }" --feedback
```

### Reset During Operation

```bash
# During any operation, you can reset to a keyframe
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'reset'}"

# This resets to initial_keyframe and pauses simulation
# Then unpause to continue
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"
```

### Different Keyframes

```bash
# Pause simulation
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'pause'}"

# Switch keyframe
ros2 service call /reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'home'}"

# Unpause to continue
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"
```

---

## Validation Checks

### Gravity Compensation Accuracy

Compare controller's gravity compensation with MuJoCo's ground truth:

```bash
# Terminal 1: Monitor MuJoCo's computed gravity torques
ros2 topic echo /mujoco/qfrc_bias

# Terminal 2: Monitor controller's effort commands
ros2 topic echo /joint_states

# Compare the effort values - should match closely
```

### Hold Performance Metrics

```bash
# Record joint states for 10 seconds
ros2 topic echo /joint_states > joint_states_log.txt

# Analyze drift (should be < 0.01 rad per joint)
```

---

## MuJoCo Viewer Controls

- **Left mouse drag**: Rotate view
- **Right mouse drag**: Pan view
- **Scroll wheel**: Zoom
- **Space**: Pause/unpause (alternative to service)
- **Right-click link**: Shows link info

---

## Expected Results

### Control Mode Performance

| Mode | Trajectory Tracking | Hold Stability | Backdrivable | Gravity Comp |
|------|-------------------|----------------|--------------|--------------|
| **JTC** (pos+vel) | ✅ Good | ⚠️ Depends on gains | ❌ No | ❌ No |
| **MIT Mode** (pos+vel+eff) | ✅ Excellent | ✅ Excellent (<0.01 rad) | ❌ No (PD active) | ✅ Yes |
| **Pure Grav Comp** (eff) | N/A | ✅ Stable (no control) | ✅ Yes | ✅ Yes |

### Disturbance Rejection

| Mode | Response to Wrench |
|------|--------------------|
| **MIT Mode** | Resists, returns to commanded position |
| **Pure Grav Comp** | Moves slowly, maintains gravity comp, no return |
| **JTC** | Resists, may drift without gravity comp |

---

## Troubleshooting

### Robot Drifts Under Gravity

**Check**:

1. Gravity compensation enabled:

   ```bash
   ros2 param get /zordi_mit_controller use_gravity_compensation
   # Should return: Boolean value is: True
   ```

2. Controller is active
3. Pinocchio installed: `python3 -c "import pinocchio; print('OK')"`

### Controller Won't Switch

```bash
# Stop all controllers first
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{deactivate_controllers: ['joint_trajectory_controller', 'zordi_mit_controller'], strictness: 1}"

# Then activate desired controller
ros2 control set_controller_state <controller_name> active
```

### Simulation Won't Unpause

```bash
# Check current state
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'status'}"

# Force unpause
ros2 service call /simulation_control \
  mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"
```

### Wrench Not Applied

**Verify**:

1. Body name matches MuJoCo model: `openarm_link1` through `openarm_link7`
2. Wrench is in world frame
3. Duration is reasonable (1-5 seconds)

```bash
# List available bodies
ros2 service call /mujoco_ros2_control/list_bodies # (if service exists)
# Or check XML: cat ~/ros2_ws/src/openarm_description/mujoco_models/openarm_v10.xml | grep "body name"
```

---

## Cleanup

```bash
# Ctrl+C in Terminal 1 to stop launch file
# Or kill process:
pkill -9 -f "test_openarm_multimode"
```

---

## Configuration Files

### Controllers Loaded

From `config/mujoco/controllers_all.yaml`:

1. **joint_state_broadcaster** - Always active, publishes joint states
2. **joint_trajectory_controller** - Position + velocity interfaces
3. **zordi_mit_controller** - Position + velocity + effort interfaces (MIT mode)
4. **effort_controller** (optional) - Effort-only interface

### Keyframes Available

From `mujoco_models/openarm_v10.xml`:

- `home`: All joints at zero [0, 0, 0, 0, 0, 0, 0]
- `pose1`: Test configuration [1.0, 1.5, -1.0, 2.0, 1.0, -1.5, 1.5]

---

## Next Steps

1. **Tune PID Gains**: Modify `position_kp` and `velocity_kd` in URDF for your requirements
2. **Add More Keyframes**: Define additional test poses in MuJoCo XML
3. **Custom Controllers**: Implement your own controllers following `zordi_mit_controller` as reference
4. **Integration Testing**: Test with your application-specific trajectories
5. **Hardware Validation**: Compare simulation results with real hardware

---

## References

- **System Architecture**: `docs/TECHNICAL_REFERENCE.md`
- **Keyframe Migration**: `docs/KEYFRAME_MIGRATION.md`
- **MuJoCo ROS2 Control**: `mujoco_ros2_control/doc/mujoco_ros2_control_updates.md`
- **Demo Examples**: `mujoco_ros2_control/mujoco_ros2_control_demos/README.md`

---

**Document Version**: 2.0
**Last Updated**: November 14, 2025
**Status**: Updated for keyframe-based initial poses and comprehensive testing workflow

**Copyright 2025 Zordi, Inc. All rights reserved.**
