# MuJoCo Simulation for OpenARM - User Guide

## Overview

This guide shows how to use MuJoCo physics simulation with OpenARM robots via ROS2 Control with **dynamic controller switching**. Switch between position, velocity, and effort control at runtime without conflicts or restarts.

**Key Feature:** Uses custom `mujoco_ros2_control` fork (commit `5a1e443`) that enables all three control modes simultaneously without interference.

## Prerequisites

### Install Custom mujoco_ros2_control Fork

**Required:** Our custom fork with dynamic switching support.

```bash
# Clone custom fork
cd ~/ros2_ws/src
git clone https://github.com/zordicom/mujoco_ros2_control.git
cd mujoco_ros2_control
git checkout 2025-11-control-interface

# Build
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select mujoco_ros2_control --symlink-install
source ~/ros2_ws/install/setup.bash

# Install MuJoCo Python
pip install mujoco urdf2mjcf
```

**Why the custom fork?**
The standard `mujoco_ros2_control` has conflicts when position/velocity/effort interfaces are all exposed. Our fork detects active controllers and prevents interference.

## URDF/Xacro Configuration

### What Your Xacro Needs for MuJoCo

To use MuJoCo as your simulation backend, your xacro files need these settings:

**In `openarm.ros2_control.xacro` (or `openarm.bimanual.ros2_control.xacro`):**

```xml
<xacro:macro name="openarm_arm_ros2_control" params="... use_mujoco:=^|false ...">
  <ros2_control name="openarm_hardware_interface" type="system">
    <hardware>
      <!-- MuJoCo Physics Simulation -->
      <xacro:if value="${use_mujoco}">
        <plugin>mujoco_ros2_control/MujocoSystem</plugin>
        <!-- Always use "all" mode to enable dynamic controller switching -->
        <param name="control_mode">all</param>
      </xacro:if>

      <!-- For comparison: Real hardware -->
      <xacro:unless value="${use_mujoco}">
        <plugin>openarm_hardware/OpenArm_${arm_type}HW</plugin>
        <param name="can_interface">${can_interface}</param>
      </xacro:unless>
    </hardware>

    <!-- Define joints with all three command interfaces -->
    <joint name="${arm_prefix}openarm_joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <!-- ... repeat for all joints ... -->
  </ros2_control>
</xacro:macro>
```

**In `v10.urdf.xacro` (top-level robot file):**

```xml
<xacro:arg name="use_mujoco" default="false" />

<xacro:openarm_robot
  arm_type="v10"
  use_mujoco="$(arg use_mujoco)"
  ros2_control="true"
  <!-- ... other params ... -->
/>
```

**Key Points:**

1. **`use_mujoco` parameter** - Switches between MuJoCo and real hardware
2. **`control_mode="all"`** - Hardcoded in xacro to enable dynamic switching (required for our custom fork)
3. **All three interfaces** - Position, velocity, and effort must be exposed on each joint
4. **No manual changes needed** - Already configured in OpenARM xacro files

## Quick Start

### 1. Generate MuJoCo Model

```bash
cd ~/ros2_ws/src/openarm_description

# Single arm (7 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml

# Arm + hand (9 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --hand --output mujoco_models/openarm_v10_hand.xml

# Bimanual (14 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --bimanual --output mujoco_models/openarm_v10_bimanual.xml
```

### 2. Launch with MuJoCo Backend

**The `use_mujoco:=true` parameter switches from real hardware to MuJoCo simulation:**

```bash
# Launch with MuJoCo (position control starts active)
ros2 launch openarm_description mujoco_sim.launch.py

# With hand
ros2 launch openarm_description mujoco_sim.launch.py hand:=true

# Start with effort control active
ros2 launch openarm_description mujoco_sim.launch.py control_mode:=effort

# Without RViz
ros2 launch openarm_description mujoco_sim.launch.py use_rviz:=false
```

**What happens behind the scenes:**

1. Xacro generates URDF with `use_mujoco:=true`
2. MuJoCo plugin loads instead of real hardware driver
3. All three controller configs loaded into hardware interface parameters
4. All three controllers loaded as **inactive** (position, velocity, effort)
5. Only the controller specified by `control_mode` is **activated**
6. You can switch controllers at runtime (see below)

### 3. Dynamic Controller Switching

**List available controllers:**

```bash
ros2 control list_controllers --controller-manager /controller_manager
```

Expected output:

```
joint_state_broadcaster[...] active
joint_trajectory_controller[...] active    # or inactive
velocity_controller[...] inactive          # or active
effort_controller[...] inactive            # or active
```

**Switch between controllers at runtime:**

```bash
# Switch to position control
ros2 control switch_controllers \
  --activate joint_trajectory_controller \
  --deactivate effort_controller velocity_controller \
  --controller-manager /controller_manager

# Switch to velocity control
ros2 control switch_controllers \
  --activate velocity_controller \
  --deactivate joint_trajectory_controller effort_controller \
  --controller-manager /controller_manager

# Switch to effort control
ros2 control switch_controllers \
  --activate effort_controller \
  --deactivate joint_trajectory_controller velocity_controller \
  --controller-manager /controller_manager
```

### 4. Send Commands

**Position Control:**

```bash
# Using example script
python3 scripts/example_position_control.py

# Or manually via action
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['openarm_joint1', 'openarm_joint2', 'openarm_joint3', 'openarm_joint4', 'openarm_joint5', 'openarm_joint6', 'openarm_joint7'], points: [{positions: [0.5, 0.3, 0.2, 0.8, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}}"
```

**Velocity Control (rad/s):**

```bash
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0]" -r 10
```

**Effort Control (N⋅m):**

```bash
# Safe torques
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [20.0, 20.0, 15.0, 15.0, 3.0, 3.0, 3.0]" -r 100

# Max torques (within joint limits)
# Joint 1-2: ±40 Nm, Joint 3-4: ±27 Nm, Joint 5-7: ±7 Nm
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [35.0, 35.0, 25.0, 25.0, 6.0, 6.0, 6.0]" -r 100
```

## Testing Controller Switching

**Complete test sequence (position → effort → velocity → position):**

```bash
# 1. Launch simulation
ros2 launch openarm_description mujoco_sim.launch.py

# 2. Verify all controllers loaded (in another terminal)
ros2 control list_controllers --controller-manager /controller_manager

# 3. Test position control (should start active)
python3 scripts/example_position_control.py

# 4. Switch to effort control
ros2 control switch_controllers \
  --activate effort_controller \
  --deactivate joint_trajectory_controller velocity_controller \
  --controller-manager /controller_manager

# 5. Send effort commands
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [20.0, 20.0, 15.0, 15.0, 3.0, 3.0, 3.0]" -r 100

# Watch robot move for 5 seconds, then Ctrl+C

# 6. Switch to velocity control
ros2 control switch_controllers \
  --activate velocity_controller \
  --deactivate effort_controller joint_trajectory_controller \
  --controller-manager /controller_manager

# 7. Send velocity commands
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0]" -r 10

# Watch robot move, then Ctrl+C

# 8. Switch back to position control
ros2 control switch_controllers \
  --activate joint_trajectory_controller \
  --deactivate velocity_controller effort_controller \
  --controller-manager /controller_manager

# 9. Send position command
python3 scripts/example_position_control.py
```

**What to verify:**

- ✅ Switching completes without errors
- ✅ Robot responds to active controller only
- ✅ No conflicts or interference between controllers
- ✅ MuJoCo viewer shows realistic movement
- ✅ RViz displays match MuJoCo state

## Monitoring

**Check joint states:**

```bash
ros2 topic echo /joint_states
```

**Check which controllers are active:**

```bash
ros2 control list_controllers --controller-manager /controller_manager
```

**View hardware interfaces:**

```bash
ros2 control list_hardware_interfaces --controller-manager /controller_manager
```

## Troubleshooting

**Problem: MuJoCo model not found**

```bash
# Generate the model first
cd ~/ros2_ws/src/openarm_description
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml
```

**Problem: Controllers not switching**

```bash
# Verify all controllers loaded
ros2 control list_controllers --controller-manager /controller_manager

# If not loaded, check that you're using the custom mujoco_ros2_control fork (commit 5a1e443)
```

**Problem: Effort control not moving robot**

This means you're using the standard `mujoco_ros2_control` instead of our custom fork. The standard version has conflicts when all three interfaces are exposed. Install our fork:

```bash
cd ~/ros2_ws/src
git clone https://github.com/zordicom/mujoco_ros2_control.git
cd mujoco_ros2_control
git checkout 2025-11-control-interface
cd ~/ros2_ws && colcon build --packages-select mujoco_ros2_control
```

**Problem: "No transform from [link] to [world]" in RViz**

This is expected - the launch file includes a static transform publisher. If RViz still shows this error, verify `robot_state_publisher` and `joint_state_broadcaster` are active.

## Summary

**What you need:**

1. **Custom fork:** `mujoco_ros2_control` (commit `5a1e443`) - enables dynamic switching
2. **Xacro config:** `use_mujoco:=true` parameter + `control_mode="all"` hardcoded
3. **Three controllers:** All loaded simultaneously, switch at runtime

**Key commands:**

```bash
# Launch
ros2 launch openarm_description mujoco_sim.launch.py

# Switch controllers
ros2 control switch_controllers --activate CONTROLLER --deactivate OTHERS --controller-manager /controller_manager

# Send commands
ros2 topic pub /CONTROLLER/commands ...
```

**Resources:**

- Custom fork: <https://github.com/zordicom/mujoco_ros2_control> (branch: `2025-11-control-interface`)
- MuJoCo docs: <https://mujoco.readthedocs.io/>
- ros2_control: <https://control.ros.org/>

---

Copyright 2025 Zordi, Inc. All rights reserved.
