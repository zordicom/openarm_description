# OpenARM - Zordi Setup Guide

**Last Updated**: November 10, 2025

## Overview

This guide covers complete installation and setup for the Zordi OpenARM project, including:

- **MuJoCo Simulation** - Physics-based robot simulation with ros2_control
- **CRISP Controllers** - Torque-based Cartesian and joint-space controllers
- **Hardware Interface** - Dynamic controller switching with custom mujoco_ros2_control fork

> **✨ New Feature**: The MuJoCo viewer now supports interactive perturbations! Hold **Ctrl+Right-click** on any robot body to apply external forces for testing controllers. See [Interactive Perturbations Guide](./interactive_perturbations.md) for details.

## Prerequisites

### System Requirements

- Ubuntu 22.04 (Jammy)
- ROS2 Humble (system-wide installation)
- Python 3.10+

---

## Installation

### 1. Install ROS2 Humble (System-Wide)

If not already installed:

```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

**Source ROS2 in your `~/.bashrc`**:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Pinocchio (ROS2 Version)

**IMPORTANT**: Use the ROS2 package, not conda or standalone pip installation.

```bash
sudo apt install ros-humble-pinocchio
```

This provides:

- **C++ libraries** - For building CRISP controllers (optional, for advanced users)
- **Python bindings** - For `gravity_compensation_controller.py` and dynamics computations
- **CMake integration** - For colcon build system
- Proper integration with ROS2 ecosystem

**How it works:**

The ROS2 version installs Python bindings to `/opt/ros/humble/lib/python3.10/site-packages/pinocchio/`, which become available when you source ROS2:

```bash
source /opt/ros/humble/setup.bash
python3 -c "import pinocchio as pin; print(f'Pinocchio {pin.__version__}')"
```

Expected output: `Pinocchio 3.8.0`

**Note**: You do NOT need `pip install pin` if using the ROS2 version.

### 3. Install MuJoCo and Visualization Python Packages

```bash
pip install mujoco urdf2mjcf matplotlib
```

This provides:

- `mujoco` - Python bindings for MuJoCo physics engine
- `urdf2mjcf` - URDF to MuJoCo MJCF converter
- `matplotlib` - Live visualization for gravity compensation controller

### 4. Install Custom mujoco_ros2_control Fork (REQUIRED)

**Why a custom fork?**
Standard `mujoco_ros2_control` has conflicts when position/velocity/effort interfaces are all exposed. Our fork detects active controllers and prevents interference, enabling dynamic controller switching.

```bash
cd ~/ros2_ws/src
git clone https://github.com/zordicom/mujoco_ros2_control.git
cd mujoco_ros2_control
git checkout 2025-11-control-interface  # Commit: 5a1e443

cd ~/ros2_ws
colcon build --packages-select mujoco_ros2_control --symlink-install
source ~/ros2_ws/install/setup.bash
```

**Verify installation**:

```bash
cd ~/ros2_ws/src/mujoco_ros2_control
git status  # Should show branch: 2025-11-control-interface
git log --oneline -1  # Should show latest commits with controller switching callbacks
```

### 5. Install Additional ROS2 Dependencies

```bash
sudo apt install \
  ros-humble-controller-interface \
  ros-humble-hardware-interface \
  ros-humble-realtime-tools \
  ros-humble-generate-parameter-library \
  ros-humble-pluginlib \
  ros-humble-control-msgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers

# Install build tools
sudo apt install \
  python3-colcon-common-extensions \
  python3-rosdep \
  ccache
```

### 6. Build OpenARM Packages

```bash
cd ~/ros2_ws

# Build all OpenARM packages
colcon build --packages-select \
  openarm_description \
  mujoco_ros2_control \
  --symlink-install

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

**Add to your `~/.bashrc` for convenience**:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 7. Build CRISP Controllers (Optional - Advanced Users Only)

**CRISP controllers are NOT required** for testing `gravity_compensation_controller.py`. They provide advanced Cartesian control features (impedance control, OSC, etc.) but are optional.

**Only build if you need:**

- CartesianController (end-effector pose control)
- TorqueFeedbackController (compliant control)
- Advanced torque-based control beyond gravity compensation

```bash
cd ~/ros2_ws
colcon build --packages-select crisp_controllers --symlink-install
source ~/ros2_ws/install/setup.bash
```

**Verify build**:

```bash
ls ~/ros2_ws/install/crisp_controllers/lib/libcrisp_controllers.so
ros2 pkg list | grep crisp
```

**Note**: Pinocchio (from Step 2) provides both the C++ libraries for building CRISP controllers and the Python bindings for gravity compensation scripts.

---

## Generate MuJoCo Models

Before running simulation, generate MuJoCo MJCF models from URDF:

```bash
cd ~/ros2_ws/src/openarm_description

# Single arm (7 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml

# With hand (9 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --hand --output mujoco_models/openarm_v10_hand.xml

# Bimanual (14 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --bimanual --output mujoco_models/openarm_v10_bimanual.xml
```

**What this does:**

- Converts URDF to MuJoCo MJCF format
- Copies STL mesh files to `mujoco_models/meshes/`
- Removes auto-generated actuators (we use direct torque control)
- Validates the generated model

---

## Quick Start

### Launch MuJoCo Simulation

**Basic launch** (starts with trajectory controller active):

```bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch openarm_description single_arm.launch.py
```

This launches:

- MuJoCo physics simulation with dynamic mode switching
- **joint_trajectory_controller** (active by default - position_servo mode)
- effort_controller (inactive - pure torque)
- zordi_mit_controller (inactive - full MIT mode)
- Joint state broadcaster
- RViz visualization

**Switching to Effort Control with Gravity Compensation**:

The simulation always starts in position control mode. To use effort control:

```bash
# Terminal 1: Launch simulation (position control active)
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch openarm_description mujoco_sim.launch.py

# Terminal 2: Start gravity compensation controller FIRST
# (This prevents the arm from dropping when we switch)
# IMPORTANT: Source both ROS2 and workspace for Pinocchio and xacro
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/gravity_compensation_controller.py

# This will open a live plot window showing:
# - Gravity, damping, and total torques for joints 1-3
# - Joint positions and velocities
# - Updates in real-time at 1 Hz

# Terminal 3: Switch to effort controller
# (Gravity comp is already publishing, so arm won't drop)
ros2 control switch_controllers \
  --activate effort_controller \
  --deactivate joint_trajectory_controller

# The robot should now hold its position under gravity compensation!
# Watch the live plot to see if torques are correct
```

**Why start gravity comp first?**

- Gravity compensation controller publishes to `/effort_controller/commands`
- If you switch to effort control without commands → arm drops immediately
- Starting it first ensures torque commands are ready when you switch

**Why source the workspace?**

- The script uses Pinocchio (requires ROS2 environment)
- It processes `.xacro` files (requires workspace with `openarm_description` package)
- Without sourcing, you'll get `ModuleNotFoundError: No module named 'pinocchio'` or xacro errors

**Live Visualization:**

The gravity compensation controller includes a real-time matplotlib plot showing:

- **Torque Components**: Gravity, damping, Coriolis (if enabled), and total torques
- **Joint Positions**: Track if joints are drifting
- **Joint Velocities**: Monitor oscillations or instability

To disable the plot (e.g., for headless operation):

```bash
python3 scripts/gravity_compensation_controller.py --ros-args -p enable_plot:=false
```

### Test Position Control

```bash
# Use example script
cd ~/ros2_ws/src/openarm_description
python3 scripts/example_position_control.py

# Or send manual command
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [openarm_joint1, openarm_joint2, openarm_joint3, openarm_joint4, openarm_joint5, openarm_joint6, openarm_joint7]
  points:
  - positions: [0.5, 0.3, 0.2, 0.8, 0.0, 0.0, 0.0]
    time_from_start: {sec: 3}
"
```

### Dynamic Controller Switching

```bash
# List all controllers
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

## Usage

### Setting Up Your Environment

**IMPORTANT**: Always source the ROS2 environment before working with OpenARM.

```bash
# In every new terminal:
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Or if you added them to `~/.bashrc`, just open a new terminal.

### Available Control Modes

| Mode | Controller | Status | Use Case |
|------|-----------|--------|----------|
| **Position** | `joint_trajectory_controller` | ✅ Production ready | Trajectory following, waypoint navigation |
| **Velocity** | `velocity_controller` | ⚠️ Unstable | Needs tuning, not recommended |
| **Effort** | `effort_controller` | ✅ Working | Torque control with gravity compensation |

### Launch Arguments

```bash
ros2 launch openarm_description mujoco_sim.launch.py [ARGS]

Arguments:
  arm_type:=v10                  # ARM type (default: v10)
  hand:=false                    # Include hand/gripper
  bimanual:=false                # Bimanual configuration
  use_rviz:=true                 # Launch RViz
  use_sim_time:=true             # Use simulation time
  mujoco_model_path:=<path>      # Custom MuJoCo model (optional)
```

**Note**: The simulation always starts in **position control mode**. To use velocity or effort control, switch controllers at runtime using `ros2 control switch_controllers`.

### Example Configurations

**Single arm with hand**:

```bash
ros2 launch openarm_description mujoco_sim.launch.py hand:=true
```

**Bimanual setup**:

```bash
ros2 launch openarm_description mujoco_sim.launch.py bimanual:=true
```

**Without RViz** (headless, for testing):

```bash
ros2 launch openarm_description mujoco_sim.launch.py use_rviz:=false
```

---

## CRISP Controllers (Optional - Advanced)

**Note**: CRISP controllers are optional and only needed for advanced Cartesian control features. The basic gravity compensation controller works without them.

### Available Controllers

CRISP Controllers provides the following plugins for advanced control:

1. **CartesianController** - 3D pose control with impedance control or OSC
2. **PoseBroadcaster** - Broadcast end-effector pose
3. **TwistBroadcaster** - Broadcast end-effector velocity
4. **TorqueFeedbackController** - Torque feedback with PD control and friction compensation

### Using CRISP Controllers

```bash
# Terminal 1: Launch robot (starts in position control)
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch openarm_description mujoco_sim.launch.py

# Terminal 2: Switch to effort control
ros2 control switch_controllers \
  --activate effort_controller \
  --deactivate joint_trajectory_controller

# Terminal 3: Load and activate CRISP controller
ros2 control load_controller cartesian_controller
ros2 control set_controller_state cartesian_controller active

# Terminal 4: Send Cartesian commands
# (controller-specific commands - see CRISP documentation)
```

---

## Common Issues & Troubleshooting

### Issue: Symbol lookup error when loading controllers

**Symptom**:

```
symbol lookup error: undefined symbol: _ZN20controller_interface23ControllerInterfaceBaseD2Ev
```

**Cause**: ROS2 environment not properly sourced.

**Solution**:

```bash
# Make sure to source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Then launch
ros2 launch openarm_description mujoco_sim.launch.py
```

Alternatively, set `LD_LIBRARY_PATH` before launching:

```bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
ros2 launch openarm_description mujoco_sim.launch.py
```

### Issue: MuJoCo model not found

**Symptom**:

```
Error: Could not find MuJoCo model at: mujoco_models/openarm_v10.xml
```

**Solution**:

Generate the MuJoCo model first:

```bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml
```

### Issue: Robot falls under gravity in effort control

**Symptom**: Robot collapses immediately when switching to effort controller.

**Cause**: Effort control requires external gravity compensation. If no torque commands are being published when you switch, gravity pulls the arm down.

**Solution**: Start gravity compensation controller BEFORE switching to effort control:

```bash
# Terminal 1: Make sure simulation is running
ros2 control list_controllers  # Verify position controller is active

# Terminal 2: Start gravity comp (publishes to /effort_controller/commands)
cd ~/ros2_ws/src/openarm_description
python3 scripts/gravity_compensation_controller.py

# Terminal 3: Wait for "initialized" message, then switch
ros2 control switch_controllers \
  --activate effort_controller \
  --deactivate joint_trajectory_controller

# Arm should hold position now!
```

**Why this order matters:**

- Gravity comp controller starts publishing torque commands immediately
- These commands are ignored while position controller is active
- When you switch to effort controller, commands are already flowing → no drop!

### Issue: Pinocchio not found during build

**Symptom**:

```
CMake Error: Could not find package pinocchio
```

**Solution**:

```bash
# Install ROS2 version of Pinocchio
sudo apt install ros-humble-pinocchio

# Verify it's in the right location
ls /opt/ros/humble/lib/x86_64-linux-gnu/cmake/pinocchio/

# Make sure ROS2 is sourced before building
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select openarm_description crisp_controllers
```

### Issue: Custom mujoco_ros2_control not detected

**Symptom**: Controllers interfere with each other, switching doesn't work properly.

**Solution**:

Verify the custom fork is installed:

```bash
cd ~/ros2_ws/src/mujoco_ros2_control
git remote -v  # Should show zordicom/mujoco_ros2_control
git branch     # Should show 2025-11-control-interface

# If wrong version, reinstall:
cd ~/ros2_ws/src
rm -rf mujoco_ros2_control
git clone https://github.com/zordicom/mujoco_ros2_control.git
cd mujoco_ros2_control
git checkout 2025-11-control-interface

cd ~/ros2_ws
colcon build --packages-select mujoco_ros2_control --symlink-install
```

### Issue: Controllers not switching

**Solution**:

Check controller status:

```bash
ros2 control list_controllers
```

Expected output:

```
joint_state_broadcaster     [active]
joint_trajectory_controller [active]    ← or inactive
velocity_controller         [inactive]  ← or active
effort_controller           [inactive]  ← or active
```

Only one controller (position/velocity/effort) should be active at a time.

### Issue: Build fails with missing dependencies

**Solution**:

```bash
# Update rosdep
sudo rosdep init  # Only needed first time
rosdep update

# Install all dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build --symlink-install
```

---

## Development

### Rebuilding After Changes

```bash
cd ~/ros2_ws

# Rebuild specific packages
colcon build --packages-select openarm_description --symlink-install

# Or rebuild all
colcon build --symlink-install

# Use --symlink-install to avoid rebuilding for Python script changes
```

### Running Tests

```bash
cd ~/ros2_ws

# Test specific package
colcon test --packages-select openarm_description
colcon test-result --verbose

# Test all packages
colcon test
colcon test-result --verbose
```

### Debug Commands

```bash
# Check joint states
ros2 topic echo /joint_states --once

# List hardware interfaces
ros2 control list_hardware_interfaces

# View MuJoCo node logs
ros2 run rqt_console rqt_console

# Monitor controller manager
ros2 node info /controller_manager
```

---

## Notes on Conda Environments

**Do NOT use conda for ROS2 development.**

The system-wide ROS2 installation is the correct approach because:

- ROS2 packages expect system Python (`/usr/bin/python3`)
- CMake integration works seamlessly with system packages
- No path conflicts or library version mismatches
- Standard ROS2 workflow

**Conda environments** can cause library conflicts and are not needed for ROS2 work.

---

## Architecture

### Software Stack

```
┌─────────────────────────────────────────┐
│   High-Level Controllers                │
│   (CRISP, Cartesian, Custom)            │
└───────────────┬─────────────────────────┘
                ↓
┌─────────────────────────────────────────┐
│   ros2_control Framework                │
│   (Controller Manager)                  │
└───────────────┬─────────────────────────┘
                ↓
┌─────────────────────────────────────────┐
│   mujoco_ros2_control (Custom Fork)     │
│   - Dynamic controller switching        │
│   - Direct torque control (qfrc_applied)│
└───────────────┬─────────────────────────┘
                ↓
┌─────────────────────────────────────────┐
│   MuJoCo Physics Engine                 │
│   - Forward dynamics                    │
│   - Gravity, contacts, friction         │
└─────────────────────────────────────────┘
```

### Key Technologies

- **MuJoCo**: Physics simulation engine with forward dynamics
- **ros2_control**: Hardware abstraction and controller management
- **Pinocchio**: Rigid body dynamics (kinematics, Jacobians, gravity compensation)
- **Eigen3**: Linear algebra operations
- **URDF/MJCF**: Robot description formats

---

## Related Documentation

- **MuJoCo Integration Status**: [`docs/mujoco_status.md`](mujoco_status.md) - Comprehensive reference and API documentation
- **Torque Control vs Actuators**: [`docs/torque_vs_actuator_control.md`](torque_vs_actuator_control.md) - Deep dive on control approaches
- **Controller Testing Guide**: See "Testing the Controller Switching Implementation" in `mujoco_status.md`

---

## References

- [CRISP Controllers GitHub](https://github.com/utiasDSL/crisp_controllers)
- [Custom mujoco_ros2_control Fork](https://github.com/zordicom/mujoco_ros2_control)
- [Pinocchio Documentation](https://stack-of-tasks.github.io/pinocchio/)
- [ros2_control Documentation](https://control.ros.org/)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)

---

**Copyright 2025 Zordi, Inc. All rights reserved.**
