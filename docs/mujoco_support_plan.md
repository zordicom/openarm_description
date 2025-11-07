# MuJoCo ROS2 Control Integration for OpenARM

## Overview

This document provides an overview of the MuJoCo physics simulation integration with OpenARM robots. MuJoCo provides realistic dynamics simulation including gravity, inertia, friction, and contacts, enabling accurate testing of control algorithms before hardware deployment.

**Key Achievement:** This integration includes a **custom `mujoco_ros2_control` fork** that enables **dynamic switching** between position, velocity, and effort controllers at runtime without conflicts. This solves a fundamental limitation in the standard package where multiple command interfaces would interfere with each other.

## Architecture

```
ROS2 Controllers (JointTrajectory, Velocity, Effort)
                    ↕
        ros2_control (Controller Manager)
                    ↕
    ┌───────────────┬────────────────┬──────────────────┐
    │ Real Hardware │ Fake Hardware  │ MuJoCo System    │
    │ (CAN Bus)     │ (Mock)         │ (Physics Sim)    │
    └───────────────┴────────────────┴──────────────────┘
```

The integration supports seamless switching between real hardware, mock interface, and physics simulation using a single xacro parameter.

## Features

✅ **Three Control Modes**

- Position control (trajectory following)
- Velocity control (direct velocity commands)
- Effort control (torque-based control)

✅ **Full Hardware Compatibility**

- Single arm configurations
- Bimanual configurations
- Hand/gripper support

✅ **Plug-and-Play Integration**

- Switch simulation mode with one parameter: `use_mujoco:=true`
- Compatible with existing ros2_control infrastructure
- No changes needed to controller code

## Implementation Summary

The integration enables MuJoCo physics simulation as a hardware interface option in the existing ros2_control framework. The implementation adds a `use_mujoco` parameter that seamlessly switches between real hardware, mock interface, and physics simulation without requiring changes to controller code or launch files.

**Modified Files:**

- `urdf/ros2_control/openarm.ros2_control.xacro` - Added MuJoCo plugin option
- `urdf/ros2_control/openarm.bimanual.ros2_control.xacro` - Added MuJoCo plugin option for bimanual
- `urdf/robot/openarm_robot.xacro` - Pass-through for MuJoCo parameter
- `urdf/robot/v10.urdf.xacro` - Added use_mujoco argument

**Created Files:**

- `scripts/urdf_to_mjcf.py` - URDF to MJCF converter using urdf2mjcf Python API with automatic:
  - Package URI to absolute path conversion
  - Inertia matrix balancing for hand/gripper models
  - Mesh file copying to output directory
  - Mesh path fixing for MuJoCo compatibility
  - Freejoint removal for fixed-base robots
  - Fallback to direct MuJoCo loading if urdf2mjcf fails
- `scripts/test_mujoco_setup.py` - Validation test suite
- `scripts/example_position_control.py` - Position control example
- `config/mujoco/v10_sim_params.yaml` - Simulation parameters (damping, friction, solver)
- `config/mujoco/controllers_position.yaml` - JointTrajectoryController configuration
- `config/mujoco/controllers_velocity.yaml` - JointGroupVelocityController configuration
- `config/mujoco/controllers_effort.yaml` - JointGroupEffortController configuration
- `launch/mujoco_sim.launch.py` - Simulation launch with control mode selection
- `rviz/mujoco_view.rviz` - RViz visualization configuration
- `docs/mujoco_usage.md` - User guide and troubleshooting
- `docs/mujoco_api_reference.md` - Technical API reference

**Generated MuJoCo Models:**

- `mujoco_models/openarm_v10.xml` - Arm only (7 DOF, fixed-base)
- `mujoco_models/openarm_v10_hand.xml` - Arm + Hand (9 DOF, fixed-base)
- `mujoco_models/openarm_v10_bimanual.xml` - Bimanual (14 DOF, fixed-base)
- `mujoco_models/openarm_v10_bimanual_hand.xml` - Bimanual + Hands (18 DOF, fixed-base)
- `mujoco_models/meshes/` - Copied mesh files

The implementation supports position, velocity, and effort control modes for single arm, bimanual, and hand configurations. All existing URDF requirements for MuJoCo are satisfied (joint limits, inertial properties, mesh compatibility).

## Quick Start

### 1. Install Dependencies

```bash
# Clone and build custom mujoco_ros2_control fork (required for dynamic switching)
cd ~/ros2_ws/src
git clone https://github.com/zordicom/mujoco_ros2_control.git
cd mujoco_ros2_control
git checkout 2025-11-control-interface

# Build
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select mujoco_ros2_control --symlink-install
source ~/ros2_ws/install/setup.bash

# Install MuJoCo Python and urdf2mjcf converter
pip install mujoco urdf2mjcf
```

**Note:** The custom fork (commit `5a1e443`) is **required** for dynamic controller switching. The standard `mujoco_ros2_control` package has conflicts when all three command interfaces are exposed simultaneously.

### 2. Generate MuJoCo Models

```bash
cd ~/ros2_ws/src/openarm_description

# Generate arm only model
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml

# (Optional) Generate arm + hand model
python3 scripts/urdf_to_mjcf.py --arm-type v10 --hand --output mujoco_models/openarm_v10_hand.xml
```

**Expected Output (arm only):**

```
✓ Successfully converted to MJCF
✓ Fixed mesh paths in MJCF
✓ MJCF is valid
  - DOF: 7
  - Bodies: 9
  - Joints: 7
  - Actuators: 7
```

**Expected Output (arm + hand):**

```
✓ Successfully converted to MJCF
✓ Fixed mesh paths in MJCF
✓ MJCF is valid
  - DOF: 9
  - Bodies: 11
  - Joints: 9
  - Actuators: 9
```

### 2b. Test in MuJoCo Viewer (Optional)

```bash
# Option 1: Using simulate binary (recommended, better performance)
simulate mujoco_models/openarm_v10.xml

# Option 2: Using Python viewer module
python3 -m mujoco.viewer --mjcf=mujoco_models/openarm_v10.xml
```

The `simulate` binary is the official MuJoCo viewer with better performance and features. It's typically installed automatically when you run `pip install mujoco` and will be located at `~/.local/bin/simulate`.

**Verify installation:**

```bash
which simulate  # Should show: ~/.local/bin/simulate
```

If `simulate` is not found, install it manually:

```bash
# Download pre-built MuJoCo release (includes simulate binary)
cd ~/Downloads
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.3/mujoco-3.3.3-linux-x86_64.tar.gz
tar -xzf mujoco-3.3.3-linux-x86_64.tar.gz
cp mujoco-3.3.3/bin/simulate ~/.local/bin/
```

This lets you visualize and manually control the robot before ROS2 integration.

### 3. Validate Setup

Run the test suite to verify your MuJoCo integration:

```bash
# Test single arm setup (default)
python3 scripts/test_mujoco_setup.py

# Test arm + hand configuration
python3 scripts/test_mujoco_setup.py --hand

# Test bimanual configuration
python3 scripts/test_mujoco_setup.py --bimanual

# Test bimanual + hands configuration
python3 scripts/test_mujoco_setup.py --bimanual --hand
```

The test validates:

- ✅ URDF generation with MuJoCo settings
- ✅ ros2_control tags and MuJoCo plugin configuration
- ✅ All required configuration files exist
- ✅ MuJoCo model can be loaded (if generated)

### 4. Launch Simulation

```bash
# Arm only
ros2 launch openarm_description mujoco_sim.launch.py

# Arm + Hand
ros2 launch openarm_description mujoco_sim.launch.py hand:=true

# Velocity control
ros2 launch openarm_description mujoco_sim.launch.py control_mode:=velocity

# Bimanual
ros2 launch openarm_description mujoco_sim.launch.py bimanual:=true
```

### 5. Test Control

```bash
python3 scripts/example_position_control.py
```

## File Structure

```
openarm_description/
├── config/mujoco/
│   ├── v10_sim_params.yaml          # Simulation parameters
│   ├── controllers_position.yaml    # Position control config
│   ├── controllers_velocity.yaml    # Velocity control config
│   └── controllers_effort.yaml      # Effort control config
├── docs/
│   ├── mujoco_usage.md              # User guide
│   ├── mujoco_api_reference.md      # API reference
│   └── mujoco_support_plan.md       # This document
├── launch/
│   └── mujoco_sim.launch.py         # Simulation launch file
├── mujoco_models/
│   └── openarm_v10.xml              # Generated MuJoCo model
├── rviz/
│   └── mujoco_view.rviz             # RViz configuration
├── scripts/
│   ├── urdf_to_mjcf.py              # URDF converter
│   ├── test_mujoco_setup.py         # Test suite
│   └── example_position_control.py  # Example script
└── urdf/
    ├── robot/
    │   ├── v10.urdf.xacro           # Main robot xacro (modified)
    │   └── openarm_robot.xacro      # Robot macro (modified)
    └── ros2_control/
        ├── openarm.ros2_control.xacro         # Single arm (modified)
        └── openarm.bimanual.ros2_control.xacro # Bimanual (modified)
```

## Dependencies

### Required ROS2 Packages

- `mujoco_ros2_control` - MuJoCo hardware interface plugin (see Custom Fork section)
- `ros2_control` - ROS2 control framework
- `ros2_controllers` - Standard controllers
- `joint_trajectory_controller` - Trajectory execution
- `velocity_controllers` - Velocity control
- `effort_controllers` - Effort control
- `joint_state_broadcaster` - State publishing

### Python Packages

- `mujoco` (3.0.0+) - MuJoCo physics simulator
- `urdf2mjcf` - URDF to MJCF conversion utility

### System Requirements

- MuJoCo library (version 2.3.0+)
- Python 3.10+
- ROS2 Humble or later

## Custom mujoco_ros2_control Fork

This integration uses a **custom fork** of `mujoco_ros2_control` with dynamic controller switching support:

**Repository:** <https://github.com/zordicom/mujoco_ros2_control>
**Branch:** `2025-11-control-interface`
**Commit:** `5a1e443`

### What Was Changed

The standard `mujoco_ros2_control` package has a limitation: when position, velocity, and effort command interfaces are all exposed in the URDF (as required by the OpenARM configuration), they conflict. Position control directly sets joint positions (`qpos`), which overrides effort control that tries to apply torques (`qfrc_applied`).

**Our Solution:**

1. **Active Command Detection** - Track initial command values for each interface and detect when they change (indicating an active controller)
2. **Selective Control Application** - Only apply control from interfaces that are actively being commanded
3. **Dynamic Switching** - Enable seamless runtime switching between position, velocity, and effort controllers without URDF changes

**Modified Files:**

- `mujoco_system.hpp` - Added `control_mode_` parameter and `*_command_active` flags to `JointState` struct
- `mujoco_system.cpp` - Implemented active command detection logic in `write()` function
- `package.xml` - Export plugin configuration

**Key Innovation:**
The hardware interface now operates in "all" mode by default, exposing all three command interfaces. It detects which controller is actively sending commands by monitoring if command values deviate from their initial state. This allows all three control modes to coexist without conflict.

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/zordicom/mujoco_ros2_control.git
cd mujoco_ros2_control
git checkout 2025-11-control-interface

# Build
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --packages-select mujoco_ros2_control --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Launch File (`mujoco_sim.launch.py`)

The `launch/mujoco_sim.launch.py` file is the main entry point for running OpenARM in MuJoCo simulation with ROS2 control. It orchestrates the entire simulation setup and controller loading.

### What It Does

**1. Configurable Launch Arguments:**

- `arm_type` - Robot type (default: v10)
- `control_mode` - Position/velocity/effort control (default: position)
- `hand` - Include hand/gripper (default: false)
- `bimanual` - Bimanual configuration (default: false)
- `use_rviz` - Launch RViz visualization (default: true)
- `use_sim_time` - Use simulation time (default: true)

**2. Robot Description Generation:**
Dynamically generates URDF using xacro with:

- `ros2_control:=true` - Enables ros2_control framework
- `use_mujoco:=true` - Loads MuJoCo hardware interface plugin
- `hand:=<value>` - Includes hand/gripper if specified
- `bimanual:=<value>` - Sets up bimanual configuration if specified

**3. MuJoCo Model Selection:**
Automatically selects the correct MuJoCo model based on configuration:

- Single arm: `mujoco_models/openarm_v10.xml`
- Arm + hand: `mujoco_models/openarm_v10_hand.xml`
- Bimanual: `mujoco_models/openarm_v10_bimanual.xml`
- Bimanual + hands: `mujoco_models/openarm_v10_bimanual_hand.xml`

Checks both install and source directories, provides helpful error if model not found.

**4. Node Startup Sequence:**

- **Robot State Publisher** - Publishes robot TF transforms from URDF
- **Static Transform Publisher** - Publishes `world` → `openarm_link0` transform for RViz
- **MuJoCo ROS2 Control Node** - Interfaces between MuJoCo physics and ros2_control
  - Uses default node name for proper parameter scoping
  - Dynamically selects controller config based on `control_mode`
  - Loads appropriate MuJoCo model
- **RViz** - Optional visualization (if `use_rviz:=true`)

**5. Controller Loading (Sequential):**

1. **Joint State Broadcaster** loads when robot state publisher starts
   - Uses `--controller-manager /controller_manager` for proper service routing
2. **Selected Controller** loads after broadcaster completes:
   - Position mode → `joint_trajectory_controller`
   - Velocity mode → `velocity_controller`
   - Effort mode → `effort_controller`

### Launch Workflow

```
Launch Arguments
       ↓
Generate URDF (xacro with use_mujoco:=true)
       ↓
Start Robot State Publisher + MuJoCo Node + RViz
       ↓
Wait for startup → Load Joint State Broadcaster
       ↓
Wait for broadcaster → Load Selected Controller (position/velocity/effort)
       ↓
Ready for commands!
```

### Dynamic Configuration

The launch file uses `OpaqueFunction` to dynamically select configurations at runtime:

- **MuJoCo model** selected based on `hand` and `bimanual` parameters
- **Controller config** selected based on `control_mode` parameter
- **Controller loader** selected to match the control mode
- This allows a single launch file to support all control modes and configurations

### Important Implementation Details

**Parameter Scoping:**

- The MuJoCo node uses its default name (`mujoco_ros2_control_node`)
- The controller_manager is created as a child node at `/controller_manager`
- Controller configuration YAML uses simple `controller_manager:` namespace (no node prefix)
- Parameter order matters: `[robot_description, config_file, additional_params]`

**RViz Visualization:**

- Requires `world` frame as the fixed frame reference
- Static transform publisher creates `world` → `openarm_link0` transform
- Without this, RViz shows "No transform from [link] to [world]" errors

**Controller Configuration Files:**

- Must use `ros__parameters: {}` for empty parameter sections
- Cannot use only comments after `ros__parameters:` (causes parse errors)
- YAML structure must match standard ros2_control patterns

## Control Modes

### Position Control

- **Controller:** `joint_trajectory_controller/JointTrajectoryController`
- **Use Case:** Trajectory following, motion planning, pick-and-place
- **Topic:** `/joint_trajectory_controller/follow_joint_trajectory` (Action)
- **Config:** `config/mujoco/controllers_position.yaml`

### Velocity Control

- **Controller:** `velocity_controllers/JointGroupVelocityController`
- **Use Case:** Teleoperation, Cartesian velocity control, reactive behaviors
- **Topic:** `/velocity_controller/commands` (Float64MultiArray)
- **Config:** `config/mujoco/controllers_velocity.yaml`

### Effort Control

- **Controller:** `effort_controllers/JointGroupEffortController`
- **Use Case:** Force control, impedance control, learning-based control, gravity compensation
- **Topic:** `/effort_controller/commands` (Float64MultiArray)
- **Config:** `config/mujoco/controllers_effort.yaml`

## URDF to MJCF Conversion

The `scripts/urdf_to_mjcf.py` script automates the conversion process:

**Features:**

- Uses `urdf2mjcf` Python API for robust conversion
- Automatically converts `package://` URIs to absolute paths
- Pre-injects `balanceinertia="true"` for hand/gripper inertia issues
- Copies mesh files to output directory with correct structure
- Fixes mesh path references in MJCF for MuJoCo compatibility
- Removes freejoint for fixed-base robots (prevents falling)
- Validates generated MJCF by loading in MuJoCo
- Falls back to direct MuJoCo loading if urdf2mjcf fails

**Usage:**

```bash
# Arm only (7 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml

# Arm + Hand (9 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --hand --output mujoco_models/openarm_v10_hand.xml

# Bimanual (14 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --bimanual --output mujoco_models/openarm_v10_bimanual.xml

# Bimanual + Hands (18 DOF)
python3 scripts/urdf_to_mjcf.py --arm-type v10 --bimanual --hand --output mujoco_models/openarm_v10_bimanual_hand.xml
```

**Output:**

- `mujoco_models/openarm_v10*.xml` - MuJoCo model (fixed-base)
- `mujoco_models/meshes/` - Copied mesh files

**Validation:**
The script automatically validates the generated model by loading it in MuJoCo and reports:

- Number of degrees of freedom (DOF)
- Number of bodies, joints, and actuators
- Joint names and structure

**Inertia Balancing:**
The script automatically handles inertia matrix violations common in hand/gripper models by pre-injecting `<mujoco><compiler balanceinertia="true"/></mujoco>` into the URDF before conversion. This allows MuJoCo to automatically adjust inertia matrices to satisfy physical constraints (A + B >= C).

## Simulation Parameters

Tunable parameters in `config/mujoco/v10_sim_params.yaml`:

- **Solver Settings:** Timestep, iterations, tolerance
- **Joint Damping:** Viscous damping coefficients per joint
- **Joint Friction:** Armature inertia and friction loss
- **Contact Parameters:** Collision and contact settings
- **Performance:** Multi-threading and optimization options

## Troubleshooting

### Common Issues and Solutions

**1. "controller manager doesn't have an update_rate parameter"**

- **Cause:** Controller config YAML file not loaded correctly
- **Solution:** Ensure config file is loaded before additional parameters dict in Node `parameters` list
- **Correct order:** `[robot_description, config_file, {additional_params}]`

**2. "The 'type' param was not defined for controller"**

- **Cause:** Controller configuration not found or YAML has invalid structure
- **Solution:**
  - Use `ros__parameters: {}` for empty sections, not just comments
  - Check YAML file syntax is valid
  - Ensure `controller_manager` section includes controller type definitions

**3. "No transform from [link] to [world]" in RViz**

- **Cause:** RViz fixed frame expects `world` but robot starts at `openarm_link0`
- **Solution:** Static transform publisher added to launch file (already included)

**4. "Could not contact service /controller_manager/load_controller"**

- **Cause:** Controller manager services at wrong namespace
- **Solution:** Use `--controller-manager /controller_manager` in controller loading commands

**5. Robot appears white in RViz**

- **Cause:** No transforms published or joint states not available
- **Solution:** Check that `joint_state_broadcaster` is active with:

  ```bash
  ros2 control list_controllers --controller-manager /controller_manager
  ```

**6. MuJoCo model file not found**

- **Cause:** Model not generated or launch looking in wrong directory
- **Solution:** Run `urdf_to_mjcf.py` script first, launch file checks both install and source dirs

## Testing Dynamic Controller Switching

The custom `mujoco_ros2_control` fork enables **dynamic switching** between position, velocity, and effort controllers at runtime. Here's how to test this feature:

### Prerequisites

Ensure you have the custom fork installed and the xacro files properly configured:

**1. URDF Configuration (Already Done)**

The xacro files are already configured with `control_mode="all"` hardcoded in:

- `urdf/ros2_control/openarm.ros2_control.xacro`
- `urdf/ros2_control/openarm.bimanual.ros2_control.xacro`

This ensures all three command interfaces are exposed simultaneously:

```xml
<xacro:if value="${use_mujoco}">
  <plugin>mujoco_ros2_control/MujocoSystem</plugin>
  <param name="control_mode">all</param>
</xacro:if>
```

**2. Launch File Configuration (Already Done)**

The `mujoco_sim.launch.py`:

- Loads **all three controller configuration YAMLs** into the hardware interface parameters
- **Loads all three controllers as `inactive`** (position, velocity, effort)
- **Activates only the controller specified by `control_mode`** parameter

This allows dynamic switching without restarting the simulation. All three controllers are always loaded and available for switching.

### Test Procedure

**Step 1: Launch Simulation**

```bash
# Launch with default position control
ros2 launch openarm_description mujoco_sim.launch.py

# Or launch with effort control initially
ros2 launch openarm_description mujoco_sim.launch.py control_mode:=effort
```

The `control_mode` parameter only determines which controller starts **active**. All controllers are loaded and available for switching.

**Step 2: Verify All Controllers Are Loaded**

```bash
ros2 control list_controllers --controller-manager /controller_manager
```

Expected output:

```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] inactive
velocity_controller[velocity_controllers/JointGroupVelocityController] inactive
effort_controller[effort_controllers/JointGroupEffortController] active
```

(The exact active/inactive status depends on `control_mode` parameter)

**Step 3: Test Position Control**

```bash
# Switch to position control
ros2 control switch_controllers \
  --activate joint_trajectory_controller \
  --deactivate effort_controller velocity_controller \
  --controller-manager /controller_manager

# Send position command (example script)
python3 scripts/example_position_control.py
```

**Step 4: Switch to Effort Control**

```bash
# Switch to effort control
ros2 control switch_controllers \
  --activate effort_controller \
  --deactivate joint_trajectory_controller velocity_controller \
  --controller-manager /controller_manager

# Send effort command (torques in Nm)
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [20.0, 20.0, 15.0, 15.0, 3.0, 3.0, 3.0]" -r 100
```

**Step 5: Switch to Velocity Control**

```bash
# Switch to velocity control
ros2 control switch_controllers \
  --activate velocity_controller \
  --deactivate joint_trajectory_controller effort_controller \
  --controller-manager /controller_manager

# Send velocity command (rad/s)
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0]" -r 10
```

### Verification

**Check Movement in MuJoCo Viewer:**

- The robot should respond to commands from the active controller
- Switching should be seamless (no crashes or jerky motions)

**Monitor Joint States:**

```bash
ros2 topic echo /joint_states
```

**Check for Conflicts:**
With the custom fork, you should **NOT** see:

- Robot stuck at zero position when sending effort commands
- Effort control being overridden by position control
- Controllers interfering with each other

### Common Test Commands

**Quick switch test (position → effort → position):**

```bash
# Start at position
ros2 control switch_controllers --activate joint_trajectory_controller --deactivate effort_controller velocity_controller --controller-manager /controller_manager

# Switch to effort
sleep 2
ros2 control switch_controllers --activate effort_controller --deactivate joint_trajectory_controller velocity_controller --controller-manager /controller_manager

# Back to position
sleep 2
ros2 control switch_controllers --activate joint_trajectory_controller --deactivate effort_controller velocity_controller --controller-manager /controller_manager
```

**Effort control stress test (max torques within limits):**

```bash
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [35.0, 35.0, 25.0, 25.0, 6.0, 6.0, 6.0]" -r 100
```

Joint torque limits (from MuJoCo XML):

- Joints 1-2: ±40 Nm
- Joints 3-4: ±27 Nm
- Joints 5-7: ±7 Nm

### What Makes This Work

**Without the custom fork:**

- All command interfaces active simultaneously
- Position control sets `qpos` to 0 (default)
- Effort control tries to apply torque via `qfrc_applied`
- Position control wins → robot stuck at zero

**With the custom fork:**

- Detects which interface has commands that deviate from initial values
- Only applies control from actively commanded interfaces
- Inactive controllers don't interfere
- Seamless switching without conflicts

## Known Limitations

1. **Manual MJCF Generation:** MuJoCo model must be generated before first use (automated generation during launch is a future enhancement)
2. **Sensor Support:** IMU and F/T sensors not yet fully integrated
3. **Contact Simulation:** Basic contact support, advanced manipulation requires tuning
4. **Fixed Base Only:** Current implementation assumes fixed-base robots (freejoint removed from MJCF)

## Future Enhancements

1. **Automated Model Generation:** Generate MJCF on-demand during launch
2. **Advanced Contact Modeling:** Enhanced grasping and manipulation simulation
3. **Full Sensor Integration:** IMU, force/torque, and tactile sensors
4. **MoveIt2 Deep Integration:** Optimized motion planning with MuJoCo
5. **System Identification Tools:** Parameter estimation from simulation
6. **Real-time Visualization:** Improved MuJoCo viewer integration

## Success Criteria

All success criteria have been met:

- ✅ OpenARM loads and runs in MuJoCo simulator
- ✅ All three control modes (position, velocity, effort) functional
- ✅ **Dynamic controller switching working without conflicts**
- ✅ Joint limits and safety constraints enforced
- ✅ Real-time factor > 1.0 (faster than real-time)
- ✅ Controllers stable without oscillations
- ✅ Hand/gripper support working
- ✅ Complete documentation provided
- ✅ Example code and tests included

## Additional Resources

- **User Documentation:** `docs/mujoco_usage.md`
- **API Reference:** `docs/mujoco_api_reference.md`
- **MuJoCo Documentation:** <https://mujoco.readthedocs.io/>
- **Custom mujoco_ros2_control Fork:** <https://github.com/zordicom/mujoco_ros2_control> (branch: `2025-11-control-interface`, commit: `5a1e443`)
- **Original mujoco_ros2_control:** <https://github.com/moveit/mujoco_ros2_control>
- **ros2_control:** <https://control.ros.org/>

## Support

For issues, questions, or contributions:

1. Check `docs/mujoco_usage.md` for troubleshooting
2. Run `scripts/test_mujoco_setup.py` to validate setup
3. Review example scripts in `scripts/`
4. Open an issue on GitHub

---

**Status:** ✅ Implementation Complete

**Last Updated:** 2025-11-07
