# Plan: Stable Initial Pose Configuration

**Copyright 2025 Zordi, Inc. All rights reserved.**

## Objective

Enable starting simulation in stable configuration while keeping URDF canonical (q=0). Add "jump to pose" capability for MuJoCo simulation.

## Current Problem

- URDF specifies q=0 (upright/unstable equilibrium)
- Controllers oscillate/saturate trying to hold unstable pose
- Need to start at stable hanging pose without modifying canonical URDF

## Solution Architecture

### 1. Config File for Stable Poses

**Create: `config/mujoco/initial_poses.yaml`**

```yaml
# Stable initial configurations for MuJoCo simulation
# URDF remains canonical (q=0), these override at simulation startup

poses:
  # Stable hanging pose (default)
  stable_hanging:
    joint1: 0.0
    joint2: -0.785   # -45° (shoulder down)
    joint3: 0.0
    joint4: 1.57     # 90° (elbow bent)
    joint5: 0.0
    joint6: 0.0
    joint7: 0.0
    description: "Arm hanging down, stable equilibrium"

  # Canonical pose (for reference/testing)
  canonical:
    joint1: 0.0
    joint2: 0.0
    joint3: 0.0
    joint4: 0.0
    joint5: 0.0
    joint6: 0.0
    joint7: 0.0
    description: "URDF canonical, unstable for control testing"

  # Home pose (common robot home)
  home:
    joint1: 0.0
    joint2: -0.5
    joint3: 0.0
    joint4: 1.2
    joint5: 0.0
    joint6: 0.0
    joint7: 0.0
    description: "Common robot home position"
```

### 2. Launch File Parameter

**Modify: `launch/single_arm.launch.py`**

Add parameter:

```python
DeclareLaunchArgument(
    "initial_pose",
    default_value="stable_hanging",
    choices=["stable_hanging", "canonical", "home"],
    description="Initial pose from config/mujoco/initial_poses.yaml"
)
```

Pass to MuJoCo node:

```python
parameters=[
    robot_description,
    controllers_config,
    {
        "mujoco_model_path": model_path,
        "initial_pose": initial_pose,  # ← New
        "initial_pose_config": os.path.join(
            pkg_dir, "config/mujoco/initial_poses.yaml"
        ),
    }
]
```

### 3. MuJoCo System Initialization

**Modify: `mujoco_ros2_control/src/mujoco_system.cpp`**

**In `init_sim()` (after line 301):**

```cpp
bool MujocoSystem::init_sim(...)
{
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;
  logger_ = rclcpp::get_logger("mujoco_system");

  register_joints(urdf_model, hardware_info);
  register_sensors(urdf_model, hardware_info);

  // NEW: Load and apply initial pose override
  apply_initial_pose_override(hardware_info);  // ← Add this

  set_initial_pose();  // Applies joint_state.position to mj_data
  return true;
}
```

**New function:**

```cpp
void MujocoSystem::apply_initial_pose_override(
  const hardware_interface::HardwareInfo &hardware_info)
{
  // Check if initial pose override is specified
  auto pose_name_it = hardware_info.hardware_parameters.find("initial_pose");
  auto config_path_it = hardware_info.hardware_parameters.find("initial_pose_config");

  if (pose_name_it == hardware_info.hardware_parameters.end() ||
      config_path_it == hardware_info.hardware_parameters.end())
  {
    RCLCPP_INFO(logger_, "No initial pose override - using URDF defaults");
    return;
  }

  std::string pose_name = pose_name_it->second;
  std::string config_path = config_path_it->second;

  // Load YAML config
  YAML::Node config = YAML::LoadFile(config_path);

  if (!config["poses"] || !config["poses"][pose_name])
  {
    RCLCPP_ERROR(logger_, "Pose '%s' not found in %s",
                 pose_name.c_str(), config_path.c_str());
    return;
  }

  auto pose = config["poses"][pose_name];

  // Apply to joint states
  for (auto &joint_state : joint_states_)
  {
    // Extract joint number from name (e.g., "openarm_joint2" -> "joint2")
    std::string joint_key = joint_state.name.substr(joint_state.name.find("joint"));

    if (pose[joint_key])
    {
      double new_position = pose[joint_key].as<double>();
      joint_state.position = new_position;
      joint_state.position_command = new_position;

      RCLCPP_INFO(logger_, "  %s: %.3f rad (override from '%s' pose)",
                  joint_state.name.c_str(), new_position, pose_name.c_str());
    }
  }

  std::string description = pose["description"].as<std::string>("");
  RCLCPP_INFO(logger_, "Applied initial pose '%s': %s",
              pose_name.c_str(), description.c_str());
}
```

### 4. "Jump to Pose" Service

**Add to: `mujoco_ros2_control/src/mujoco_ros2_control.cpp`**

**New service definition: `mujoco_ros2_control_msgs/srv/JumpToPose.srv`**

```
# Request
string pose_name           # Name from initial_poses.yaml, or "custom"
float64[] joint_positions  # If pose_name=="custom", use these values
bool reset_velocities      # Set velocities to zero after jump
---
# Response
bool success
string message
```

**Implementation in MujocoRos2Control::init():**

```cpp
// Add service
jump_to_pose_srv_ = node_->create_service<mujoco_ros2_control_msgs::srv::JumpToPose>(
  "jump_to_pose",
  std::bind(&MujocoRos2Control::handle_jump_to_pose, this,
            std::placeholders::_1, std::placeholders::_2));

RCLCPP_INFO(logger_, "Jump to pose service ready at '~/jump_to_pose'");
```

**Handler:**

```cpp
void MujocoRos2Control::handle_jump_to_pose(
  const std::shared_ptr<JumpToPose::Request> request,
  std::shared_ptr<JumpToPose::Response> response)
{
  std::lock_guard<std::mutex> lock(jump_pose_mutex_);

  if (request->pose_name == "custom")
  {
    if (request->joint_positions.size() != n_joints)
    {
      response->success = false;
      response->message = "Invalid joint_positions size";
      return;
    }
    pending_jump_positions_ = request->joint_positions;
  }
  else
  {
    // Load from config file
    pending_jump_positions_ = load_pose_from_config(request->pose_name);
    if (pending_jump_positions_.empty())
    {
      response->success = false;
      response->message = "Pose not found: " + request->pose_name;
      return;
    }
  }

  pending_jump_reset_vel_ = request->reset_velocities;
  pending_jump_active_ = true;

  response->success = true;
  response->message = "Jump scheduled for next physics step";
}
```

**In update() loop (before mj_step1):**

```cpp
void MujocoRos2Control::update()
{
  // Check for pending pose jump
  {
    std::lock_guard<std::mutex> lock(jump_pose_mutex_);
    if (pending_jump_active_)
    {
      // Apply to MuJoCo state
      for (size_t i = 0; i < pending_jump_positions_.size(); i++)
      {
        mj_data_->qpos[i] = pending_jump_positions_[i];
      }

      if (pending_jump_reset_vel_)
      {
        for (int i = 0; i < mj_model_->nv; i++)
        {
          mj_data_->qvel[i] = 0.0;
        }
      }

      // Reset forward kinematics
      mj_forward(mj_model_, mj_data_);

      RCLCPP_INFO(logger_, "Jumped to new pose");
      pending_jump_active_ = false;
    }
  }

  // Continue with normal physics step
  mj_step1(mj_model_, mj_data_);
  // ...
}
```

## Implementation Steps

### Step 1: Revert URDF to Canonical ✅

**File:** `urdf/ros2_control/openarm.ros2_control.xacro` (lines 82, 84)

Change back to:

```xml
initial_position="0.0"  (for joint2)
initial_position="0.0"  (for joint4)
```

### Step 2: Create Poses Config ✅

**File:** `config/mujoco/initial_poses.yaml` (new)

Create with stable_hanging, canonical, and home poses as shown above.

### Step 3: Update Launch File ✅

**File:** `launch/single_arm.launch.py`

- Add `initial_pose` parameter (default: "stable_hanging")
- Add `initial_pose_config` parameter (path to YAML)
- Pass both to mujoco_ros2_control node

### Step 4: Implement Pose Override in mujoco_system.cpp ✅

**Files:**

- `mujoco_ros2_control/src/mujoco_system.cpp`
  - Add `apply_initial_pose_override()` function (~50 lines)
  - Call from `init_sim()` after `register_joints()`
  - Requires: `#include <yaml-cpp/yaml.h>`

- `mujoco_ros2_control/include/mujoco_system.hpp`
  - Add function declaration

- `mujoco_ros2_control/CMakeLists.txt`
  - Add `yaml-cpp` dependency

### Step 5: Implement Jump Service ✅

**Files:**

- `mujoco_ros2_control_msgs/srv/JumpToPose.srv` (new)
  - Define service interface

- `mujoco_ros2_control/src/mujoco_ros2_control.cpp`
  - Add service in `init()` (~5 lines)
  - Add handler `handle_jump_to_pose()` (~40 lines)
  - Add mutex and state variables (~3 lines)
  - Apply jump in `update()` before mj_step1 (~20 lines)

- `mujoco_ros2_control/include/mujoco_ros2_control.hpp`
  - Add handler declaration
  - Add member variables (mutex, pending_jump state)

- `mujoco_ros2_control_msgs/CMakeLists.txt`
  - Add new service to rosidl_generate_interfaces

### Step 6: Regenerate XML Files ✅

```bash
python3 scripts/urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml
```

### Step 7: Rebuild ✅

```bash
colcon build --packages-select mujoco_ros2_control_msgs mujoco_ros2_control openarm_description
```

## Usage After Implementation

### Launch with Stable Pose

```bash
# Default: stable_hanging
ros2 launch openarm_description single_arm.launch.py

# Or specify
ros2 launch openarm_description single_arm.launch.py initial_pose:=home

# Use canonical (unstable, for testing)
ros2 launch openarm_description single_arm.launch.py initial_pose:=canonical
```

### Jump During Simulation

```bash
# Jump to named pose
ros2 service call /mujoco_ros2_control_node/jump_to_pose \
  mujoco_ros2_control_msgs/srv/JumpToPose \
  "{pose_name: 'home', reset_velocities: true}"

# Jump to custom position
ros2 service call /mujoco_ros2_control_node/jump_to_pose \
  mujoco_ros2_control_msgs/srv/JumpToPose \
  "{pose_name: 'custom', joint_positions: [0.0, -0.5, 0.0, 1.2, 0.0, 0.0, 0.0], reset_velocities: true}"
```

## Files to Modify/Create

1. **openarm_description/config/mujoco/initial_poses.yaml** - NEW
2. **openarm_description/launch/single_arm.launch.py** - MODIFY (~10 lines)
3. **openarm_description/urdf/ros2_control/openarm.ros2_control.xacro** - REVERT (lines 82, 84)
4. **mujoco_ros2_control_msgs/srv/JumpToPose.srv** - NEW
5. **mujoco_ros2_control_msgs/CMakeLists.txt** - MODIFY (1 line)
6. **mujoco_ros2_control/include/mujoco_ros2_control/mujoco_system.hpp** - MODIFY (~5 lines)
7. **mujoco_ros2_control/src/mujoco_system.cpp** - MODIFY (~70 lines)
8. **mujoco_ros2_control/include/mujoco_ros2_control/mujoco_ros2_control.hpp** - MODIFY (~10 lines)
9. **mujoco_ros2_control/src/mujoco_ros2_control.cpp** - MODIFY (~80 lines)
10. **mujoco_ros2_control/CMakeLists.txt** - MODIFY (add yaml-cpp dependency)
11. **mujoco_ros2_control/package.xml** - MODIFY (add yaml-cpp dependency)

## Benefits

- ✅ URDF remains canonical (q=0)
- ✅ Simulation can start in stable configuration
- ✅ Configurable via launch parameter
- ✅ Runtime pose jumping for testing/debugging
- ✅ Clean separation: URDF = canonical, config = simulation-specific

## Estimated Complexity

- New code: ~200 lines
- Modified code: ~30 lines
- New files: 2
- Modified files: 9
- Estimated time: 30-45 minutes

---

**Ready for implementation upon approval.**
