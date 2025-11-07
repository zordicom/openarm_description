# MuJoCo Integration API Reference

## Overview

This document provides a technical reference for the MuJoCo integration with OpenARM.

## Xacro Parameters

### `v10.urdf.xacro` Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `arm_type` | string | `v10` | ARM variant identifier |
| `body_type` | string | `v10` | Body variant identifier |
| `hand` | bool | `true` | Include hand/gripper |
| `ee_type` | string | `openarm_hand` | End-effector type |
| `ros2_control` | bool | `false` | Enable ros2_control |
| `use_fake_hardware` | bool | `false` | Use mock hardware interface |
| `use_mujoco` | bool | `false` | Use MuJoCo physics simulation |
| `bimanual` | bool | `false` | Bimanual configuration |
| `can_interface` | string | `can0` | CAN interface name |
| `left_can_interface` | string | `can1` | Left arm CAN interface |
| `right_can_interface` | string | `can0` | Right arm CAN interface |

### ros2_control Xacro Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `arm_type` | string | - | ARM variant (required) |
| `arm_prefix` | string | `''` | Joint name prefix |
| `can_interface` | string | - | CAN interface for real hardware |
| `use_fake_hardware` | bool | `false` | Enable fake hardware |
| `use_mujoco` | bool | `false` | Enable MuJoCo simulation |
| `fake_sensor_commands` | bool | `false` | Create command interfaces for sensors |
| `hand` | bool | `false` | Include hand joints |
| `bimanual` | bool | `false` | Bimanual mode |

## Launch File Arguments

### `mujoco_sim.launch.py`

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `arm_type` | string | `v10` | ARM type |
| `control_mode` | string | `position` | Control mode (`position`, `velocity`, `effort`) |
| `hand` | bool | `false` | Include hand |
| `bimanual` | bool | `false` | Bimanual configuration |
| `use_rviz` | bool | `true` | Launch RViz |
| `rviz_config` | string | (auto) | Path to RViz config |
| `mujoco_model_path` | string | (auto) | Path to MuJoCo XML |
| `use_sim_time` | bool | `true` | Use simulation time |

## Controller Configuration

### Position Control (`controllers_position.yaml`)

**Controller:** `joint_trajectory_controller/JointTrajectoryController`

**Topics:**

- Action: `/joint_trajectory_controller/follow_joint_trajectory`
- State: `/joint_states`

**Parameters:**

```yaml
joints: [openarm_joint1, ..., openarm_joint7]
command_interfaces: [position]
state_interfaces: [position, velocity]
allow_partial_joints_goal: false
open_loop_control: false
```

### Velocity Control (`controllers_velocity.yaml`)

**Controller:** `velocity_controllers/JointGroupVelocityController`

**Topics:**

- Command: `/velocity_controller/commands` (`std_msgs/Float64MultiArray`)
- State: `/joint_states`

**Parameters:**

```yaml
joints: [openarm_joint1, ..., openarm_joint7]
interface_name: velocity
```

### Effort Control (`controllers_effort.yaml`)

**Controller:** `effort_controllers/JointGroupEffortController`

**Topics:**

- Command: `/effort_controller/commands` (`std_msgs/Float64MultiArray`)
- State: `/joint_states`

**Parameters:**

```yaml
joints: [openarm_joint1, ..., openarm_joint7]
interface_name: effort
```

## Simulation Parameters (`v10_sim_params.yaml`)

### Solver Settings

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `solver.timestep` | float | `0.001` | Integration timestep (s) |
| `solver.iterations` | int | `100` | Solver iterations |
| `solver.tolerance` | float | `1.0e-6` | Constraint tolerance |

### Joint Damping

| Joint | Default Damping (N⋅m⋅s/rad) |
|-------|----------------------------|
| joint1 | 0.5 |
| joint2 | 0.5 |
| joint3 | 0.3 |
| joint4 | 0.3 |
| joint5 | 0.1 |
| joint6 | 0.1 |
| joint7 | 0.1 |

### Joint Friction

| Joint | Armature (kg⋅m²) | Friction Loss (N⋅m) |
|-------|------------------|---------------------|
| joint1 | 0.01 | 0.1 |
| joint2 | 0.01 | 0.1 |
| joint3 | 0.005 | 0.05 |
| joint4 | 0.005 | 0.05 |
| joint5 | 0.002 | 0.02 |
| joint6 | 0.002 | 0.02 |
| joint7 | 0.002 | 0.02 |

## Scripts

### `urdf_to_mjcf.py`

Convert URDF to MuJoCo MJCF format.

**Usage:**

```bash
python3 urdf_to_mjcf.py --arm-type v10 --output model.xml [OPTIONS]
```

**Arguments:**

- `--arm-type`: ARM type (default: v10)
- `--bimanual`: Generate bimanual config
- `--hand`: Include hand
- `--output`, `-o`: Output MJCF file (required)
- `--package-path`: Package path (auto-detect)
- `--sim-params`: Simulation parameters YAML
- `--validate-only`: Only validate existing MJCF

**Returns:** Exit code 0 on success, 1 on failure

### `test_mujoco_setup.py`

Validate MuJoCo setup.

**Usage:**

```bash
python3 test_mujoco_setup.py [OPTIONS]
```

**Arguments:**

- `--bimanual`: Test bimanual configuration
- `--hand`: Include hand in tests
- `--package-path`: Package path (auto-detect)

**Tests:**

1. URDF generation
2. ros2_control tags validation
3. Configuration files existence
4. MuJoCo model loading

**Returns:** Exit code 0 if all tests pass, 1 otherwise

### `example_position_control.py`

Example position control demonstration.

**Usage:**

```bash
# Terminal 1
ros2 launch openarm_description mujoco_sim.launch.py

# Terminal 2
python3 example_position_control.py
```

**Behavior:**

1. Moves to home position (all zeros)
2. Moves to configuration 1
3. Moves to configuration 2
4. Returns to home

## ROS2 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Joint position, velocity, effort |
| `/robot_description` | `std_msgs/String` | Robot URDF |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |
| `/tf_static` | `tf2_msgs/TFMessage` | Static transforms |

### Subscribed Topics

#### Position Control Mode

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Joint trajectory action |

#### Velocity Control Mode

| Topic | Type | Description |
|-------|------|-------------|
| `/velocity_controller/commands` | `std_msgs/Float64MultiArray` | Velocity commands (rad/s) |

#### Effort Control Mode

| Topic | Type | Description |
|-------|------|-------------|
| `/effort_controller/commands` | `std_msgs/Float64MultiArray` | Torque commands (N⋅m) |

## ROS2 Services

| Service | Type | Description |
|---------|------|-------------|
| `/controller_manager/list_controllers` | `controller_manager_msgs/ListControllers` | List all controllers |
| `/controller_manager/switch_controller` | `controller_manager_msgs/SwitchController` | Switch active controllers |
| `/controller_manager/load_controller` | `controller_manager_msgs/LoadController` | Load a controller |
| `/controller_manager/unload_controller` | `controller_manager_msgs/UnloadController` | Unload a controller |

## Joint Limits

| Joint | Position (rad) | Velocity (rad/s) | Effort (N⋅m) |
|-------|----------------|------------------|--------------|
| joint1 | -1.396 to 3.491 | 16.755 | 40 |
| joint2 | -1.745 to 1.745 | 16.755 | 40 |
| joint3 | -1.571 to 1.571 | 5.445 | 27 |
| joint4 | 0.0 to 2.443 | 5.445 | 27 |
| joint5 | -1.571 to 1.571 | 20.944 | 7 |
| joint6 | -0.785 to 0.785 | 20.944 | 7 |
| joint7 | -1.571 to 1.571 | 20.944 | 7 |

## Error Codes

### URDF Generation Errors

| Code | Description | Solution |
|------|-------------|----------|
| E001 | Xacro file not found | Check file path |
| E002 | Xacro processing failed | Check xacro syntax |
| E003 | Invalid parameters | Check parameter values |

### MuJoCo Conversion Errors

| Code | Description | Solution |
|------|-------------|----------|
| E101 | URDF parse error | Validate URDF with `check_urdf` |
| E102 | MJCF compilation failed | Check joint limits and inertials |
| E103 | Model validation failed | Check MuJoCo error message |

### Runtime Errors

| Code | Description | Solution |
|------|-------------|----------|
| E201 | Action server not available | Check controller is loaded |
| E202 | Controller failed to load | Check configuration file |
| E203 | Joint limits exceeded | Adjust trajectory |

## Plugin Architecture

The MuJoCo integration uses the pluginlib architecture:

```
ros2_control
    ↓
MujocoSystemInterface (base class)
    ↓
MujocoSystem (implementation)
    ↓
MuJoCo C API
```

**Plugin Name:** `mujoco_ros2_control/MujocoSystem`

**Base Class:** `hardware_interface::SystemInterface`

## File Structure

```
openarm_description/
├── config/
│   └── mujoco/
│       ├── v10_sim_params.yaml
│       ├── controllers_position.yaml
│       ├── controllers_velocity.yaml
│       └── controllers_effort.yaml
├── launch/
│   └── mujoco_sim.launch.py
├── mujoco_models/
│   └── openarm_v10.xml (generated)
├── rviz/
│   └── mujoco_view.rviz
├── scripts/
│   ├── urdf_to_mjcf.py
│   ├── test_mujoco_setup.py
│   └── example_position_control.py
└── urdf/
    └── ros2_control/
        ├── openarm.ros2_control.xacro
        └── openarm.bimanual.ros2_control.xacro
```

## Version Compatibility

| Component | Minimum Version | Tested Version |
|-----------|----------------|----------------|
| ROS2 | Humble | Humble, Iron |
| MuJoCo | 2.3.0 | 3.1.0 |
| Python | 3.10 | 3.10, 3.11 |
| mujoco_ros2_control | 0.1.0 | 0.1.0 |

---

Copyright 2025 Zordi, Inc. All rights reserved.
