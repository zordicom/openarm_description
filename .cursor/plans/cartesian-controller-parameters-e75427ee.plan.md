<!-- e75427ee-08f6-49cb-8019-1eb5119e65d5 c9715c7c-526a-43aa-ac90-cf39344d56b6 -->
# Expose Cartesian Controller Parameters

## Overview

Add `kp_cartesian`, `lookahead_cartesian`, and related limits as configurable ROS parameters under a `hardware_pd` group (these apply when `compute_pd_internally: false`, i.e., MIT mode).

## Part 1: Add "(MIT mode)" Comments

For any `hardware_pd` references that don't already have "(MIT mode)" clarification, add it. This ensures consistent terminology across the codebase.

Example: `hardware_pd` -> `hardware_pd (MIT mode)` in comments/descriptions.

## Part 2: Add Parameters

### 1. [zordi_cartesian_controller_parameters.yaml](zordi_ros_controllers/zordi_ros_controllers/config/zordi_cartesian_controller_parameters.yaml)

Add new parameters under `hardware_pd` group:

```yaml
# Hardware PD (MIT mode) parameters - used when compute_pd_internally: false
hardware_pd:
  kp_cartesian: {
    type: double,
    default_value: 25.0,
    description: "Cartesian velocity gain (1/s). Controls convergence speed to target."
  }
  lookahead_cartesian: {
    type: double,
    default_value: 0.15,
    description: "Position target lookahead time (s). Sets how far ahead to place position command."
  }
  max_cartesian_velocity: {
    type: double,
    default_value: 3.0,
    description: "Maximum Cartesian linear velocity (m/s)"
  }
  max_cartesian_omega: {
    type: double,
    default_value: 6.0,
    description: "Maximum Cartesian angular velocity (rad/s)"
  }
  max_joint_velocity: {
    type: double,
    default_value: 5.0,
    description: "Maximum joint velocity (rad/s)"
  }
  max_position_offset: {
    type: double,
    default_value: 0.3,
    description: "Maximum position offset per joint (rad)"
  }
```

### 2. [zordi_cartesian_controller.cpp](zordi_ros_controllers/zordi_ros_controllers/src/zordi_cartesian_controller.cpp)

Replace hardcoded constants (~lines 840-875) with `params_.hardware_pd.*`:

```cpp
// Before (hardcoded):
const double kp_cartesian = 25.0;
const double lookahead_cartesian = 0.15;
const double max_cart_vel = 3.0;
// ...

// After (from params):
const double kp_cartesian = params_.hardware_pd.kp_cartesian;
const double lookahead_cartesian = params_.hardware_pd.lookahead_cartesian;
const double max_cart_vel = params_.hardware_pd.max_cartesian_velocity;
// ...
```

### 3. [test_controllers.yaml](openarm_tests/test/config/test_controllers.yaml)

Add explicit values to test config (optional but makes tests self-documenting):

```yaml
zordi_cartesian_mit_controller:
  ros__parameters:
    hardware_pd:
      kp_cartesian: 25.0
      lookahead_cartesian: 0.15
```

## Build and Test

```bash
colcon build --packages-select zordi_ros_controllers
launch_test src/openarm_tests/test/test_cartesian_control.test.py
```

### To-dos

- [ ] Add hardware_pd parameters to zordi_cartesian_controller_parameters.yaml
- [ ] Replace hardcoded constants with params_.hardware_pd.* in C++
- [ ] Add explicit hardware_pd parameter values to test_controllers.yaml
- [ ] Rebuild and run test_cartesian_control.test.py to verify