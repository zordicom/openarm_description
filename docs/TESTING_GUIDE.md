# OpenARM Robot Testing Guide

**Copyright 2025 Zordi, Inc. All rights reserved.**

Quick-start guide for testing the OpenARM 7-DOF robot with gravity compensation and multiple control modes.

---

## System Overview

### Packages in This System

| Package | Purpose | Provides |
|---------|---------|----------|
| **`openarm_description`** | Robot models and launch files | URDF/MJCF models, keyframes, launch files, configs |
| **`mujoco_ros2_control`** | MuJoCo hardware interface | Actuator-centric control, MIT mode implementation |
| **`zordi_ros_controllers`** | MIT controller with gravity comp | Trajectory tracking, gravity compensation via Pinocchio |

### Key Features

✅ **Multi-Mode Control** (`mujoco_ros2_control`) - Switch between position, velocity, torque, or combined control
✅ **Gravity Compensation** (`zordi_ros_controllers`) - Hold any pose with <0.01 rad drift via Pinocchio dynamics
✅ **MIT Mode** (`mujoco_ros2_control`) - Full state control: `τ = Kp·(q_cmd - q) + Kd·(qd_cmd - qd) + τ_gravity`
✅ **MuJoCo Simulation** (`mujoco_ros2_control`) - Physics-accurate testing at 1000 Hz
✅ **Controller Switching** (ROS2 Control Framework) - Seamlessly switch modes during operation

### Available Controllers

| Controller | Package | Interfaces | Behavior | Use Case |
|-----------|---------|-----------|----------|----------|
| **joint_trajectory_controller** | ros2_controllers (ROS2 standard) | pos + vel | Zero oscillation, no gravity comp | Standard trajectory execution |
| **zordi_joint_mit_controller** | zordi_ros_controllers | pos + vel + eff | MIT mode in hardware, ~0.5s settling | Hardware-like simulation with gravity comp |
| **zordi_joint_effort_controller** | zordi_ros_controllers | eff only | Software PD with gravity comp | Pure torque-controlled robots |
| **zordi_joint_effort_grav_comp_controller** | zordi_ros_controllers | eff only | Backdrivable, slow drift | Pure gravity compensation testing |
| **zordi_joint_mit_rnea_controller** | zordi_ros_controllers | pos + vel + eff | Full inverse dynamics with cubic splines | Superior tracking performance |
| **zordi_cartesian_mit_controller** | zordi_ros_controllers | pos + vel + eff | Cartesian impedance with nullspace | Cartesian pose tracking |
| **zordi_cartesian_mit_rnea_controller** | zordi_ros_controllers | pos + vel + eff | Advanced Cartesian with RNEA | High-performance Cartesian control |

---

## Prerequisites

### Installation

```bash
# Install dependencies
sudo apt install ros-humble-pinocchio  # Required for gravity comp
pip install mujoco urdf2mjcf

# Build workspace
cd ~/ros2_ws
colcon build --packages-select mujoco_ros2_control mujoco_ros2_control_demos zordi_ros_controllers openarm_description mujoco_ros2_control_msgs openarm_tests --symlink-install
source install/setup.bash
```

---

## Quick Start Tests

### Test 1: Basic Trajectory with Standard Controller

**Launch:**

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch openarm_description test_openarm_multimode.launch.py
```

**Expected:** MuJoCo viewer opens, robot at home position. Simulation starts automatically (`unpause: True`).

**Activate and test:**

```bash
# Terminal 2
ros2 control set_controller_state joint_trajectory_controller active

# Send trajectory
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{
    joint_names: [openarm_joint1, openarm_joint2, openarm_joint3,
                  openarm_joint4, openarm_joint5, openarm_joint6, openarm_joint7],
    points: [{
      positions: [0.5, 0.0, 0.0, 0.3, 0.5, 0.0, 0.0],
      time_from_start: {sec: 2}
    }]
  }"
```

### Test 2: Gravity Compensation with MIT Mode

**Launch at test pose:**

```bash
ros2 launch openarm_description test_openarm_multimode.launch.py initial_keyframe:=pose1
```

**Expected:** Robot starts at pose1 `[1.0, 1.5, -1.0, 2.0, 1.0, -0.7, 1.5]`

**Activate gravity compensation:**

```bash
# Terminal 2
ros2 control set_controller_state zordi_joint_effort_grav_comp_controller active
```

**Expected Result:**

- Robot holds position with gravity compensation
- Some drift may occur (this is expected for pure gravity comp mode)

**Monitor:**

```bash
# Check joint states
ros2 topic echo /joint_states --once

# Expected: positions near [1.0, 1.5, -1.0, 2.0, 1.0, -0.7, 1.5]
# Expected: velocities near [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

---

### Test 3: Trajectory Tracking with Effort Controller

**Activate effort controller:**

```bash
ros2 control set_controller_state zordi_joint_effort_controller active
```

**Send trajectory via action:**

```bash
ros2 action send_goal /zordi_joint_effort_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [openarm_joint1, openarm_joint2, openarm_joint3,
                    openarm_joint4, openarm_joint5, openarm_joint6, openarm_joint7],
      points: [
        {
          positions: [0.5, 0.5, -0.5, 1.0, 0.5, -0.5, 0.5],
          time_from_start: {sec: 3}
        }
      ]
    }
  }" --feedback
```

**Expected Result:**

- ✅ Smooth trajectory execution with gravity compensation
- ✅ Progress feedback shown in terminal
- ✅ Automatic hold after completion (no drift)

**Multi-point trajectory:**

```bash
ros2 action send_goal /zordi_joint_effort_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [openarm_joint1, openarm_joint2, openarm_joint3,
                    openarm_joint4, openarm_joint5, openarm_joint6, openarm_joint7],
      points: [
        {positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}},
        {positions: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 4}},
        {positions: [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 6}},
        {positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 8}}
      ]
    }
  }" --feedback
```

---

## Integration Test Suite (openarm_tests)

The `openarm_tests` package provides comprehensive integration tests using the
ROS2 `launch_testing` framework. These tests run automatically with pass/fail
verification.

### Running All Tests

```bash
# Build the test package
colcon build --packages-select openarm_tests

# Run all integration tests
colcon test --packages-select openarm_tests

# View detailed results
colcon test-result --test-result-base build/openarm_tests --verbose
```

### Running Individual Tests

```bash
# Run a single test with full output
python3 -m launch_testing.launch_test \
    src/openarm_tests/test/test_joint_mit.test.py

# Filter output for pass/fail summary
python3 -m launch_testing.launch_test \
    src/openarm_tests/test/test_joint_mit.test.py \
    2>&1 | grep -E "PASS|FAIL|error"
```

### Available Integration Tests

#### Single Arm Tests

| Test File | Controller | What's Verified |
|-----------|------------|-----------------|
| `test_joint_mit.test.py` | `zordi_joint_mit_controller` | MIT mode trajectory tracking |
| `test_joint_mit_rnea.test.py` | `zordi_joint_mit_rnea_controller` | RNEA inverse dynamics tracking |
| `test_joint_mit_effort.test.py` | `zordi_joint_effort_controller` | Software PD trajectory tracking |
| `test_joint_mit_effort_rnea.test.py` | `zordi_joint_effort_rnea_controller` | Software PD + RNEA |
| `test_joint_grav_comp.test.py` | `zordi_joint_effort_grav_comp_controller` | Position hold against gravity |
| `test_cartesian_mit.test.py` | `zordi_cartesian_mit_controller` | Cartesian impedance (MIT mode) |
| `test_cartesian_mit_rnea.test.py` | `zordi_cartesian_mit_rnea_controller` | Cartesian MIT + RNEA |
| `test_cartesian_mit_effort.test.py` | `zordi_cartesian_effort_controller` | Cartesian impedance (software PD) |
| `test_cartesian_mit_effort_rnea.test.py` | `zordi_cartesian_effort_rnea_controller` | Cartesian software PD + RNEA |
| `test_cartesian_ik.test.py` | `zordi_cartesian_ik_controller` | IK + joint trajectory execution |
| `test_gain_safety_limits.test.py` | N/A | Tests max_kp/max_kd safety limits |

#### Bimanual Tests (Right Arm)

| Test File | Controller | What's Verified |
|-----------|------------|-----------------|
| `test_bimanual_joint_mit.test.py` | `right_zordi_joint_mit_controller` | Right arm MIT mode |
| `test_bimanual_joint_mit_rnea.test.py` | `right_zordi_joint_mit_rnea_controller` | Right arm RNEA |
| `test_bimanual_joint_mit_effort.test.py` | `right_zordi_joint_effort_controller` | Right arm software PD |
| `test_bimanual_joint_mit_effort_rnea.test.py` | `right_zordi_joint_effort_rnea_controller` | Right arm software PD + RNEA |
| `test_bimanual_joint_grav_comp.test.py` | `right_zordi_joint_effort_grav_comp_controller` | Right arm gravity comp |
| `test_bimanual_cartesian_mit.test.py` | `right_zordi_cartesian_mit_controller` | Right arm Cartesian |
| `test_bimanual_cartesian_mit_rnea.test.py` | `right_zordi_cartesian_mit_rnea_controller` | Right arm Cartesian + RNEA |
| `test_bimanual_cartesian_mit_effort.test.py` | `right_zordi_cartesian_effort_controller` | Right arm Cartesian software PD |
| `test_bimanual_cartesian_mit_effort_rnea.test.py` | `right_zordi_cartesian_effort_rnea_controller` | Right arm Cartesian software PD + RNEA |
| `test_bimanual_cartesian_ik.test.py` | `right_zordi_cartesian_ik_controller` | Right arm Cartesian IK |

---

## Available Launch Files

| Launch File | Description |
|-------------|-------------|
| `test_openarm_multimode.launch.py` | Single arm with all controllers (inactive) |
| `test_openarm_bimanual_multimode.launch.py` | Right arm (bimanual mount) with all controllers |
| `display_openarm.launch.py` | Display robot in RViz |
| `mujoco_with_full_viewer.launch.py` | MuJoCo with full viewer |
| `single_arm.launch.py` | Basic single arm launch |

### Running Launch Files

```bash
# Single arm multimode (default home keyframe)
ros2 launch openarm_description test_openarm_multimode.launch.py

# Single arm with specific keyframe
ros2 launch openarm_description test_openarm_multimode.launch.py initial_keyframe:=pose1

# Right arm (bimanual config)
ros2 launch openarm_description test_openarm_bimanual_multimode.launch.py
```

### Expected Results

**Joint Trajectory Tests:**

- Robot moves smoothly from home to pose1 and back
- No oscillation during trajectory execution
- Stable hold at final position

**Gravity Compensation Test:**

- Robot holds pose with minimal drift
- Gravity compensation torques visible in `/joint_states`

**Cartesian Tests:**

- End-effector reaches target Cartesian pose
- Smooth trajectory in Cartesian space

---

## Advanced Testing

### Test All Keyframes

**Single arm keyframes (openarm_v10.xml):**

```bash
# Stable hanging
ros2 launch openarm_description test_openarm_multimode.launch.py initial_keyframe:=stable_hanging

# Canonical (all zeros)
ros2 launch openarm_description test_openarm_multimode.launch.py initial_keyframe:=canonical

# Home (elbow bent)
ros2 launch openarm_description test_openarm_multimode.launch.py initial_keyframe:=home

# Test poses
ros2 launch openarm_description test_openarm_multimode.launch.py initial_keyframe:=pose1
ros2 launch openarm_description test_openarm_multimode.launch.py initial_keyframe:=pose2
```

**Available keyframes in `openarm_v10.xml`:**

| Name | Joint Values (qpos) | Description |
|------|---------------------|-------------|
| stable_hanging | 0.0, -0.785, 0.0, 1.57, 0.0, 0.0, 0.0 | Arm hanging down |
| canonical | 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 | All zeros |
| home | 0.0, 0.785, 0.0, 1.57, 0.0, 0.0, 0.0 | Elbow bent at 90° |
| pose1 | 1.0, 1.5, -1.0, 2.0, 1.0, -0.7, 1.5 | Test configuration 1 |
| pose2 | -1.0, -1.5, 1.0, -2.0, -1.0, 1.5, -1.5 | Test configuration 2 |

### Runtime Keyframe Reset

```bash
# Reset to named keyframe
ros2 service call /mujoco_ros2_control/reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'pose1'}"

# Reset by index (as string)
ros2 service call /mujoco_ros2_control/reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: '3'}"
```

### Controller Switching

```bash
# Launch at pose1
ros2 launch openarm_description test_openarm_multimode.launch.py initial_keyframe:=pose1

# Start with MIT controller
ros2 control set_controller_state zordi_joint_mit_controller active

# Wait for settling...

# Switch to gravity comp
ros2 control set_controller_state zordi_joint_mit_controller inactive
ros2 control set_controller_state zordi_joint_effort_grav_comp_controller active

# Watch it drift slowly (expected!)

# Switch back to MIT
ros2 control set_controller_state zordi_joint_effort_grav_comp_controller inactive
ros2 control set_controller_state zordi_joint_mit_controller active

# Now holds at wherever it drifted to
```

### Gravity Compensation Accuracy

```bash
# Launch at pose1
ros2 launch openarm_description test_openarm_multimode.launch.py initial_keyframe:=pose1

# Activate MIT controller
ros2 control set_controller_state zordi_joint_mit_controller active

# Monitor effort commands (should see non-zero gravity torques)
ros2 topic echo /joint_states | grep -A 7 "effort:"

# Expected: joint2 ~5 Nm, other joints proportional to their loads
```

---

## Performance Benchmarks

### Expected Behavior

| Controller | Settling Time | Overshoot | Steady-State Error |
|-----------|---------------|-----------|-------------------|
| joint_trajectory_controller | 0.0s | None | Small (gravity sag) |
| zordi_joint_mit_controller | ~0.5s | Slight | ~Zero (gravity comp) |
| zordi_joint_effort_controller | ~0.5s | Slight | ~Zero (gravity comp) |
| zordi_joint_effort_grav_comp_controller | N/A | N/A | Continuous drift |
| zordi_joint_mit_rnea_controller | ~0.3s | Minimal | ~Zero (full dynamics) |

### System Performance

- **Update Rate:** 1000 Hz (1ms timestep)
- **CPU Usage:** ~10-15% (single core)
- **Real-Time Factor:** ~1.0

---

## Modifying Configuration

### Update PID Gains

**For MIT Mode controllers (HW PD, SW PD):**

Edit `urdf/ros2_control/openarm.ros2_control.xacro`:

```xml
<xacro:configure_joint joint_name="openarm_${arm_prefix}joint1"
                      initial_position="0.0"
                      kp="150.0"   <!-- Increase from 100.0 -->
                      kd="12.0"    <!-- Increase from 10.0 -->
                      ki="0.0"/>
```

Then regenerate and rebuild:

```bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/urdf_to_mjcf.py --output mujoco_models/openarm_v10.xml

cd ~/ros2_ws
colcon build --packages-select openarm_description --symlink-install
source install/setup.bash
```

**Note:** Script automatically reads URDF gains and applies to MuJoCo XML!

### Add Custom Keyframes

Edit `config/mujoco/initial_poses.yaml`:

```yaml
poses:
  my_custom_pose:
    joint1: 0.5
    joint2: 0.3
    joint3: -0.2
    joint4: 1.0
    joint5: 0.0
    joint6: 0.0
    joint7: 0.0
    description: "My custom test pose"
```

Then regenerate:

```bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/urdf_to_mjcf.py --output mujoco_models/openarm_v10.xml
colcon build --packages-select openarm_description --symlink-install
```

The script automatically reads keyframes from `config/mujoco/initial_poses.yaml` and validates joint counts!

---

## Troubleshooting

### Robot Falls/Drops

**Possible causes:**

1. Gravity compensation not working (Pinocchio failed to load)
2. Effort limits too low
3. Wrong controller active

**Check:**

```bash
# Look for Pinocchio initialization in logs
# Should see: "Pinocchio model loaded: 7 DOF, 8 joints"
# Should see: "Gravity vector set to: [0.00, 0.00, -9.81]"

# Check controllers
ros2 control list_controllers

# Verify Pinocchio
python3 -c "import pinocchio; print('OK')"
```

### Wild Oscillation

**Cause:** Using old version without joint ordering fix

**Fix:**

```bash
cd ~/ros2_ws
colcon build --packages-select zordi_ros_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Test with joint_trajectory_controller (should be stable)
ros2 control set_controller_state joint_trajectory_controller active
```

### MuJoCo Viewer Doesn't Render

**Fix:**

```bash
# Test if XML loads
cd ~/ros2_ws/src/openarm_description
python3 -c "import mujoco; m = mujoco.MjModel.from_xml_path('mujoco_models/openarm_v10.xml'); print('OK')"

# If it fails, regenerate
python3 scripts/urdf_to_mjcf.py --output mujoco_models/openarm_v10.xml

# Rebuild
cd ~/ros2_ws
colcon build --packages-select openarm_description --symlink-install
```

### Keyframes Don't Work

**Cause:** Keyframe definitions missing or wrong DOF count

**Fix:** Regenerate XML from `initial_poses.yaml`:

```bash
cd ~/ros2_ws/src/openarm_description
python3 scripts/urdf_to_mjcf.py --output mujoco_models/openarm_v10.xml
colcon build --packages-select openarm_description --symlink-install
```

The script automatically reads keyframes from `config/mujoco/initial_poses.yaml` and validates joint counts!

### Clean Slate Reset

If things get messy:

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select openarm_description zordi_ros_controllers mujoco_ros2_control openarm_tests --symlink-install
source install/setup.bash
ros2 launch openarm_description test_openarm_multimode.launch.py initial_keyframe:=home
```

---

## Quick Reference Commands

```bash
# List controllers
ros2 control list_controllers

# Activate controller
ros2 control set_controller_state <controller_name> active

# Deactivate controller
ros2 control set_controller_state <controller_name> inactive

# Reset to keyframe
ros2 service call /mujoco_ros2_control/reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'pose1'}"

# Monitor joint states
ros2 topic echo /joint_states

# Check system status
ros2 control list_controllers && ros2 topic hz /joint_states
```

---

## Summary: What to Expect

✅ **joint_trajectory_controller:**

- Zero oscillation, smooth tracking
- Slight gravity sag (no gravity comp)
- Standard ROS2 trajectory execution

✅ **zordi_joint_mit_controller / zordi_joint_effort_controller:**

- ~0.5s settling with slight overshoot (**normal for discrete-time PD!**)
- Stable hold with gravity compensation
- Zero gravity sag

✅ **zordi_joint_effort_grav_comp_controller:**

- Slow drift (expected - no position control!)
- Fully backdrivable
- Gravity compensated

✅ **zordi_joint_mit_rnea_controller:**

- Superior tracking with full inverse dynamics
- Faster settling (~0.3s)
- Best for high-performance applications

❌ **Problem indicators:**

- **Wild oscillation** → Joint ordering fix not applied, rebuild controller
- **Immediate drop** → Pinocchio not loaded or effort limits too low
- **Won't activate** → Check YAML configuration

---

## Next Steps

### For New Users

1. Run Quick Start tests above
2. Try different keyframes
3. Practice controller switching
4. Monitor joint states and effort commands

### For Developers

1. **Technical details:** See `KEY_FIXES_SUMMARY.md`
2. **Tune PID gains:** Modify URDF, regenerate XML
3. **Add keyframes:** Edit `initial_poses.yaml`, regenerate
4. **Custom controllers:** Use `zordi_ros_controllers` as template

### For Real Hardware

1. Same controllers work with `OpenArm_v10HW` plugin
2. Tune gains for your payload
3. Test gravity compensation accuracy
4. Integrate with your application

---

## Related Documentation

### This Package (`openarm_description`)

- **Technical fixes:** [KEY_FIXES_SUMMARY.md](KEY_FIXES_SUMMARY.md) - Critical bug fixes and solutions
- **Cartesian Quick Start:** [CARTESIAN_QUICK_START_BIMANUAL.md](CARTESIAN_QUICK_START_BIMANUAL.md) - Right arm Cartesian control

### Other Packages

- **MuJoCo ROS2 Control:** `mujoco_ros2_control/doc/updates.md` - MIT mode architecture
- **MuJoCo Demos:** `mujoco_ros2_control_demos/README.md` - Examples and tutorials
- **Zordi MIT Controller:** `zordi_ros_controllers/README.md` - Controller API and configuration
- **OpenARM Tests:** `openarm_tests/README.md` - Integration test documentation

---

**Document Version:** 4.0
**Last Updated:** November 28, 2025
**Status:** ✅ Production Ready
