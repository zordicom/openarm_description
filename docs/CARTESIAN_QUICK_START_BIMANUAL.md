# Cartesian Controller Quick Start - Bimanual

**Status:** ✅ Ready for Testing
**Last Updated:** November 18, 2025

---

## Overview

This guide covers the bimanual OpenARM setup with **two 7-DOF arms (14 joints total), no hands**. Each arm has its own set of controllers for independent control.

---

## Quick Launch

```bash
cd /home/gilwoo/ros2_ws
source install/setup.bash

# Launch full bimanual multimode system
ros2 launch openarm_description test_openarm_bimanual_multimode.launch.py

# In another terminal (after sourcing):
ros2 control list_controllers

# Activate left arm controller
ros2 control set_controller_state left_zordi_cartesian_controller active

# Activate right arm controller
ros2 control set_controller_state right_zordi_cartesian_controller active
```

---

## Configuration Requirements

### 1. Torque Limits Must Be Explicit

Each arm requires explicit torque limits:

```yaml
# Left arm
torque_limits: [200.0, 200.0, 150.0, 150.0, 50.0, 50.0, 50.0]

# Right arm
torque_limits: [200.0, 200.0, 150.0, 150.0, 50.0, 50.0, 50.0]
```

### 2. Frame Names Must Exist in URDF

```yaml
# Left arm
end_effector_frame: "openarm_left_link7"
base_frame: "openarm_left_link0"

# Right arm
end_effector_frame: "openarm_right_link7"
base_frame: "openarm_right_link0"
```

---

## Available Controllers

### Left Arm Controllers

1. **left_joint_trajectory_controller** - Standard ROS2 (position+velocity)
2. **left_zordi_hardware_pd_controller** - Hardware PD (MIT mode)
3. **left_zordi_software_pd_controller** - Software PD (effort only)
4. **left_zordi_grav_comp_controller** - Pure gravity compensation (backdrivable)
5. **left_zordi_mit_rnea_controller** - Full inverse dynamics with cubic splines
6. **left_zordi_cartesian_controller** - Cartesian impedance with nullspace control
7. **left_zordi_cartesian_rnea_controller** - Advanced Cartesian with full RNEA

### Right Arm Controllers

1. **right_joint_trajectory_controller** - Standard ROS2 (position+velocity)
2. **right_zordi_hardware_pd_controller** - Hardware PD (MIT mode)
3. **right_zordi_software_pd_controller** - Software PD (effort only)
4. **right_zordi_grav_comp_controller** - Pure gravity compensation (backdrivable)
5. **right_zordi_mit_rnea_controller** - Full inverse dynamics with cubic splines
6. **right_zordi_cartesian_controller** - Cartesian impedance with nullspace control
7. **right_zordi_cartesian_rnea_controller** - Advanced Cartesian with full RNEA

---

## Usage Examples

### Activate Both Cartesian Controllers

```bash
# Activate left arm
ros2 control set_controller_state left_zordi_cartesian_controller active

# Activate right arm
ros2 control set_controller_state right_zordi_cartesian_controller active
```

### Send Target Pose to Left Arm

```bash
ros2 topic pub /left_zordi_cartesian_controller/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: 'openarm_left_link0'
    },
    pose: {
      position: {
        x: -0.110293,
        y: 0.114208,
        z: 0.296562
      },
      orientation: {
        x: -0.142549,
        y: -0.371741,
        z: 0.826390,
        w: 0.398207
      }
    }
  }"
```

### Send Target Pose to Right Arm

**Test Sequence: Home → Extended**

This example shows the right arm moving from its home position to an extended configuration.

**Step 1: Move to Home Position** (joint4 = 1.57 rad):

```bash
ros2 topic pub --once /right_zordi_cartesian_controller/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: 'openarm_right_link0'
    },
    pose: {
      position: {
        x: 0.216000,
        y: -0.155747,
        z: 0.278123
      },
      orientation: {
        x: 0.270836,
        y: 0.652967,
        z: 0.270620,
        w: 0.653488
      }
    }
  }"
```

**Step 2: Wait for convergence, then move to Extended Position** (joint4 = 1.0 rad, elbow extends ~12 cm):

```bash
ros2 topic pub --once /right_zordi_cartesian_controller/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: 'openarm_right_link0'
    },
    pose: {
      position: {
        x: 0.181758,
        y: -0.238181,
        z: 0.360492
      },
      orientation: {
        x: 0.335998,
        y: 0.442895,
        z: 0.183556,
        w: 0.810714
      }
    }
  }"
```

> **Note:** The `--once` flag sends a single message and exits. The controller will smoothly move to each target using internal trajectory generation.

### Coordinated Bimanual Control

```bash
# Activate both gravity compensation controllers for backdrivable mode
ros2 control set_controller_state left_zordi_grav_comp_controller active
ros2 control set_controller_state right_zordi_grav_comp_controller active

# Or activate both Cartesian RNEA controllers for synchronized control
ros2 control set_controller_state left_zordi_cartesian_rnea_controller active
ros2 control set_controller_state right_zordi_cartesian_rnea_controller active
```

---

## Joint Names

### Left Arm Joints

```yaml
joints:
  - openarm_left_joint1
  - openarm_left_joint2
  - openarm_left_joint3
  - openarm_left_joint4
  - openarm_left_joint5
  - openarm_left_joint6
  - openarm_left_joint7
```

### Right Arm Joints

```yaml
joints:
  - openarm_right_joint1
  - openarm_right_joint2
  - openarm_right_joint3
  - openarm_right_joint4
  - openarm_right_joint5
  - openarm_right_joint6
  - openarm_right_joint7
```

---

## Control Modes

### Software PD Mode (`compute_pd_internally: true`)

- **Interfaces:** `effort` only
- **Control:** Pure torque control with software PD damping
- **Use case:** Pure torque-controlled robots

### Hardware PD Mode (`compute_pd_internally: false`)

- **Interfaces:** `position`, `velocity`, `effort`
- **Control:** Hardware PD + feedforward torques
- **Use case:** MIT-capable hardware (DAMIAO, MuJoCo)

---

## Troubleshooting

### Controller Won't Load

Check the console output for validation errors. Common issues:

1. **Empty torque_limits:** Add explicit values for each arm
2. **Wrong frame names:** Verify frames exist in URDF with correct prefixes
3. **Array size mismatch:** Ensure arrays match joint count (7 per arm)
4. **Mixed joint names:** Ensure left/right prefixes are consistent

### Only One Arm Working

Verify that:

- Both arms have ros2_control interfaces defined in URDF
- Joint names match between URDF, XML, and controller config
- Both arms' actuators are properly defined in the MuJoCo XML

### Controllers Conflict

Remember:

- Only ONE controller can be active per arm at a time
- Controllers don't share joints between arms
- Use `--set-state inactive` when switching controllers

---

## Files

- **URDF:** `mujoco_models/openarm_v10_bimanual.urdf`
- **MuJoCo XML:** `mujoco_models/openarm_v10_bimanual.xml`
- **Controller Config:** `config/mujoco/controllers_bimanual_multimode_test.yaml`
- **Launch File:** `launch/test_openarm_bimanual_multimode.launch.py`

---

## Controller Switching

### Switch Controllers on Same Arm

```bash
# Switch left arm from Cartesian to gravity comp
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['left_zordi_grav_comp_controller'],
    deactivate_controllers: ['left_zordi_cartesian_controller'],
    strictness: 2}"
```

### Independent Arm Control

```bash
# Left arm: Cartesian control
ros2 control set_controller_state left_zordi_cartesian_controller active

# Right arm: Gravity compensation
ros2 control set_controller_state right_zordi_grav_comp_controller active
```

---

## Validation

The controller includes built-in C++ validation that runs during `on_configure()`. It checks:

- ✅ All required parameters present
- ✅ Frames exist in robot model
- ✅ Array sizes match joint count
- ✅ Torque limits properly configured

If validation fails, the controller will log clear error messages and refuse to configure.

---

## Next Steps

1. Test individual arm control with Cartesian controllers
2. Test coordinated bimanual control
3. Experiment with different control modes per arm
4. Test nullspace posture control for each arm independently
5. Try mixed control modes (e.g., left arm Cartesian, right arm joint space)

For detailed control equations and implementation details, see:

- `zordi_mit_controller/docs/CARTESIAN_CONTROL_EQUATIONS.md`
- Main single-arm quick start: `CARTESIAN_QUICK_START.md`

---

## Tips for Bimanual Control

1. **Independent Control:** Each arm can use a different controller type
2. **Synchronized Control:** Both arms can use the same controller type for coordinated tasks
3. **Backdrivable Mode:** Use `grav_comp` controllers on both arms for manual teaching
4. **Frame Coordination:** Consider using a common world frame for coordinated tasks
5. **Collision Avoidance:** MuJoCo handles self-collision between arms automatically
