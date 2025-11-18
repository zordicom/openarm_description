# Cartesian Controller Quick Start

**Status:** ✅ Working
**Last Updated:** November 18, 2025

---

## Quick Launch

```bash
cd /home/gilwoo/ros2_ws
source install/setup.bash

# Launch full multimode system with all controllers
ros2 launch openarm_description test_openarm_multimode.launch.py

# In another terminal (after sourcing):
ros2 control list_controllers
ros2 control set_controller_state zordi_cartesian_controller active
```

---

## What Was Fixed

The controller was crashing due to **missing explicit torque limits** in the configuration.

**The Fix:**

```yaml
# Before (FAILED):
torque_limits: []

# After (WORKS):
torque_limits: [200.0, 200.0, 150.0, 150.0, 50.0, 50.0, 50.0]
```

---

## Configuration Requirements

### 1. Torque Limits Must Be Explicit

```yaml
torque_limits: [200.0, 200.0, 150.0, 150.0, 50.0, 50.0, 50.0]
```

- Must match the number of joints (7 for OpenARM)
- Values should match URDF effort limits
- Empty array `[]` causes controller to crash during initialization

### 2. Frame Names Must Exist in URDF

```yaml
end_effector_frame: "openarm_link7"  # Must be a valid link name
base_frame: "openarm_link0"          # Must be a valid link name
```

### 3. PID Parameters in URDF

The OpenARM URDF includes both software and hardware PID parameters:

```xml
<joint name="openarm_joint1">
  <!-- Software PD gains (for Cartesian/torque controllers) -->
  <param name="kp">800.0</param>
  <param name="kd">40.0</param>

  <!-- Hardware PD gains (for MIT mode actuators) -->
  <param name="velocity_kp">40.0</param>
  ...
</joint>
```

The controller uses a priority system:

- Prefers `kp`/`kd` for software PD
- Falls back to `position_kp`/`position_kd` or `velocity_kp` if needed

---

## Available Controllers

### Joint Space Controllers

1. **joint_trajectory_controller** - Standard ROS2 (position+velocity)
2. **zordi_hardware_pd_controller** - Hardware PD (MIT mode)
3. **zordi_software_pd_controller** - Software PD (effort only)
4. **zordi_grav_comp_controller** - Pure gravity compensation (backdrivable)
5. **zordi_mit_rnea_controller** - Full inverse dynamics with cubic splines

### Cartesian Space Controllers

6. **zordi_cartesian_controller** - Cartesian impedance with nullspace control
7. **zordi_cartesian_rnea_controller** - Advanced Cartesian with full RNEA

---

## Usage Examples

### Activate Cartesian Controller

```bash
ros2 control set_controller_state zordi_cartesian_controller active
ros2 service call /reset_to_keyframe   mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'pose1'}"
ros2 service call /simulation_control   mujoco_ros2_control_msgs/srv/SimulationControl "{command: 'unpause'}"


```

### Send Target Pose

```bash
ros2 topic pub /zordi_cartesian_controller/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: 'openarm_link0'
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

### Send Cartesian Trajectory (Action Interface)

```bash
ros2 action send_goal /zordi_cartesian_controller/follow_cartesian_trajectory \
  moveit_msgs/action/ExecuteTrajectory "{...}"
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

1. **Empty torque_limits:** Add explicit values
2. **Wrong frame names:** Verify frames exist in URDF
3. **Array size mismatch:** Ensure arrays match joint count

### Controller Loads But Doesn't Activate

```bash
# Check controller status
ros2 control list_controllers

# Check for errors
ros2 control switch_controller --activate zordi_cartesian_controller
```

---

## Files

- **Main Config:** `config/mujoco/controllers_multimode_test.yaml`
- **Main Launch:** `launch/test_openarm_multimode.launch.py`
- **Validation:** Built into controller (automatic)
- **Documentation:** `docs/TESTING_GUIDE.md` (comprehensive testing guide)

---

## Next Steps

1. Test with the full multimode launch file
2. Try different Cartesian control modes
3. Experiment with stiffness/damping parameters
4. Test nullspace posture control

For detailed control equations and implementation details, see:

- `zordi_mit_controller/docs/CARTESIAN_CONTROL_EQUATIONS.md`
