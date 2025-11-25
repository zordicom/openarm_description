# Cartesian Controller Quick Start - Right Arm

**Status:** ✅ Ready for Testing
**Last Updated:** November 19, 2025

---

## Overview

This guide covers the right arm OpenARM setup (single 7-DOF arm, bimanual shoulder mounting style). This configuration uses the right arm extracted from the bimanual URDF, mounted with the same shoulder geometry as the bimanual setup.

---

## Quick Launch

```bash
cd /home/gilwoo/ros2_ws
source install/setup.bash

# Launch right arm system (starts from "extended" keyframe by default)
ros2 launch openarm_description test_openarm_bimanual_multimode.launch.py initial_keyframe:="extended"

# In another terminal (after sourcing):
ros2 control list_controllers

# Activate right arm Cartesian controller
ros2 control set_controller_state right_zordi_cartesian_mit_controller active
```

---

## Controller Configuration Requirements ("controllers_bimanual_multimode_test.yaml")

### 1. Torque Limits Must Be Explicit

The right arm requires explicit torque limits:

```yaml
torque_limits: [200.0, 200.0, 150.0, 150.0, 50.0, 50.0, 50.0]
```

### 2. Frame Names Must Be Configured in Controller Config

These parameters are set in `controllers_bimanual_multimode_test.yaml` for each Cartesian controller:

```yaml
# For right_zordi_cartesian_mit_controller (lines 466-467):
end_effector_frame: "openarm_right_link7"
base_frame: "openarm_right_link0"

# These frame names must exist in the URDF!
```

---

## Available Controllers

### Right Arm Controllers

1. **right_joint_trajectory_controller** - Standard ROS2 (position+velocity)
2. **right_zordi_grav_comp_controller** - Pure gravity compensation
3. **right_zordi_hardware_pd_controller** - Hardware PD (MIT mode)
4. **right_zordi_software_pd_controller** - Software PD (effort control)
5. **right_zordi_mit_rnea_controller** - Joint trajectory tracking with RNEA feedforward (MIT mode)
6. **right_zordi_cartesian_mit_controller** - Cartesian impedance with nullspace control (MIT mode)
7. **right_zordi_cartesian_mit_rnea_controller** - Cartesian impedance with RNEA feedforward (MIT mode)

### MIT Mode Explanation

The controllers marked with **(MIT mode)** utilize all three command interfaces (`position`, `velocity`, `effort`) simultaneously to achieve high-performance control:

1. **Position (`q_cmd`)**: Sends the desired joint position (computed via differential IK) to the hardware.
2. **Velocity (`qd_cmd`)**: Sends **0.0** velocity. This effectively uses the hardware's D-gain to damp the system (`Kd * (0 - q_vel)`).
3. **Effort (`tau_ff`)**: Sends the computed feedforward torque (Gravity + Coriolis + Task Forces + Nullspace).

This combination allows the hardware (or simulator) to run a high-frequency PD loop around the setpoints while the controller provides the complex dynamics feedforward.

---

## Usage Examples

### Method 1: Topic-Based Control (Simple, Immediate)

Send target poses directly to the controller. The controller smoothly interpolates to the target.

**Example: Move to Home Position**

```bash
ros2 topic pub --once /right_zordi_cartesian_mit_rnea_controller/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: 'openarm_right_link0'
    },
    pose: {
      position: {x: 0.216000, y: -0.155747, z: 0.278123},
      orientation: {x: 0.270836, y: 0.652967, z: 0.270620, w: 0.653488}
    }
  }"
```

This shuold get the joint states close to "<key name="home" qpos="0.0 0.785 0.0 1.57 0.0 0.0 0.0" />".

### Method 2: Action-Based Control (Trajectory with Timing)

Use actions for precise timing and feedback. Recommended for smoother, more controlled motions.

**Example: Move to Home Position Over 3 Seconds**

```bash
ros2 action send_goal --feedback /right_zordi_cartesian_mit_rnea_controller/follow_cartesian_trajectory \
  zordi_ros_controllers_msgs/action/FollowCartesianTrajectory \
  "{
    trajectory: {
      points: [
        {
          point: {
            pose: {
              position: {x: 0.216000, y: -0.155747, z: 0.278123},
              orientation: {x: 0.270836, y: 0.652967, z: 0.270620, w: 0.653488}
            }
          },
          time_from_start: {sec: 3, nanosec: 0}
        }
      ]
    }
  }"
```

### Keyframe Cartesian Poses (All in Base Frame: openarm_right_link0)

Keyframes are defined in "openarm_v10_right_arm_proper.xml"

#### **Home** (joint4 = 1.57 rad, elbow bent)

```yaml
position: {x: 0.216000, y: -0.155747, z: 0.278123}
orientation: {x: 0.270836, y: 0.652967, z: 0.270620, w: 0.653488}
```

#### **Extended** (joint4 = 1.0 rad, elbow more extended)

```yaml
position: {x: 0.181758, y: -0.238181, z: 0.360492}
orientation: {x: 0.335998, y: 0.442895, z: 0.183556, w: 0.810714}
```

#### **Canonical** (all joints = 0)

```yaml
position: {x: 0.000000, y: -0.436000, z: 0.122500}
orientation: {x: 0.707107, y: 0.000000, z: 0.000000, w: 0.707107}
```

#### **Pose1** (reaching outward)

```yaml
position: {x: 0.212857, y: -0.256713, z: 0.122500}
orientation: {x: 0.604925, y: 0.615794, z: 0.298166, w: 0.407383}
```

### Test Sequence: Extended → Home → Extended

**Step 1: Activate Controller**

```bash
ros2 launch openarm_description test_openarm_bimanual_multimode.launch.py  initial_keyframe:="extended"
# or ros2 service call /reset_to_keyframe   mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'extended'}"
ros2 control set_controller_state right_zordi_cartesian_mit_rnea_controller active
ros2 service call /simulation_control mujoco_ros2_control_msgs/srv/SimulationControl   "{command: 'unpause'}"
```

**Step 2: Move to Home (from default "extended" startup)**

```bash
ros2 action send_goal --feedback /right_zordi_cartesian_mit_rnea_controller/follow_cartesian_trajectory \
  zordi_ros_controllers_msgs/action/FollowCartesianTrajectory \
  "{
    trajectory: {
      points: [
        {
          point: {
            pose: {
              position: {x: 0.216000, y: -0.155747, z: 0.278123},
              orientation: {x: 0.270836, y: 0.652967, z: 0.270620, w: 0.653488}
            }
          },
          time_from_start: {sec: 3, nanosec: 0}
        }
      ]
    }
  }"
```

**Step 3: Move Back to Extended**

```bash
ros2 action send_goal --feedback /right_zordi_cartesian_mit_rnea_controller/follow_cartesian_trajectory \
  zordi_ros_controllers_msgs/action/FollowCartesianTrajectory \
  "{
    trajectory: {
      points: [
        {
          point: {
            pose: {
              position: {x: 0.181758, y: -0.238181, z: 0.360492},
              orientation: {x: 0.335998, y: 0.442895, z: 0.183556, w: 0.810714}
            }
          },
          time_from_start: {sec: 3, nanosec: 0}
        }
      ]
    }
  }"
```

Check the joint states to confirm that it's roughly at the extended pose.
The extended pose is defined as:

```xml
    <key name="extended" qpos="0.0 0.785 0.0 1.0 0.0 0.0 0.0" />
```
