# OpenARM Description Package

**Copyright 2025 Zordi, Inc. All rights reserved.**

ROS2 package for the OpenARM 7-DOF robot arm with MuJoCo simulation support, gravity compensation, and multi-mode control.

---

## Quick Start

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select openarm_description zordi_ros_controllers --symlink-install
source install/setup.bash

# Launch simulation
ros2 launch openarm_description test_openarm_multimode.launch.py

# Terminal 2: Activate controller
ros2 control set_controller_state zordi_hardware_pd_controller active

# Send trajectory
ros2 action send_goal /zordi_hardware_pd_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
    trajectory: {
      joint_names: [openarm_joint1, openarm_joint2, openarm_joint3,
                    openarm_joint4, openarm_joint5, openarm_joint6, openarm_joint7],
      points: [{
        positions: [0.5, 0.5, -0.5, 1.0, 0.5, -0.5, 0.5],
        time_from_start: {sec: 3}
      }]
    }
  }" --feedback
```

---

## Documentation

### 📖 Main Guides

1. **[TESTING_GUIDE.md](docs/TESTING_GUIDE.md)** ⭐ **START HERE**
   - Quick-start testing instructions
   - All 4 controller modes explained
   - Trajectory control examples
   - Troubleshooting guide

2. **[KEY_FIXES_SUMMARY.md](docs/KEY_FIXES_SUMMARY.md)** ⭐ **Technical Details**
   - Critical bug fixes explained
   - Joint ordering mismatch solution
   - PID gains configuration
   - Why oscillation occurs in MIT mode

### 📚 Additional Documentation

For detailed architecture and implementation:
- **MuJoCo ROS2 Control:** [mujoco_ros2_control/doc/mujoco_ros2_control_updates.md](../../mujoco_ros2_control/doc/mujoco_ros2_control_updates.md)
- **Demos and Examples:** [mujoco_ros2_control_demos/README.md](../../mujoco_ros2_control/mujoco_ros2_control_demos/README.md)
- **zordi_ros_controllers:** [zordi_ros_controllers/README.md](../../zordi_ros_controllers/README.md)

---

## Key Features

✅ **Multi-Mode Control** - Position, velocity, torque, or combined (MIT mode)  
✅ **Gravity Compensation** - Hold any pose with <0.01 rad drift  
✅ **4 Controllers** - Standard JTC, HW PD, SW PD, Gravity Comp  
✅ **MuJoCo Simulation** - Physics-accurate testing at 1000 Hz  
✅ **Automatic Gain Sync** - URDF PID gains → MuJoCo XML actuators  
✅ **Keyframe Management** - Define poses in YAML, auto-validated

---

## Available Controllers

| Controller | Type | Behavior |
|-----------|------|----------|
| `joint_trajectory_controller` | Standard ROS2 | Zero oscillation, uses MuJoCo native actuators |
| `zordi_hardware_pd_controller` | MIT Mode | Discrete-time PD + gravity comp, ~0.5s settling |
| `zordi_software_pd_controller` | MIT Mode | Same as HW PD (internal PD not yet implemented) |
| `zordi_grav_comp_controller` | Effort Only | Backdrivable, pure gravity compensation |

---

## Package Structure

```
openarm_description/
├── urdf/                           # Robot description
│   ├── robot/v10.urdf.xacro       # Main robot xacro
│   └── ros2_control/              # ROS2 Control configs with PID gains
├── mujoco_models/                  # MuJoCo XML models
│   └── openarm_v10.xml            # Generated from URDF, includes keyframes
├── config/
│   ├── mujoco/
│   │   ├── initial_poses.yaml     # Keyframe definitions (SOURCE OF TRUTH)
│   │   └── controllers_*.yaml     # Controller configurations
│   └── arm/v10/
│       └── joint_limits.yaml      # Joint limits and effort limits
├── launch/
│   ├── test_openarm_multimode.launch.py  # Multi-controller testing
│   └── single_arm.launch.py       # Single controller launch
├── scripts/
│   └── urdf_to_mjcf.py           # Auto-generate XML with gains + keyframes
└── docs/                          # Documentation (see above)
```

---

## Workflow: Updating Configuration

### Update PID Gains

1. Edit `urdf/ros2_control/openarm.ros2_control.xacro` (change `kp`, `kd` values)
2. Run `python3 scripts/urdf_to_mjcf.py --output mujoco_models/openarm_v10.xml`
3. Script automatically:
   - ✅ Reads URDF PID gains
   - ✅ Writes to MuJoCo actuator kp/kv
   - ✅ Preserves keyframes from `initial_poses.yaml`
   - ✅ Validates joint matching
   - ✅ Formats XML properly

### Update Keyframes

1. Edit `config/mujoco/initial_poses.yaml` (add/modify poses)
2. Run `python3 scripts/urdf_to_mjcf.py --output mujoco_models/openarm_v10.xml`
3. Script validates all joints are present and rebuilds XML

### Update Effort Limits

1. Edit `config/arm/v10/joint_limits.yaml`
2. Regenerate: `python3 scripts/urdf_to_mjcf.py --output mujoco_models/openarm_v10.xml`
3. Rebuild: `colcon build --packages-select openarm_description --symlink-install`

---

## Important Notes

### PID Gains: Two Locations, Same Values

**URDF parameters** (`urdf/ros2_control/openarm.ros2_control.xacro`):
```xml
<param name="position_kp">100.0</param>
<param name="position_kd">10.0</param>
```
- Used by: `zordi_hardware_pd`, `zordi_software_pd` (MIT mode)
- When: Software PD in `mujoco_system.cpp`

**MuJoCo actuator gains** (`mujoco_models/openarm_v10.xml`):
```xml
<position kp="100.0" kv="0.0" ... />
<velocity kv="10.0" ... />
```
- Used by: `joint_trajectory_controller`
- When: MuJoCo native continuous-time control

**The `urdf_to_mjcf.py` script automatically synchronizes these!**

### Why Keep Two Sets?

- **Different control paradigms:** Continuous-time (MuJoCo) vs discrete-time (software)
- **Different use cases:** Pure simulation vs hardware-realistic testing
- **Real hardware only uses URDF:** No "MuJoCo XML" on actual DAMIAO motors
- **Allows testing both:** JTC for zero-oscillation sim, MIT mode for realistic hardware simulation

---

## Contact

**Maintainer:** Zordi Dev <dev@zordi.com>  
**Package Version:** 3.0  
**ROS2 Distribution:** Humble  
**Status:** ✅ Production Ready

---

**📖 Start here:** [docs/TESTING_GUIDE.md](docs/TESTING_GUIDE.md)  
**🔧 Technical fixes:** [docs/KEY_FIXES_SUMMARY.md](docs/KEY_FIXES_SUMMARY.md)
