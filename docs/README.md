# OpenARM - Actuator-Centric Control with MIT Mode

**Status**: ✅ Production Ready  
**Last Updated**: November 13, 2025

---

## What is This?

An **actuator-centric control system** for the Zordi OpenARM 6-DOF robot arm with:

- ✅ **Multi-mode control** - Position, velocity, torque, or combined (MIT mode)
- ✅ **ROS2 Control integration** - Dynamic controller switching
- ✅ **Gravity compensation** - Hold any pose against gravity (< 0.001 rad drift)
- ✅ **MuJoCo simulation** - Physics-accurate testing environment
- ✅ **Validated performance** - Excellent tracking (< 0.003 rad RMSE)

---

## System Capabilities

### Control Modes
1. **Position-only** - Direct position control
2. **Velocity-only** - Direct velocity control  
3. **Torque-only** - Direct torque control
4. **Position + Velocity** - Joint trajectory tracking
5. **MIT Mode** - Full state control with gravity compensation

### Performance Metrics

| Test | Metric | Result | Status |
|------|--------|--------|--------|
| Joint Trajectory | Tracking RMSE | 0.0014 rad | ✅ Excellent |
| MIT Mode | Tracking RMSE | 0.0025 rad | ✅ Excellent |
| Gravity Hold (1-DOF) | Drift (10s) | 0.0001 rad | ✅ Perfect |
| Gravity Hold (6-DOF) | Drift (15s) | 0.0005 rad | ✅ Perfect |

---

## Quick Start

### 1. Installation

#### Prerequisites
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

#### Install Dependencies
```bash
# ROS2 Humble (if not installed)
sudo apt install ros-humble-desktop

# Pinocchio (ROS2 version - REQUIRED)
sudo apt install ros-humble-pinocchio

# MuJoCo and Python packages
pip install mujoco urdf2mjcf matplotlib

# Clone workspace
cd ~/ros2_ws/src
git clone https://github.com/zordicom/mujoco_ros2_control.git
git clone https://github.com/zordicom/openarm_description.git
git clone https://github.com/zordicom/openarm_config.git
git clone https://github.com/zordicom/zordi_mit_controller.git
git clone https://github.com/zordicom/crisp_controllers.git

# Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch OpenARM

#### With Gravity Compensation (Recommended)
```bash
ros2 launch openarm_description single_arm.launch.py \
  default_controller:=zordi_mit_controller \
  rviz:=true \
  headless:=false
```

#### Headless (No GUI)
```bash
ros2 launch openarm_description single_arm.launch.py \
  default_controller:=zordi_mit_controller \
  rviz:=false \
  headless:=true
```

### 3. Send Commands

#### Check Active Controllers
```bash
ros2 control list_controllers
```

Expected output:
```
zordi_mit_controller  zordi_mit_controller/ZordiMITController  active
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster  active
```

#### Send Trajectory (via action)
```bash
ros2 action send_goal /zordi_mit_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['openarm_joint1', 'openarm_joint2', 'openarm_joint3', 
                    'openarm_joint4', 'openarm_joint5', 'openarm_joint6'],
      points: [
        {positions: [0.0, 0.5, -0.5, 0.3, 0.0, 0.0], 
         time_from_start: {sec: 3}}
      ]
    }
  }"
```

#### Send Trajectory (via topic)
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

rclpy.init()
node = Node('trajectory_publisher')

pub = node.create_publisher(
    JointTrajectory,
    '/zordi_mit_controller/joint_trajectory',
    10
)

traj = JointTrajectory()
traj.joint_names = [
    'openarm_joint1', 'openarm_joint2', 'openarm_joint3',
    'openarm_joint4', 'openarm_joint5', 'openarm_joint6'
]

point = JointTrajectoryPoint()
point.positions = [0.0, 0.5, -0.5, 0.3, 0.0, 0.0]
point.velocities = [0.0] * 6
point.time_from_start = Duration(sec=3)
traj.points.append(point)

pub.publish(traj)
node.get_logger().info('Trajectory sent!')
```

---

## MIT Mode - Quick Reference

### What is MIT Mode?

MIT mode enables **full state control** with feedforward compensation:

```
τ = Kp*(q_cmd - q) + Kd*(qd_cmd - qd) + τ_ff
```

Where:
- `q_cmd`, `qd_cmd` = Desired position/velocity
- `q`, `qd` = Current position/velocity  
- `τ_ff` = Feedforward torque (e.g., gravity compensation)
- `Kp`, `Kd` = PID gains from URDF

### How to Enable MIT Mode

MIT mode is **automatically activated** when a controller claims all three interfaces:

```yaml
zordi_mit_controller:
  ros__parameters:
    joints:
      - openarm_joint1
      - openarm_joint2
      # ... etc
    command_interfaces:
      - position
      - velocity
      - effort  # ← This triggers MIT mode!
    state_interfaces:
      - position
      - velocity
    use_gravity_compensation: true  # Enable Pinocchio gravity comp
```

### Configuration

#### URDF - PID Gains
```xml
<joint name="openarm_joint1">
  <command_interface name="position"/>
  <command_interface name="velocity"/>
  <command_interface name="effort"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  
  <!-- MIT mode PID gains (use underscores, not dots!) -->
  <param name="position_kp">20.0</param>
  <param name="position_ki">0.0</param>
  <param name="position_kd">0.0</param>
  <param name="velocity_kp">0.0</param>
  <param name="velocity_ki">0.0</param>
  <param name="velocity_kd">2.0</param>
</joint>
```

#### MuJoCo XML - Actuators
```xml
<actuator>
  <!-- Position actuator (inactive in MIT mode) -->
  <position name="act_pos_openarm_joint1" joint="openarm_joint1"
            kp="20.0" kv="0.0" ctrlrange="-3.14 3.14" forcerange="-87 87"/>
  
  <!-- Velocity actuator (inactive in MIT mode) -->
  <velocity name="act_vel_openarm_joint1" joint="openarm_joint1"
            kv="2.0" ctrlrange="-2.0 2.0" forcerange="-87 87"/>
  
  <!-- Torque actuator (receives composed τ in MIT mode) -->
  <motor name="act_tau_openarm_joint1" joint="openarm_joint1"
         ctrlrange="-87 87" forcerange="-87 87"/>
</actuator>
```

### Typical Gains

| Joint Type | Kp | Kd | Notes |
|------------|----|----|-------|
| 1-DOF test | 100 | 10 | High gains for testing |
| OpenARM (full) | 20 | 2 | Lower for stability |
| Heavy payload | 30-50 | 3-5 | Increase for tracking |

### How It Works

1. **Controller claims** `[position, velocity, effort]`
2. **mujoco_system.cpp detects** all three interfaces active
3. **Switches to MIT mode**:
   - Reads `position_kp`, `velocity_kd` from URDF
   - Computes: `τ_pd = Kp*(q_cmd - q) + Kd*(qd_cmd - qd)`
   - Adds feedforward: `τ_total = τ_pd + effort_cmd`
   - Sends `τ_total` to torque actuator
4. **Position/velocity actuators neutralized** (ctrl = current state)

---

## Controller Selection

### Available Controllers

1. **zordi_mit_controller** (Recommended)
   - Claims: `[position, velocity, effort]`
   - Features: Gravity compensation via Pinocchio
   - Best for: Full state control, holding against gravity
   - Action: `/zordi_mit_controller/follow_joint_trajectory`
   - Topic: `/zordi_mit_controller/joint_trajectory`

2. **joint_trajectory_controller**
   - Claims: `[position, velocity]`
   - Features: Standard ROS2 trajectory execution
   - Best for: Trajectory tracking without MIT mode
   - Action: `/joint_trajectory_controller/follow_joint_trajectory`

3. **effort_controller**
   - Claims: `[effort]`
   - Features: Direct torque control
   - Best for: Force control, impedance control

### Switching Controllers

```bash
# Stop current controller
ros2 control set_controller_state zordi_mit_controller inactive

# Start different controller
ros2 control set_controller_state joint_trajectory_controller active
```

---

## Common Commands

### Monitor Joint States
```bash
ros2 topic echo /joint_states
```

### Check Controller Status
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### Inspect Trajectory Progress
```bash
# For action-based control
ros2 action list
ros2 action info /zordi_mit_controller/follow_joint_trajectory

# Check feedback
ros2 action send_goal /zordi_mit_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "<trajectory>" --feedback
```

### Debug MuJoCo Gravity
```bash
# Compare Pinocchio vs MuJoCo gravity compensation
ros2 topic echo /mujoco/qfrc_bias
```

### Restart Simulation
```bash
# Kill all nodes
pkill -9 -f "single_arm.launch.py"

# Relaunch
ros2 launch openarm_description single_arm.launch.py \
  default_controller:=zordi_mit_controller
```

---

## Troubleshooting

### Robot Drifts After Trajectory
**Problem**: Robot falls under gravity after trajectory completes.

**Solution**: Ensure `zordi_mit_controller` has `use_gravity_compensation: true` in YAML config.

### PID Gains Not Loading
**Problem**: Gains are 0.0, robot doesn't track.

**Solution**: Check URDF uses **underscores**, not dots:
```xml
<!-- CORRECT -->
<param name="position_kp">20.0</param>
<param name="velocity_kd">2.0</param>

<!-- WRONG -->
<param name="position.kp">20.0</param>
<param name="velocity.kd">2.0</param>
```

### Initial Pose Not Loading
**Problem**: Robot starts at wrong position.

**Solution**: Pass `initial_pose` and `initial_pose_config` to launch file:
```bash
ros2 launch openarm_description single_arm.launch.py \
  initial_pose:=home \
  initial_pose_config:=/path/to/poses.yaml
```

### Controller Won't Start
**Problem**: `zordi_mit_controller` fails to activate.

**Solution**: 
1. Check Pinocchio is installed: `python3 -c "import pinocchio"`
2. Check URDF is valid: `check_urdf <urdf_file>`
3. Rebuild: `colcon build --packages-select zordi_mit_controller`

### Trajectory Not Executing
**Problem**: Published trajectory but robot doesn't move.

**Solution**:
1. Check controller is active: `ros2 control list_controllers`
2. Verify topic/action name matches controller namespace
3. Ensure trajectory has valid joint names and time_from_start

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Controller                          │
│              (zordi_mit_controller)                         │
│  • Claims: [position, velocity, effort]                     │
│  • Computes gravity compensation (Pinocchio)                │
│  • Outputs: q_cmd, qd_cmd, τ_ff                            │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│              ROS2 Control Framework                         │
│  • Manages hardware interfaces                              │
│  • Enables controller switching                             │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│           MuJoCo Hardware Interface                         │
│              (mujoco_system.cpp)                            │
│  • Detects MIT mode (all 3 interfaces active)              │
│  • Computes: τ = Kp*e_pos + Kd*e_vel + τ_ff               │
│  • Applies to torque actuator                               │
│  • Neutralizes position/velocity actuators                  │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│              MuJoCo Physics Engine                          │
│  • Simulates robot dynamics                                 │
│  • Returns: q, qd, τ                                        │
└─────────────────────────────────────────────────────────────┘
```

---

## Key Files

### Configuration
- `openarm_config/config/zordi_mit_controller.yaml` - Controller config
- `openarm_description/urdf/openarm_v10.urdf` - Robot URDF with PID gains
- `openarm_description/mujoco_models/openarm_v10.xml` - MuJoCo model

### Launch Files
- `openarm_description/launch/single_arm.launch.py` - Main launch file

### Source Code
- `mujoco_ros2_control/src/mujoco_system.cpp` - Hardware interface
- `zordi_mit_controller/src/zordi_mit_controller.cpp` - MIT controller

### Documentation
- `README.md` - This file
- `PROJECT_HISTORY.md` - Development history & validation
- `TECHNICAL_REFERENCE.md` - Implementation details

---

## Next Steps

### For New Users
1. Run the quick start commands above
2. Send a simple trajectory
3. Monitor joint states
4. Experiment with different controllers

### For Developers
1. Read `TECHNICAL_REFERENCE.md` for architecture details
2. Review `PROJECT_HISTORY.md` for validation approach
3. Modify PID gains in URDF for your application
4. Implement custom controllers using `zordi_mit_controller` as template

### For Troubleshooting
1. Check this README's troubleshooting section
2. Review logs: `ros2 launch ... 2>&1 | tee launch.log`
3. Consult `PROJECT_HISTORY.md` for similar issues solved

---

## License

[Add your license here]

## Contact

[Add your contact information]

---

**Last validated**: November 13, 2025  
**System status**: Production ready ✅  
**Documentation version**: 1.0

