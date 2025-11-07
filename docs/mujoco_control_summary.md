# MuJoCo Control Implementation Summary

## Overview
Successfully implemented stable position control for OpenARM in MuJoCo simulation with dynamic controller switching capability.

## Key Fixes Implemented

### 1. **PID Gain Tuning** (Critical Fix)
**Problem**: Original gains (Kp=2000-3000) were 100x too high, causing:
- Torques exceeding actuator limits by 300x (12,000 Nm vs 40 Nm limit)
- Massive instability and oscillations
- Robot spinning out of control even without gravity

**Solution**: Reduced PID gains dramatically:
```yaml
Position Control:
  - Joint 1-2: Kp=20.0, Kd=2.0, Ki=0.0  (actuator limit: 40 Nm)
  - Joint 3-4: Kp=15.0, Kd=1.5, Ki=0.0  (actuator limit: 27 Nm)
  - Joint 5-7: Kp=5.0,  Kd=0.5, Ki=0.0  (actuator limit: 7 Nm)

Velocity Control:
  - Kp = position_kp / 100.0
  - Kd = position_kd / 100.0
  - Ki = 0.0
```

**Result**: Robot now holds stable at zero position with < 0.1 degree error (except joint 6: -1.4 degrees)

### 2. **Initial Control Application**
**Problem**: `position_command_active` was initialized to `false`, so position control didn't apply until a controller sent commands. Robot would fall during this delay.

**Solution**: Initialize `position_command_active = true` in `register_joints()`:
```cpp
last_joint_state.position_command_active = true;  // Start holding immediately
```

### 3. **Controller Conflict Resolution**
**Problem**: With `control_mode="all"`, all three control interfaces (position, velocity, effort) were enabled and fighting each other, overwriting torques.

**Solution**: Modified control application logic to disable position when velocity/effort are active:
```cpp
bool apply_position = joint_state.is_position_control_enabled &&
                      (control_mode_ == "position" ||
                       (control_mode_ == "all" && joint_state.position_command_active &&
                        !joint_state.velocity_command_active && !joint_state.effort_command_active));
```

### 4. **Clock Synchronization Fix**
**Problem**: Non-monotonic time causing RViz resets due to threading race condition.

**Solution**: 
- Moved `mj_step1` to beginning of `update()` so time is read after physics step
- Added mutex-protected monotonic clock check in `publish_sim_time()`

### 5. **Library Path Fix**
**Problem**: Symbol lookup error when loading velocity_controller: `undefined symbol: _ZN20controller_interface23ControllerInterfaceBaseD2Ev`

**Solution**: Export `LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH` before launching

## Current Status

### ✅ Working
- **Position Control**: Robot holds stable at commanded positions with gravity
  - Trajectory following works (tested: joint1 from 0 to 0.5 rad)
  - Error: < 0.1 degrees for most joints
  - Joint 6 has -1.4 degree sag (acceptable, may need higher Ki for gravity compensation)
- **Controller Switching**: Can switch between position/velocity/effort controllers at runtime
- **Dynamic Switching**: All controllers load as inactive, can activate any one
- **Clock Publishing**: Monotonic time, no RViz resets

### ⚠️ Partially Working
- **Velocity Control**: 
  - Controller activates correctly
  - Position control disables when velocity is active ✅
  - BUT: Velocity commands cause instability/jumping
  - Issue: Velocity PID gains may still be too high, or velocity control inherently difficult in MuJoCo

### ❌ Not Tested
- **Effort Control**: Not yet tested
- **Bimanual Control**: Only single arm tested
- **Hand Control**: Not tested

## Files Modified

### 1. `mujoco_ros2_control/src/mujoco_system.cpp`
- Added `position_command_active = true` initialization
- Modified control application logic to prevent conflicts
- Added active command detection
- Added debug logging (should be removed for production)

### 2. `mujoco_ros2_control/src/mujoco_ros2_control.cpp`
- Moved `mj_step1` to beginning of `update()`
- Added mutex-protected monotonic clock in `publish_sim_time()`

### 3. `openarm_description/urdf/ros2_control/openarm.ros2_control.xacro`
- Hardcoded `control_mode="all"` for dynamic switching
- Changed to `position_pid` and `velocity_pid` command interfaces
- Reduced PID gains dramatically (Kp: 5-20, Kd: 0.5-2.0)
- Set Ki=0.0 to prevent integral windup

### 4. `openarm_description/launch/mujoco_sim.launch.py`
- Modified to load all three controllers as inactive
- Activate only the one specified by `control_mode` argument
- Added `mujoco_model_path` argument for custom models

## Usage

### Basic Launch
```bash
# Position control (default)
ros2 launch openarm_description mujoco_sim.launch.py control_mode:=position

# With custom model (e.g., no gravity)
ros2 launch openarm_description mujoco_sim.launch.py \
  control_mode:=position \
  mujoco_model_path:=/path/to/custom_model.xml
```

### Important: Set LD_LIBRARY_PATH
To avoid symbol lookup errors, always set:
```bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
```

### Send Position Trajectory
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [openarm_joint1, openarm_joint2, openarm_joint3, openarm_joint4, openarm_joint5, openarm_joint6, openarm_joint7]
  points:
  - positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 3, nanosec: 0}
"
```

### Switch to Velocity Control
```bash
# Switch controllers
ros2 control switch_controllers \
  --deactivate joint_trajectory_controller \
  --activate velocity_controller

# Send velocity command (continuous publishing required)
ros2 topic pub /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" \
  --rate 10
```

## Known Issues

### 1. Velocity Control Instability
**Symptom**: Robot jumps/spins wildly when velocity commands are sent
**Cause**: Velocity PID gains may still be too high, or velocity control fundamentally difficult with PID in MuJoCo
**Workaround**: Use very small velocity commands (< 0.1 rad/s) or use position control for trajectories
**TODO**: Investigate MuJoCo's native velocity actuators instead of PID

### 2. Symbol Lookup Error
**Symptom**: `undefined symbol: _ZN20controller_interface23ControllerInterfaceBaseD2Ev`
**Cause**: Library path not set when dynamically loading controllers
**Workaround**: Export `LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH` before launch
**TODO**: Fix in launch file or package configuration

### 3. Joint 6 Sag
**Symptom**: Joint 6 sags to -1.4 degrees under gravity
**Cause**: PD control without integral term cannot compensate for constant gravity torque
**Workaround**: Acceptable for most use cases
**TODO**: Tune Ki carefully to add gravity compensation without integral windup

## Next Steps

1. **Fix Velocity Control**: 
   - Try even lower gains (Kp/1000?)
   - Consider using MuJoCo's native velocity actuators
   - Add velocity clamping/limiting

2. **Test Effort Control**: Verify effort controller works

3. **Remove Debug Logging**: Clean up debug prints in `mujoco_system.cpp`

4. **Tune Joint 6**: Add small Ki for gravity compensation

5. **Test Bimanual**: Verify both arms work independently

6. **Permanent LD_LIBRARY_PATH Fix**: Update package configuration or launch file

## Technical Notes

### Why PID for Position/Velocity?
MuJoCo supports two control modes:
- **Direct (kinematic)**: Set `qpos`/`qvel` directly - ignores physics, breaks with gravity
- **PID (dynamic)**: Compute torques via PID - respects physics, requires tuning

We use PID because we want realistic dynamics with gravity.

### Why `position_pid` and `velocity_pid` names?
The `_pid` suffix is a convention in `mujoco_ros2_control` to explicitly enable PID control. Without it, the system tries to set `qpos`/`qvel` directly, which causes instability with gravity.

### Why Ki=0?
Integral gain causes "integral windup" - the integral term accumulates large errors during startup or disturbances, leading to massive torques and instability. We disable it for now. Proper anti-windup mechanisms would be needed to use Ki safely.

### Why such low gains?
The actuator limits are relatively low (7-40 Nm). With error potentially reaching several radians during startup, we need `Kp * max_error < actuator_limit`. For safety, we use very conservative gains.

## Comparison with Main Branch

**Main Branch (before changes)**:
- Single control mode only (position OR velocity OR effort)
- No dynamic switching
- Higher PID gains (caused instability)
- Direct qpos setting without PID (broke with gravity)

**Current Implementation**:
- Dynamic switching between all three modes ✅
- Much lower, stable PID gains ✅
- Proper PID-based control ✅
- Robot holds position with gravity ✅
- Velocity control needs more work ⚠️

