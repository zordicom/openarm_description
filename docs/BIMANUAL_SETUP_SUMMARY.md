# Bimanual Setup Summary

**Created:** November 18, 2025
**Configuration:** OpenARM v10 Bimanual (14 DOF - Two arms without hands)

---

## Files Created/Modified

### 1. MuJoCo XML Model

**File:** `mujoco_models/openarm_v10_bimanual.xml`

**Status:** ✅ Fixed actuator joint names

**Changes:**
- Fixed actuator joint names to match URDF convention
- Changed from `openarm_joint*` to `openarm_left_joint*` and `openarm_right_joint*`
- Updated control ranges to match URDF joint limits
- Added proper left/right arm separation in actuator definitions

**Key Details:**
- Total actuators: 42 (21 per arm: 3 per joint × 7 joints)
- Actuator types per joint: position, velocity, motor (torque)
- Joint names properly prefixed with `left_` and `right_`

---

### 2. Controller Configuration

**File:** `config/mujoco/controllers_bimanual_multimode_test.yaml`

**Status:** ✅ Created

**Configuration:**
- 14 total controllers (7 per arm)
- Independent control for each arm
- All controllers start inactive

**Left Arm Controllers:**
1. `left_joint_trajectory_controller` - Standard ROS2 position+velocity
2. `left_zordi_hardware_pd_controller` - Hardware PD (MIT mode)
3. `left_zordi_software_pd_controller` - Software PD (effort only)
4. `left_zordi_grav_comp_controller` - Gravity compensation only
5. `left_zordi_mit_rnea_controller` - Full inverse dynamics
6. `left_zordi_cartesian_controller` - Cartesian impedance
7. `left_zordi_cartesian_rnea_controller` - Advanced Cartesian

**Right Arm Controllers:**
1. `right_joint_trajectory_controller` - Standard ROS2 position+velocity
2. `right_zordi_hardware_pd_controller` - Hardware PD (MIT mode)
3. `right_zordi_software_pd_controller` - Software PD (effort only)
4. `right_zordi_grav_comp_controller` - Gravity compensation only
5. `right_zordi_mit_rnea_controller` - Full inverse dynamics
6. `right_zordi_cartesian_controller` - Cartesian impedance
7. `right_zordi_cartesian_rnea_controller` - Advanced Cartesian

**Key Parameters:**
- Update rate: 1000 Hz (1 ms timestep)
- Torque limits: [200.0, 200.0, 150.0, 150.0, 50.0, 50.0, 50.0] per arm
- Cartesian stiffness: 500.0 N/m (translation), 50.0 Nm/rad (rotation)
- Nullspace stiffness: 10.0 (basic), 20.0 (RNEA)

---

### 3. Launch File

**File:** `launch/test_openarm_bimanual_multimode.launch.py`

**Status:** ✅ Created

**Features:**
- Loads bimanual URDF and MuJoCo XML
- Spawns robot_state_publisher for TF and gravity compensation
- Loads all 14 controllers (inactive) plus joint_state_broadcaster (active)
- Uses ExecuteProcess for controller loading (avoids parameter visibility issues)
- Configurable initial keyframe and headless mode

**Launch Arguments:**
- `initial_keyframe` (default: "home")
- `headless` (default: "false")

**Usage:**
```bash
ros2 launch openarm_description test_openarm_bimanual_multimode.launch.py
```

---

### 4. Quick Start Documentation

**File:** `docs/CARTESIAN_QUICK_START_BIMANUAL.md`

**Status:** ✅ Created

**Contents:**
- Overview of bimanual setup
- Quick launch instructions
- Configuration requirements
- Controller list and descriptions
- Usage examples for both arms
- Joint naming conventions
- Control modes explanation
- Troubleshooting guide
- File references
- Controller switching examples
- Tips for bimanual control

---

## System Architecture

### Joint Naming Convention

```
Left Arm:                    Right Arm:
- openarm_left_joint1       - openarm_right_joint1
- openarm_left_joint2       - openarm_right_joint2
- openarm_left_joint3       - openarm_right_joint3
- openarm_left_joint4       - openarm_right_joint4
- openarm_left_joint5       - openarm_right_joint5
- openarm_left_joint6       - openarm_right_joint6
- openarm_left_joint7       - openarm_right_joint7
```

### Frame Naming Convention

```
Left Arm:                    Right Arm:
- openarm_left_link0        - openarm_right_link0
- openarm_left_link1        - openarm_right_link1
- openarm_left_link2        - openarm_right_link2
- openarm_left_link3        - openarm_right_link3
- openarm_left_link4        - openarm_right_link4
- openarm_left_link5        - openarm_right_link5
- openarm_left_link6        - openarm_right_link6
- openarm_left_link7        - openarm_right_link7 (end-effector)
```

### Hardware Interfaces

**URDF ros2_control tags:**
- `openarm_left_hardware_interface` (7 joints)
- `openarm_right_hardware_interface` (7 joints)

**Command interfaces per joint:**
- `position_pid` (Hardware PD position control)
- `velocity_pid` (Hardware PD velocity control)
- `effort` (Direct torque control)

**State interfaces per joint:**
- `position`
- `velocity`
- `effort`

---

## Testing Checklist

### Phase 1: System Launch
- [ ] Launch file runs without errors
- [ ] MuJoCo viewer opens correctly
- [ ] Robot appears in correct pose
- [ ] Both arms are visible

### Phase 2: Controller Loading
- [ ] All 14 controllers load successfully
- [ ] `joint_state_broadcaster` is active
- [ ] All other controllers are inactive
- [ ] No parameter errors in logs

### Phase 3: Left Arm Testing
- [ ] Activate `left_zordi_cartesian_controller`
- [ ] Send target pose to left arm
- [ ] Verify smooth motion
- [ ] Check nullspace behavior
- [ ] Test controller switching

### Phase 4: Right Arm Testing
- [ ] Activate `right_zordi_cartesian_controller`
- [ ] Send target pose to right arm
- [ ] Verify smooth motion
- [ ] Check nullspace behavior
- [ ] Test controller switching

### Phase 5: Bimanual Testing
- [ ] Activate both Cartesian controllers
- [ ] Send synchronized poses
- [ ] Test independent motion
- [ ] Verify collision avoidance
- [ ] Test coordinated tasks

### Phase 6: Advanced Testing
- [ ] Test gravity compensation mode on both arms
- [ ] Test RNEA controllers
- [ ] Test mixed control modes
- [ ] Test rapid controller switching
- [ ] Monitor torque limits

---

## Key Differences from Single Arm

1. **Controller Naming:** All controllers are prefixed with `left_` or `right_`
2. **Frame References:** All frames are prefixed with `openarm_left_` or `openarm_right_`
3. **Joint Naming:** All joints are prefixed with `left_` or `right_`
4. **Independent Control:** Each arm can use a different controller type
5. **Separate Hardware Interfaces:** Two ros2_control interfaces in URDF
6. **Doubled Controllers:** 14 total controllers instead of 7

---

## Related Files

- Single arm config: `config/mujoco/controllers_multimode_test.yaml`
- Single arm launch: `launch/test_openarm_multimode.launch.py`
- Single arm docs: `docs/CARTESIAN_QUICK_START.md`
- Single arm URDF: `mujoco_models/openarm_v10.urdf`
- Single arm XML: `mujoco_models/openarm_v10.xml`

---

## Next Steps

1. Build the workspace:
   ```bash
   cd /home/gilwoo/ros2_ws
   colcon build --packages-select openarm_description
   source install/setup.bash
   ```

2. Test the launch file:
   ```bash
   ros2 launch openarm_description test_openarm_bimanual_multimode.launch.py
   ```

3. Verify controller loading:
   ```bash
   ros2 control list_controllers
   ```

4. Test individual arm control

5. Test coordinated bimanual control

---

## Notes

- The bimanual URDF already existed but had incorrect actuator names in the XML
- The XML file has been fixed to match the URDF joint naming convention
- All new files follow the same patterns as the single-arm configuration
- Controllers use the same parameters as single-arm, just duplicated per arm
- The setup supports fully independent control of each arm

