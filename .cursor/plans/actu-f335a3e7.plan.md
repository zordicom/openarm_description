<!-- f335a3e7-51a8-46e6-9974-993fd17dc257 6f567e52-0971-4f41-90a0-fb5ea29dc3ed -->
# Actuator Type Automated Tests

## Summary

Add layered automated tests for the three actuator types:

- **Position Servo** - position interface only (like Dynamixel)
- **Torque Motor** - effort interface only (like Kuka iiwa)
- **MIT Motor** - all 5 interfaces (like Damiao)

## Part 1: Simple Tests in mujoco_ros2_control

### 1.1 Add Test Infrastructure

**Files to modify:**

- [`mujoco_ros2_control/package.xml`](mujoco_ros2_control/mujoco_ros2_control/package.xml) - Add test dependencies
- [`mujoco_ros2_control/CMakeLists.txt`](mujoco_ros2_control/mujoco_ros2_control/CMakeLists.txt) - Add test configuration

**Test dependencies to add:**

```xml
<test_depend>launch_testing_ament_cmake</test_depend>
<test_depend>launch_testing_ros</test_depend>
<test_depend>forward_command_controller</test_depend>
```

### 1.2 Create Test Models (headless versions)

Create test URDFs with `mujoco_viewer: false` for CI:

- `test/models/test_position_servo.urdf` - Copy from examples, disable viewer
- `test/models/test_torque_motor.urdf` - Copy from examples, disable viewer
- `test/models/test_mit_motor.urdf` - Copy from examples, disable viewer

Symlink or copy the XML files from examples.

### 1.3 Create Test Config

**New file:** `test/config/test_actuator_types.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    forward_position_controller:
      type: forward_command_controller/ForwardCommandController
    forward_effort_controller:
      type: forward_command_controller/ForwardCommandController
```

### 1.4 Create Test Files

**Test 1:** `test/test_interface_validation.test.py`

- Verify position servo claims only `position` command interface
- Verify torque motor claims only `effort` command interface
- Verify MIT motor claims all 5 interfaces (`position`, `velocity`, `effort`, `kp`, `kd`)

**Test 2:** `test/test_position_servo.test.py`

- Use `forward_command_controller` to send position command
- Verify joint moves to commanded position (within tolerance)
- Verify position actuator PD works (no gravity compensation needed)

**Test 3:** `test/test_torque_motor.test.py`

- Use `forward_command_controller` to send effort command
- Verify joint responds to torque (pendulum swings under gravity if torque=0)
- Send gravity-compensating torque, verify hold

**Test 4:** `test/test_mit_motor.test.py`

- Verify kp/kd interfaces accept commands
- Verify gain safety limits (max_kp, max_kd) are enforced
- Test basic impedance response

## Part 2: Extend zordi_ros_controllers Tests

### 2.1 Single-Motor MIT Trajectory Test

**New file:** `test/test_single_motor_mit.test.py`

Using the existing MIT motor example model:

- Load `ZordiJointController` in MIT mode
- Send simple trajectory (0 -> 0.5 rad)
- Verify tracking error < 5 degrees
- Tests gravity compensation with single pendulum

### 2.2 Single-Motor Effort Trajectory Test

**New file:** `test/test_single_motor_effort.test.py`

Using the existing torque motor example model:

- Load `ZordiJointController` in effort mode (`compute_pd_internally: true`)
- Send simple trajectory
- Verify tracking with software PD + gravity compensation

### 2.3 Single-Motor Position Servo Test (ROS Native)

**New file:** `test/test_single_motor_position_servo.test.py`

Using the position servo example model with ROS-native `joint_trajectory_controller`:

- Load standard `joint_trajectory_controller` (position interface only)
- Send simple trajectory (0 -> 0.5 rad)
- Verify tracking (servo's internal PD handles it)
- Demonstrates ROS-native controller compatibility with MuJoCo position actuators

### 2.4 Test Config for Single Motor

**New file:** `test/config/test_single_motor_controllers.yaml`

```yaml
# Zordi MIT controller for single motor
zordi_joint_mit_controller:
  ros__parameters:
    joints: [joint1]
    command_interfaces: [position, velocity, effort]
    compute_pd_internally: false
    use_gravity_compensation: true
    default_kp: [100.0]
    default_kd: [10.0]

# Zordi effort controller for single motor
zordi_joint_effort_controller:
  ros__parameters:
    joints: [joint1]
    command_interfaces: [effort]
    compute_pd_internally: true
    use_gravity_compensation: true

# ROS-native JTC for position servo
joint_trajectory_controller:
  ros__parameters:
    joints: [joint1]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

## File Structure

```
mujoco_ros2_control/mujoco_ros2_control/test/
├── config/
│   └── test_actuator_types.yaml
├── models/
│   ├── test_position_servo.urdf
│   ├── test_torque_motor.urdf
│   └── test_mit_motor.urdf
├── test_interface_validation.test.py
├── test_position_servo.test.py
├── test_torque_motor.test.py
└── test_mit_motor.test.py

zordi_ros_controllers/zordi_ros_controllers/test/
├── config/
│   └── test_single_motor_controllers.yaml  (new)
├── test_single_motor_mit.test.py           (new)
├── test_single_motor_effort.test.py        (new)
└── test_single_motor_position_servo.test.py (new, ROS-native)
```

## Running Tests

```bash
# All mujoco_ros2_control tests
colcon test --packages-select mujoco_ros2_control

# All zordi_ros_controllers tests
colcon test --packages-select zordi_ros_controllers

# Single test
python3 -m launch_testing.launch_test \
    src/mujoco_ros2_control/mujoco_ros2_control/test/test_position_servo.test.py
```

### To-dos

- [ ] Add test dependencies to mujoco_ros2_control package.xml and CMakeLists.txt
- [ ] Create headless test URDFs for position_servo, torque_motor, mit_motor
- [ ] Create test_actuator_types.yaml with forward_command_controller configs
- [ ] Create test_interface_validation.test.py to verify correct interfaces per mode
- [ ] Create test_position_servo.test.py with forward command tests
- [ ] Create test_torque_motor.test.py with effort command tests
- [ ] Create test_mit_motor.test.py with gain interface and safety limit tests
- [ ] Create test_single_motor_controllers.yaml for zordi tests
- [ ] Create test_single_motor_mit.test.py trajectory test
- [ ] Create test_single_motor_effort.test.py trajectory test