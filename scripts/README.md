# OpenARM Scripts

**Purpose**: Essential scripts for testing and utilities  
**Last Updated**: November 13, 2025

---

## Testing Scripts

### 1. test_gravity_compensation.py (28KB)

**Purpose**: Test gravity compensation with zordi_mit_controller

**Usage**:
```bash
# Terminal 1: Start simulation
cd ~/ros2_ws
source install/setup.bash
ros2 launch openarm_description single_arm.launch.py \
  default_controller:=zordi_mit_controller \
  headless:=true

# Terminal 2: Run test
cd ~/ros2_ws/src/openarm_description/scripts
python3 test_gravity_compensation.py
```

**What it tests**:
- Gravity compensation accuracy
- Hold stability (measures drift over time)
- Pinocchio vs MuJoCo gravity torque comparison
- Controller response to disturbances

**Expected output**:
```
Gravity compensation test:
  Duration: 10 seconds
  Max drift: < 0.01 rad
  Status: PASS
```

---

### 2. test_trajectory_tracking.py (19KB)

**Purpose**: Test trajectory tracking performance

**Usage**:
```bash
# Terminal 1: Start simulation
ros2 launch openarm_description single_arm.launch.py \
  default_controller:=zordi_mit_controller

# Terminal 2: Run test
python3 test_trajectory_tracking.py
```

**What it tests**:
- Trajectory following accuracy (RMSE)
- Position and velocity tracking
- MIT mode PD composition
- Maximum tracking error

**Expected output**:
```
Trajectory tracking test:
  RMSE: < 0.005 rad
  Max error: < 0.01 rad
  Status: PASS
```

---

## Utility Scripts

### 3. gravity_compensation_controller.py (35KB)

**Purpose**: Standalone gravity compensation controller (reference implementation)

**Usage**:
```bash
# Terminal 1: Start simulation without controller
ros2 launch openarm_description single_arm.launch.py \
  load_controllers:=false

# Terminal 2: Run standalone controller
python3 gravity_compensation_controller.py
```

**Features**:
- Pure Python implementation
- Pinocchio-based gravity torque computation
- Live matplotlib visualization
- Direct torque publishing to `/effort_controller/commands`

**Use cases**:
- Understanding Pinocchio integration
- Debugging gravity compensation
- Educational reference
- Prototyping control algorithms

**Note**: This is a standalone controller, not integrated with ROS2 Control. For production use, use `zordi_mit_controller` instead.

---

### 4. urdf_to_mjcf.py (25KB)

**Purpose**: Convert URDF to MuJoCo MJCF format

**Usage**:
```bash
python3 urdf_to_mjcf.py <input.urdf> <output.xml>
```

**Example**:
```bash
python3 urdf_to_mjcf.py \
  ../urdf/openarm_v10.urdf \
  ../mujoco_models/openarm_v10_converted.xml
```

**Features**:
- Converts meshes, inertias, joints, limits
- Generates actuators automatically (position, velocity, torque)
- Validates conversion
- Handles complex URDF structures

**Use cases**:
- Creating MuJoCo models from URDF
- Updating robot models
- Prototyping new designs

---

### 5. example_position_control.py (5.1KB)

**Purpose**: Simple example for new users

**Usage**:
```bash
# Terminal 1: Start simulation
ros2 launch openarm_description single_arm.launch.py

# Terminal 2: Run example
python3 example_position_control.py
```

**What it demonstrates**:
- Basic ROS2 publisher setup
- Sending position commands to controllers
- Simple sine wave motion
- Reading joint states

**Target audience**: New users learning ROS2 Control

---

### 6. restart_simulation.sh (579B)

**Purpose**: Quickly restart simulation

**Usage**:
```bash
./restart_simulation.sh
```

**What it does**:
1. Kills all running simulation nodes
2. Cleans up processes
3. Relaunches with default settings

**Use cases**:
- Rapid iteration during development
- Recovering from stuck simulations
- Quick testing cycles

---

## Quick Reference

### Test Everything
```bash
# 1. Start simulation
ros2 launch openarm_description single_arm.launch.py \
  default_controller:=zordi_mit_controller \
  headless:=true

# 2. Test gravity compensation
python3 test_gravity_compensation.py

# 3. Test trajectory tracking
python3 test_trajectory_tracking.py
```

### Common Issues

#### Script can't find ROS2
**Problem**: `ros2: command not found`

**Solution**: Source ROS2 environment
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

#### Python import errors
**Problem**: `ModuleNotFoundError: No module named 'pinocchio'`

**Solution**: Install ROS2 Pinocchio
```bash
sudo apt install ros-humble-pinocchio
```

#### Simulation not running
**Problem**: Scripts can't connect to topics

**Solution**: Start simulation first
```bash
ros2 launch openarm_description single_arm.launch.py
```

---

## Script Dependencies

### Required Packages
```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Pinocchio (for gravity compensation)
sudo apt install ros-humble-pinocchio

# Python packages
pip install mujoco matplotlib numpy
```

### ROS2 Topics/Services Used

**test_gravity_compensation.py**:
- Subscribe: `/joint_states`
- Subscribe: `/mujoco/qfrc_bias`

**test_trajectory_tracking.py**:
- Subscribe: `/joint_states`
- Action client: `/zordi_mit_controller/follow_joint_trajectory`

**gravity_compensation_controller.py**:
- Subscribe: `/joint_states`
- Publish: `/effort_controller/commands`

**example_position_control.py**:
- Subscribe: `/joint_states`
- Publish: `/position_controller/commands`

---

## Development Guidelines

### Adding New Test Scripts

1. **Follow naming convention**: `test_<feature>.py`
2. **Include documentation**: Docstring at top of file
3. **Use argparse**: For command-line options
4. **Return exit codes**: 0 for pass, 1 for fail
5. **Log results**: Clear pass/fail messages

**Template**:
```python
#!/usr/bin/env python3
"""
Test <feature> functionality.

Usage:
    python3 test_<feature>.py [--duration 10]
"""

import rclpy
from rclpy.node import Node

class FeatureTest(Node):
    def __init__(self):
        super().__init__('feature_test')
        # Setup

    def run_test(self):
        # Test logic
        if success:
            self.get_logger().info('✓ PASS')
            return 0
        else:
            self.get_logger().error('✗ FAIL')
            return 1

def main():
    rclpy.init()
    test = FeatureTest()
    exit_code = test.run_test()
    rclpy.shutdown()
    return exit_code

if __name__ == '__main__':
    exit(main())
```

### Adding New Utilities

1. **Document in this README**
2. **Include usage examples**
3. **Handle errors gracefully**
4. **Provide helpful error messages**

---

## Maintenance

### When to Update Scripts

- ✅ Robot model changes (URDF/MJCF)
- ✅ Controller interface changes
- ✅ ROS2 topic/action name changes
- ✅ Performance requirements change

### When to Remove Scripts

- ❌ Feature is deprecated
- ❌ Test is superseded by better test
- ❌ One-off diagnostic no longer needed

---

## See Also

- **Main Documentation**: `../docs/README.md` - Quick start guide
- **Technical Reference**: `../docs/TECHNICAL_REFERENCE.md` - Implementation details
- **Project History**: `../docs/PROJECT_HISTORY.md` - Development record

---

**Script Status**: Production Ready ✅  
**Total Scripts**: 6  
**Last Cleanup**: November 13, 2025

