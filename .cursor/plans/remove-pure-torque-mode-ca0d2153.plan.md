<!-- ca0d2153-874f-4051-bc88-37a7aa53900c 8884a750-94d1-4865-89aa-a968f534b458 -->
# Controller and Test Naming Convention Refactor

## Naming Convention

```
Controllers: zordi_[joint/cartesian]_[mit/mit_effort/torque_motor][_rnea]_controller
Tests:       test_[gravity/planar]_[joint/cartesian]_[mit/mit_effort/torque_motor][_rnea].test.py
```

| Mode | `actuator_type` | hardware_kp/kd | software_kp/kd |

|------|-----------------|----------------|----------------|

| `mit` | `"mit"` | non-zero | zero |

| `mit_effort` | `"mit"` | zero | non-zero |

| `torque_motor` | `"torque_motor"` | N/A | non-zero |

## Phase 1: Rename Controllers in Config Files

### [test_controllers.yaml](zordi_ros_controllers/zordi_ros_controllers/test/config/test_controllers.yaml) (gravity model)

| Current | New |

|---------|-----|

| `zordi_joint_effort_controller` | `zordi_joint_mit_effort_controller` |

| `zordi_joint_effort_rnea_controller` | `zordi_joint_mit_effort_rnea_controller` |

| `zordi_joint_torque_controller` | `zordi_joint_torque_motor_controller` |

| `zordi_joint_torque_rnea_controller` | `zordi_joint_torque_motor_rnea_controller` |

| `zordi_cartesian_effort_controller` | `zordi_cartesian_mit_effort_controller` |

| `zordi_cartesian_effort_rnea_controller` | `zordi_cartesian_mit_effort_rnea_controller` |

### [test_planar_controllers.yaml](zordi_ros_controllers/zordi_ros_controllers/test/config/test_planar_controllers.yaml)

Same renames as above, plus add missing `torque_motor` controllers.

### [test_single_motor_controllers.yaml](zordi_ros_controllers/zordi_ros_controllers/test/config/test_single_motor_controllers.yaml)

| Current | New |

|---------|-----|

| `zordi_joint_mit_all_controller` | `zordi_joint_mit_controller` |

| `zordi_joint_mit_effort_only_controller` | `zordi_joint_mit_effort_controller` |

| `zordi_joint_effort_controller` | `zordi_joint_torque_motor_controller` |

## Phase 2: Rename Test Files

### Gravity Model Tests

| Current | New |

|---------|-----|

| `test_joint_mit_trajectory.test.py` | `test_gravity_joint_mit.test.py` |

| `test_joint_mit_rnea.test.py` | `test_gravity_joint_mit_rnea.test.py` |

| `test_joint_effort_trajectory.test.py` | `test_gravity_joint_mit_effort.test.py` |

| `test_joint_effort_rnea.test.py` | `test_gravity_joint_mit_effort_rnea.test.py` |

| `test_joint_torque_trajectory.test.py` | `test_gravity_joint_torque_motor.test.py` |

| `test_joint_torque_rnea.test.py` | `test_gravity_joint_torque_motor_rnea.test.py` |

| `test_cartesian_effort.test.py` | `test_gravity_cartesian_mit_effort.test.py` |

| `test_cartesian_effort_rnea.test.py` | `test_gravity_cartesian_mit_effort_rnea.test.py` |

| `test_cartesian_mit_rnea.test.py` | `test_gravity_cartesian_mit_rnea.test.py` |

| `test_cartesian_ik.test.py` | `test_gravity_cartesian_ik.test.py` |

| `test_gravity_compensation.test.py` | `test_gravity_joint_grav_comp.test.py` |

### Planar Model Tests

| Current | New |

|---------|-----|

| `test_planar_joint_mit_trajectory.test.py` | `test_planar_joint_mit.test.py` |

| `test_planar_joint_mit_rnea.test.py` | `test_planar_joint_mit_rnea.test.py` |

| `test_planar_joint_effort_trajectory.test.py` | `test_planar_joint_mit_effort.test.py` |

| `test_planar_joint_effort_rnea.test.py` | `test_planar_joint_mit_effort_rnea.test.py` |

| `test_planar_cartesian_effort.test.py` | `test_planar_cartesian_mit_effort.test.py` |

| `test_planar_cartesian_effort_rnea.test.py` | `test_planar_cartesian_mit_effort_rnea.test.py` |

| `test_planar_cartesian_mit_rnea.test.py` | `test_planar_cartesian_mit_rnea.test.py` |

| `test_planar_cartesian_ik.test.py` | `test_planar_cartesian_ik.test.py` |

### Single Motor Tests

| Current | New |

|---------|-----|

| `test_single_motor_mit.test.py` | `test_single_motor_mit.test.py` |

| `test_single_motor_mit_effort_only.test.py` | `test_single_motor_mit_effort.test.py` |

| `test_single_motor_effort.test.py` | `test_single_motor_torque_motor.test.py` |

| `test_single_motor_position_servo.test.py` | (keep as-is, ROS native) |

### Other Tests (keep as-is)

- `test_cancel_handlers.test.py`
- `test_ros_native_comparison.test.py`

## Phase 3: Create Missing Tests

| New Test | Description |

|----------|-------------|

| `test_gravity_cartesian_mit.test.py` | Gravity, Cartesian MIT hardware PD (non-RNEA) |

| `test_planar_cartesian_mit.test.py` | Planar, Cartesian MIT hardware PD (non-RNEA) |

| `test_planar_joint_torque_motor.test.py` | Planar, Joint torque motor |

| `test_planar_joint_torque_motor_rnea.test.py` | Planar, Joint torque motor + RNEA |

## Phase 4: Update References

Update all internal references in:

- Test files (controller names in launch configs)
- CMakeLists.txt (test file names)
- README.md and STATUS.md documentation

## Files to Modify

- Config: 3 files
- Tests: ~22 files (rename + update controller refs)
- New tests: 4 files
- Build: CMakeLists.txt
- Docs: README.md, STATUS.md, test/README.md

### To-dos

- [ ] Update mujoco_system.cpp to require kp/kd when effort is claimed
- [ ] Delete examples/torque_motor/ directory
- [ ] Delete test_torque_motor.test.py and test model files
- [ ] Update README and documentation with new configuration
- [ ] Update test_interface_validation.test.py
- [ ] Update test_single_motor_effort.urdf to add kp/kd interfaces
- [ ] Update zordi_joint_effort_controller to use MIT mode with kp=kd=0
- [ ] Update zordi_joint_controller.cpp to claim kp/kd in software PD mode
- [ ] Update test controller configs to claim pos/vel/effort interfaces
- [ ] Update compute_pd_internally parameter description
- [ ] Update zordi_joint_controller.cpp with implicit mode detection
- [ ] Update mujoco_system.cpp to require kp/kd when effort is claimed
- [ ] Delete examples/torque_motor/ directory
- [ ] Delete test_torque_motor.test.py and test model files
- [ ] Update ACTUATOR_TYPES.md, README.md, and updates.md
- [ ] Update test_interface_validation.test.py
- [ ] Update zordi_joint_controller.cpp to claim kp/kd in software PD mode
- [ ] Update parameter definition: remove flag, rename default_kp/kd to kp/kd
- [ ] Update mujoco_system.cpp to require kp/kd when effort is claimed
- [ ] Delete examples/torque_motor/ directory
- [ ] Delete test_torque_motor.test.py and test model files
- [ ] Update ACTUATOR_TYPES.md, README.md, and updates.md
- [ ] Update test_interface_validation.test.py
- [ ] Update test_single_motor_effort.urdf to add kp/kd interfaces
- [ ] Update zordi_joint_effort_controller to use MIT mode with kp=kd=0
- [ ] Update zordi_joint_controller.cpp to claim kp/kd in software PD mode
- [ ] Update test controller configs to claim pos/vel/effort interfaces
- [ ] Update compute_pd_internally parameter description
- [ ] Update mujoco_system.cpp to require kp/kd when effort is claimed
- [ ] Delete examples/torque_motor/ directory
- [ ] Delete test_torque_motor.test.py and test model files
- [ ] Update test_interface_validation.test.py