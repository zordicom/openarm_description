<!-- 67d7b1df-d9d0-4678-81ce-770e8c8b7936 b1036203-1ffd-4ec9-93f3-0ffd4f936ae3 -->
# MIT Mode Support for OpenARM (DAMIAO Motors)

## Current State

All OpenARM tests currently use **effort-only mode with software PD** (`compute_pd_internally: true`) because the URDF lacks `kp`/`kd` command interfaces. The controller names contain "mit" but they actually run in software PD mode.

## Changes Required

### 1. URDF: Add kp/kd Command Interfaces

**File:** [openarm.ros2_control.xacro](openarm_description/urdf/ros2_control/openarm.ros2_control.xacro)

Update the `configure_joint` macro to add MIT mode interfaces:

```xml
<!-- Add after existing command interfaces -->
<command_interface name="kp"/>
<command_interface name="kd"/>
<!-- Safety limits for gains -->
<param name="max_kp">500.0</param>
<param name="max_kd">50.0</param>
```

**Also update:** [openarm.bimanual.ros2_control.xacro](openarm_description/urdf/ros2_control/openarm.bimanual.ros2_control.xacro) with same changes.

### 2. MuJoCo Models: Simplify to Motor-Only Actuators

For MIT mode, remove position/velocity actuators (hardware interface computes PD).

**Files to update:**

- [openarm_v10.xml](openarm_description/mujoco_models/openarm_v10.xml)
- [openarm_v10_bimanual.xml](openarm_description/mujoco_models/openarm_v10_bimanual.xml)
- [openarm_v10_hand.xml](openarm_description/mujoco_models/openarm_v10_hand.xml)
- [openarm_v10_bimanual_hand.xml](openarm_description/mujoco_models/openarm_v10_bimanual_hand.xml)

**Change from (per joint):**

```xml
<position name="act_pos_openarm_joint1" joint="openarm_joint1" kp="100.0" .../>
<velocity name="act_vel_openarm_joint1" joint="openarm_joint1" kv="10.0" .../>
<motor name="act_tau_openarm_joint1" joint="openarm_joint1" .../>
```

**To (motor only):**

```xml
<motor name="act_tau_openarm_joint1" joint="openarm_joint1" ctrlrange="-200 200" forcerange="-200 200"/>
```

### 3. Regenerate URDFs from Xacro

Run xacro to regenerate all URDF files after modifying the xacro sources.

### 4. Controller Config: Update for MIT Mode

**File:** [test_controllers.yaml](openarm_tests/test/config/test_controllers.yaml)

Update controllers to use true MIT mode:

```yaml
zordi_joint_mit_controller:
  ros__parameters:
    actuator_type: "mit"
    command_interfaces:
      - position
      - velocity
      - effort
    hardware_kp: [100.0, 100.0, 100.0, 100.0, 50.0, 50.0, 50.0]
    hardware_kd: [10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0]
    software_kp: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    software_kd: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

Add new MIT-effort controllers for software PD mode testing.

**Also update:** [test_bimanual_controllers.yaml](openarm_tests/test/config/test_bimanual_controllers.yaml)

### 5. Test File Renames (openarm_tests)

Rename to match zordi_ros_controllers naming convention:

**Single Arm:**

| Current | New |

|---------|-----|

| `test_gravity_compensation.test.py` | `test_joint_grav_comp.test.py` |

| `test_joint_trajectory.test.py` | `test_joint_mit_effort.test.py` |

| `test_joint_rnea.test.py` | `test_joint_mit_effort_rnea.test.py` |

| `test_cartesian_control.test.py` | `test_cartesian_mit_effort.test.py` |

| `test_cartesian_rnea.test.py` | `test_cartesian_mit_effort_rnea.test.py` |

| `test_cartesian_mit_rnea.test.py` | (keep - already correct) |

| `test_cartesian_ik.test.py` | (keep - already correct) |

**Bimanual:**

| Current | New |

|---------|-----|

| `test_bimanual_gravity_compensation.test.py` | `test_bimanual_joint_grav_comp.test.py` |

| `test_bimanual_joint_trajectory.test.py` | `test_bimanual_joint_mit_effort.test.py` |

| `test_bimanual_joint_rnea.test.py` | `test_bimanual_joint_mit_effort_rnea.test.py` |

| `test_bimanual_cartesian_control.test.py` | `test_bimanual_cartesian_mit_effort.test.py` |

| `test_bimanual_cartesian_rnea.test.py` | `test_bimanual_cartesian_mit_effort_rnea.test.py` |

| `test_bimanual_cartesian_mit_rnea.test.py` | (keep - already correct) |

| `test_bimanual_cartesian_ik.test.py` | (keep - already correct) |

**Keep unchanged:** `test_gain_safety_limits.test.py`, `test_actuator_modes.test.py`

---

## Missing Tests Assessment

Based on supported actuator modes (MIT hardware PD, MIT-effort software PD, RNEA variants):

### Currently Missing Tests (Single Arm)

| Controller | MIT (hw PD) | MIT-effort (sw PD) | Notes |

|------------|-------------|-------------------|-------|

| Joint trajectory | **MISSING** | Exists (renamed) | Need `test_joint_mit.test.py` |

| Joint RNEA | **MISSING** | Exists (renamed) | Need `test_joint_mit_rnea.test.py` |

| Cartesian control | **MISSING** | Exists (renamed) | Need `test_cartesian_mit.test.py` |

| Cartesian RNEA | Exists | Exists (renamed) | OK |

| Cartesian IK | Exists | N/A | OK |

| Grav Comp | Exists | N/A | OK |

### Currently Missing Tests (Bimanual)

Same gaps as single arm - need MIT hardware PD variants.

### New Tests to Add

**Single Arm (4 tests):**

- `test_joint_mit.test.py` - Joint trajectory with hardware PD
- `test_joint_mit_rnea.test.py` - Joint RNEA with hardware PD
- `test_cartesian_mit.test.py` - Cartesian control with hardware PD
- (cartesian_mit_rnea already exists)

**Bimanual (4 tests):**

- `test_bimanual_joint_mit.test.py`
- `test_bimanual_joint_mit_rnea.test.py`
- `test_bimanual_cartesian_mit.test.py`
- (bimanual_cartesian_mit_rnea already exists)

### To-dos

- [ ] Add kp/kd command interfaces and max_kp/max_kd params to URDF xacros
- [ ] Simplify MuJoCo XMLs to motor-only actuators (remove pos/vel actuators)
- [ ] Regenerate URDF files from xacro sources
- [ ] Update test_controllers.yaml for true MIT mode with hardware_kp/kd
- [ ] Rename openarm_tests files to match zordi_ros_controllers convention
- [ ] Add missing MIT hardware PD tests (joint_mit, joint_mit_rnea, cartesian_mit)