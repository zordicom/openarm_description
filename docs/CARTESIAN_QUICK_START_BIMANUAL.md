# Cartesian Quick Start - Right Arm (Bimanual Config)

**Quick reference for the right arm with bimanual shoulder mounting.**

For comprehensive Cartesian control documentation, see: [`zordi_ros_controllers/docs/OPENARM_CARTESIAN_EXAMPLE.md`](../../../zordi_ros_controllers/docs/OPENARM_CARTESIAN_EXAMPLE.md)

---

## Quick Launch

```bash
# Launch right arm (simulation auto-starts)
ros2 launch openarm_description test_openarm_bimanual_multimode.launch.py

# Activate Cartesian controller
ros2 control set_controller_state right_zordi_cartesian_mit_rnea_controller active

# Send target pose
ros2 topic pub --once /right_zordi_cartesian_mit_rnea_controller/target_pose \
  geometry_msgs/msg/PoseStamped "{
    header: {frame_id: 'openarm_right_link0'},
    pose: {
      position: {x: 0.216, y: -0.156, z: 0.278},
      orientation: {x: 0.271, y: 0.653, z: 0.271, w: 0.653}
    }
  }"
```

---

## Bimanual-Specific Configuration

| Parameter | Value |
|-----------|-------|
| Base frame | `openarm_right_link0` |
| End-effector frame | `openarm_right_link7` |
| Config file | `config/mujoco/controllers_bimanual_multimode_test.yaml` |
| Launch file | `test_openarm_bimanual_multimode.launch.py` |

---

## Available Controllers

| Controller | Description |
|------------|-------------|
| `right_zordi_cartesian_mit_controller` | Cartesian impedance (MIT mode) |
| `right_zordi_cartesian_mit_rnea_controller` | Cartesian + RNEA feedforward |
| `right_zordi_joint_mit_controller` | Joint trajectory (MIT mode) |
| `right_zordi_joint_mit_rnea_controller` | Joint + RNEA feedforward |
| `right_zordi_joint_effort_grav_comp_controller` | Gravity compensation only |

---

## Keyframes (`openarm_v10_right_arm_proper.xml`)

| Name | qpos | Cartesian Pose (x, y, z) |
|------|------|--------------------------|
| home | 0, 0.785, 0, 1.57, 0, 0, 0 | 0.216, -0.156, 0.278 |
| extended | 0, 0.785, 0, 1.0, 0, 0, 0 | 0.182, -0.238, 0.360 |
| canonical | 0, 0, 0, 0, 0, 0, 0 | 0.0, -0.436, 0.123 |

```bash
# Reset to keyframe
ros2 service call /mujoco_ros2_control/reset_to_keyframe \
  mujoco_ros2_control_msgs/srv/ResetToKeyframe "{keyframe: 'extended'}"
```

---

## Action-Based Control

```bash
ros2 action send_goal --feedback /right_zordi_cartesian_mit_rnea_controller/follow_cartesian_trajectory \
  zordi_ros_controllers_msgs/action/FollowCartesianTrajectory \
  "{trajectory: {points: [{point: {pose: {position: {x: 0.216, y: -0.156, z: 0.278}, orientation: {x: 0.271, y: 0.653, z: 0.271, w: 0.653}}}, time_from_start: {sec: 3}}]}}"
```

---

**See also:** [TESTING_GUIDE.md](TESTING_GUIDE.md) for comprehensive testing documentation.
