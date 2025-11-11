#!/bin/bash
# Simple shell script to test external wrench service

echo "Testing external wrench service with ros2 service call..."
echo "This should make the robot move DOWN for 2 seconds"
echo ""

ros2 service call /apply_external_wrench mujoco_ros2_control_msgs/srv/ApplyExternalWrench "{
  body_name: 'openarm_link7',
  wrench: {
    force: {x: 0.0, y: 0.0, z: -50.0},
    torque: {x: 0.0, y: 0.0, z: 0.0}
  },
  duration: 2.0
}"

echo ""
echo "If the robot moved, the service works!"
echo "If nothing happened, check MuJoCo logs for errors."
