#!/bin/bash
# Quick test - call service and show result immediately

echo "Calling external wrench service..."
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
echo "Check the response above. If it shows 'accepted: true', the service worked."
echo "Also check your launch terminal for these messages:"
echo "  - 'apply_external_wrench: body=...'"
echo "  - 'Applying external wrench: body_id=...'"
