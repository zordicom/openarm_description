#!/bin/bash
# Simple check to see if service responds at all

echo "Testing if service exists and is callable..."
echo ""

# Try with a very short timeout
timeout 5 ros2 service call /apply_external_wrench mujoco_ros2_control_msgs/srv/ApplyExternalWrench "{
  body_name: 'openarm_link7',
  wrench: {
    force: {x: 50.0, y: 50.0, z: -50.0},
    torque: {x: 0.0, y: 0.0, z: 0.0}
  },
  duration: 10.0
}"

EXIT_CODE=$?

echo ""
if [ $EXIT_CODE -eq 124 ]; then
    echo "❌ TIMEOUT: Service exists but is not responding (hanging)."
    echo "   This indicates a threading/deadlock issue in mujoco_ros2_control."
    echo ""
    echo "   Check your LAUNCH TERMINAL for this message:"
    echo "     [mujoco_ros2_control-3] [INFO] ... apply_external_wrench SERVICE RECEIVED ..."
    echo ""
    echo "   If you see that message: Service handler is called but can't respond"
    echo "   If you DON'T see it: Rebuilt binary isn't being used"
elif [ $EXIT_CODE -eq 0 ]; then
    echo "✅ SUCCESS: Service responded!"
else
    echo "❌ ERROR: Service call failed with exit code $EXIT_CODE"
fi
