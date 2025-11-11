#!/bin/bash
# Completely restart simulation with rebuilt binaries

echo "Killing all mujoco processes..."
pkill -9 -f mujoco_ros2_control
pkill -9 -f ros2
sleep 1

echo "Sourcing workspace..."
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo ""
echo "Binary timestamp:"
stat -c "Modified: %y" ~/ros2_ws/install/mujoco_ros2_control/lib/mujoco_ros2_control/mujoco_ros2_control

echo ""
echo "Now launch simulation:"
echo "  ros2 launch openarm_description mujoco_sim.launch.py"
echo ""
echo "Once started, test with:"
echo "  ./scripts/check_service.sh"
