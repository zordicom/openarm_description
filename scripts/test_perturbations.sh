#!/bin/bash
# Quick test script for interactive perturbations feature
# This script helps you verify the perturbation feature works correctly

echo "=========================================="
echo "Interactive Perturbations Test"
echo "=========================================="
echo ""
echo "This script will guide you through testing the new perturbation feature."
echo ""
echo "Step 1: Build the updated mujoco_ros2_control package"
echo "----------------------------------------"
echo "Running: colcon build --packages-select mujoco_ros2_control"
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_ros2_control --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo "ERROR: Build failed!"
    exit 1
fi

echo ""
echo "✓ Build successful!"
echo ""
echo "Step 2: Source the workspace"
echo "----------------------------------------"
source ~/ros2_ws/install/setup.bash

echo ""
echo "Step 3: Launch MuJoCo simulation"
echo "----------------------------------------"
echo "Starting simulation in 3 seconds..."
sleep 3

# Launch in background
ros2 launch openarm_description mujoco_sim.launch.py &
LAUNCH_PID=$!

echo ""
echo "Simulation launched (PID: $LAUNCH_PID)"
echo ""
echo "=========================================="
echo "Manual Testing Instructions"
echo "=========================================="
echo ""
echo "1. Wait for the MuJoCo viewer window to appear"
echo ""
echo "2. Test Camera Controls (Normal Operation):"
echo "   - Left-click + drag: Rotate camera"
echo "   - Right-click + drag: Pan camera"
echo "   - Scroll: Zoom"
echo ""
echo "3. Test Perturbation (NEW FEATURE):"
echo "   - Hold Ctrl + Right-click on a robot link"
echo "   - You should see the body get highlighted/selected"
echo "   - While holding Ctrl + Right-click, drag the mouse"
echo "   - The link should move with your mouse (applying force)"
echo "   - Release the mouse - the link should spring back"
echo ""
echo "4. Test with Effort Controller:"
echo "   In a new terminal, run:"
echo "   ----------------------------------------"
echo "   cd ~/ros2_ws/src/openarm_description"
echo "   source ~/ros2_ws/install/setup.bash"
echo "   "
echo "   # Switch to effort controller"
echo "   ros2 control switch_controllers \\"
echo "       --deactivate joint_trajectory_controller \\"
echo "       --activate effort_controller"
echo "   "
echo "   # Start gravity compensation"
echo "   python3 scripts/gravity_compensation_controller.py"
echo "   "
echo "   # Now try perturbations with Ctrl+Right-click"
echo "   # The arm should resist and maintain position!"
echo "   ----------------------------------------"
echo ""
echo "5. To stop the simulation:"
echo "   - Close the MuJoCo viewer window, or"
echo "   - Press Ctrl+C in this terminal"
echo ""
echo "=========================================="
echo "Expected Behavior"
echo "=========================================="
echo ""
echo "✓ Camera controls work normally (without Ctrl)"
echo "✓ Ctrl+Right-click selects bodies"
echo "✓ Dragging while holding Ctrl+Right-click applies forces"
echo "✓ Visual indicator shows perturbation point"
echo "✓ With gravity comp active, arm resists disturbances"
echo ""
echo "Press Ctrl+C to stop the simulation when done testing."
echo ""

# Wait for user to kill
wait $LAUNCH_PID
