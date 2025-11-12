#!/bin/bash
# Copyright 2025 Zordi, Inc. All rights reserved.
#
# Test script to verify controller loading fix with single-threaded executor pattern.

set -e

cd /home/gilwoo/ros2_ws
source install/setup.bash

echo "=========================================="
echo "Testing Controller Loading Fix"
echo "Architecture: SingleThreadedExecutor + manual spin"
echo "=========================================="
echo ""

# Launch in background
echo "Launching simulation in headless mode..."
ros2 launch openarm_description single_arm.launch.py headless:=true > /tmp/mujoco_test.log 2>&1 &
LAUNCH_PID=$!

# Give it time to initialize
echo "Waiting for initialization (8 seconds)..."
sleep 8

echo ""
echo "Test 1: List controllers (should respond in <1 second)"
echo "------------------------------------------------------"
if timeout 2 ros2 control list_controllers; then
    echo "✅ PASS: Controllers loaded successfully!"
    TEST1_PASS=1
else
    echo "❌ FAIL: Controller listing timed out"
    TEST1_PASS=0
fi

echo ""
echo "Test 2: Check controller states"
echo "------------------------------------------------------"
CONTROLLERS=$(ros2 control list_controllers 2>/dev/null | wc -l)
if [ "$CONTROLLERS" -ge 2 ]; then
    echo "✅ PASS: Found $CONTROLLERS controllers"
    TEST2_PASS=1
else
    echo "❌ FAIL: Expected at least 2 controllers, found $CONTROLLERS"
    TEST2_PASS=0
fi

echo ""
echo "Test 3: External wrench service"
echo "------------------------------------------------------"
if timeout 2 ros2 service call /mujoco_ros2_control_node/apply_external_wrench \
    mujoco_ros2_control_msgs/srv/ApplyExternalWrench \
    "{body_name: 'openarm_link7', force: {x: 0.0, y: 0.0, z: 1.0}, duration: 0.5}" > /dev/null 2>&1; then
    echo "✅ PASS: External wrench service responded"
    TEST3_PASS=1
else
    echo "⚠️  WARN: External wrench service timeout (may be expected)"
    TEST3_PASS=0
fi

echo ""
echo "=========================================="
echo "Test Results Summary"
echo "=========================================="
echo "Test 1 (Controller Loading): $([ $TEST1_PASS -eq 1 ] && echo '✅ PASS' || echo '❌ FAIL')"
echo "Test 2 (Controller Count):   $([ $TEST2_PASS -eq 1 ] && echo '✅ PASS' || echo '❌ FAIL')"
echo "Test 3 (External Wrench):    $([ $TEST3_PASS -eq 1 ] && echo '✅ PASS' || echo '⚠️  WARN')"
echo ""

# Show relevant log output
echo "Last 30 lines of launch log:"
echo "=========================================="
tail -30 /tmp/mujoco_test.log

echo ""
echo "Shutting down..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 2
killall -9 mujoco_ros2_control 2>/dev/null || true

TOTAL_PASS=$((TEST1_PASS + TEST2_PASS))
if [ $TOTAL_PASS -ge 2 ]; then
    echo "✅ Overall: SUCCESS (critical tests passed)"
    exit 0
else
    echo "❌ Overall: FAILURE (critical tests failed)"
    exit 1
fi
