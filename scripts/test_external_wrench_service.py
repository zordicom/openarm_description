#!/usr/bin/env python3
"""
Simple test to verify the apply_external_wrench service works.

This applies a strong force to the end-effector and you should see the robot move.
"""

import time

import rclpy
from geometry_msgs.msg import Wrench
from mujoco_ros2_control_msgs.srv import ApplyExternalWrench
from rclpy.node import Node


def main():
    """Test the external wrench service."""
    rclpy.init()
    node = Node("wrench_tester")

    print("=" * 80)
    print("TESTING EXTERNAL WRENCH SERVICE")
    print("=" * 80)

    # Create service client
    client = node.create_client(ApplyExternalWrench, "/apply_external_wrench")

    print("\n1. Waiting for service...")
    if not client.wait_for_service(timeout_sec=10.0):
        print("❌ Service /apply_external_wrench is NOT available!")
        print("\nMake sure MuJoCo simulation is running:")
        print("  ros2 launch openarm_description mujoco_sim.launch.py")
        rclpy.shutdown()
        return

    print("✓ Service is available!")

    # Create request
    req = ApplyExternalWrench.Request()
    req.body_name = "openarm_link7"  # Apply force to end-effector
    req.wrench.force.x = 0.0
    req.wrench.force.y = 0.0
    req.wrench.force.z = -50.0  # 50N downward (should be very visible!)
    req.wrench.torque.x = 0.0
    req.wrench.torque.y = 0.0
    req.wrench.torque.z = 0.0
    req.duration = 2.0  # 2 seconds

    print("\n2. Sending request:")
    print(f"   Body: {req.body_name}")
    print("   Force: [0, 0, -50] N (strong downward)")
    print("   Duration: 2.0 seconds")
    print("\n   👀 WATCH THE ROBOT - You should see the arm move down!")

    # Call service
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if not future.done():
        print("❌ Service call timed out!")
        rclpy.shutdown()
        return

    response = future.result()

    print("\n3. Response:")
    print(f"   Accepted: {response.accepted}")
    print(f"   Message: {response.message}")

    if response.accepted:
        print("\n✅ SUCCESS! Service call accepted.")
        print("   The robot should be moving now.")
        print("   Wait 2 seconds for force to be applied...")
        time.sleep(2.5)
        print("   Force should now be released.")
    else:
        print("\n❌ Service call was REJECTED!")
        print("   Check the response message above for details.")

    print("\n" + "=" * 80)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
