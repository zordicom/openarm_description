#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

MuJoCo Passive Viewer with Full UI for ROS2.

This node launches the full MuJoCo interactive viewer (with ImGui interface)
that mirrors the state from the ROS2 mujoco_ros2_control node. This gives you
the complete simulate binary experience while staying synchronized with ROS2.

Usage:
    ros2 run openarm_description mujoco_passive_viewer.py --ros-args -p model_path:=/path/to/model.xml
"""

import argparse
import threading
import time
from pathlib import Path

import mujoco.viewer
import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

import mujoco


class MuJoCoPassiveViewer(Node):
    """ROS2 node that displays MuJoCo simulation with full interactive UI."""

    def __init__(self):
        super().__init__("mujoco_passive_viewer")

        # Declare parameters
        self.declare_parameter("model_path", "")
        # Note: use_sim_time is automatically declared by ROS2 when passed from launch file

        # Get parameters
        model_path_str = self.get_parameter("model_path").value
        if not model_path_str:
            self.get_logger().error(
                "model_path parameter is required! "
                "Use: --ros-args -p model_path:=/path/to/model.xml"
            )
            raise ValueError("model_path parameter required")

        model_path = Path(model_path_str)
        if not model_path.exists():
            self.get_logger().error(f"Model file not found: {model_path}")
            raise FileNotFoundError(f"Model not found: {model_path}")

        self.get_logger().info(f"Loading MuJoCo model from: {model_path}")

        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)

        # Initialize simulation
        mujoco.mj_forward(self.model, self.data)

        # Subscribe to joint states from mujoco_ros2_control
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Note: Force detection removed - causes threading issues with viewer
        # Use the embedded C++ viewer instead for working perturbations!

        # Joint name to index mapping
        self.joint_name_to_id = {}
        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name:
                self.joint_name_to_id[joint_name] = i

        self.get_logger().info(f"Found {len(self.joint_name_to_id)} joints in model")

        # Launch viewer in separate thread
        self.viewer_running = False
        self.viewer_thread = threading.Thread(target=self._run_viewer, daemon=True)
        self.viewer_thread.start()

        self.get_logger().info("MuJoCo passive viewer launched!")
        self.get_logger().info(
            "The viewer window has FULL interactive controls (like simulate binary)"
        )
        self.get_logger().info("Use Ctrl+Right-click to apply forces in the viewer!")

    def joint_state_callback(self, msg: JointState):
        """Update MuJoCo data from ROS2 joint states."""
        for i, name in enumerate(msg.name):
            if name in self.joint_name_to_id:
                joint_id = self.joint_name_to_id[name]
                qpos_adr = self.model.jnt_qposadr[joint_id]
                qvel_adr = self.model.jnt_dofadr[joint_id]

                # Update position and velocity
                if i < len(msg.position):
                    self.data.qpos[qpos_adr] = msg.position[i]
                if i < len(msg.velocity):
                    self.data.qvel[qvel_adr] = msg.velocity[i]

        # Note: Don't call mj_forward here - it conflicts with viewer rendering
        # The viewer.sync() call in the viewer thread handles the visualization update

    def _run_viewer(self):
        """Run the MuJoCo viewer in a separate thread."""
        self.get_logger().info("Starting MuJoCo interactive viewer...")

        try:
            # Launch passive viewer with full UI
            with mujoco.viewer.launch_passive(
                self.model, self.data, show_left_ui=True, show_right_ui=True
            ) as viewer:
                self.viewer_running = True
                self.get_logger().info("Viewer window opened - full UI enabled!")

                # Keep viewer alive and synced
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.01)  # ~100 Hz update rate

                self.viewer_running = False
                self.get_logger().info("Viewer closed")

        except Exception as e:
            self.get_logger().error(f"Viewer error: {e}")
            self.viewer_running = False


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = MuJoCoPassiveViewer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
