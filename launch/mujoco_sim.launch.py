"""
Copyright 2025 Zordi, Inc. All rights reserved.

Launch OpenARM in MuJoCo simulation with ros2_control.

This launch file:
1. Generates robot description with MuJoCo plugin enabled
2. Starts the MuJoCo ROS2 control node
3. Loads all controllers (position, velocity, effort) for dynamic switching
4. Optionally starts RViz for visualization

The system uses control_mode:=all in MuJoCo, which enables automatic controller
switching based on which ROS2 controller is active.

Usage Examples:
    # Basic launch (starts with position controller active)
    ros2 launch openarm_description mujoco_sim.launch.py

    # Switch to velocity controller at runtime:
    ros2 control switch_controllers --deactivate joint_trajectory_controller \
        --activate velocity_controller

    # Switch to effort controller at runtime:
    ros2 control switch_controllers --deactivate velocity_controller \
        --activate effort_controller

    # With hand
    ros2 launch openarm_description mujoco_sim.launch.py hand:=true

    # Bimanual configuration
    ros2 launch openarm_description mujoco_sim.launch.py bimanual:=true

    # Without RViz
    ros2 launch openarm_description mujoco_sim.launch.py use_rviz:=false

    # Custom MuJoCo model
    ros2 launch openarm_description mujoco_sim.launch.py \
        mujoco_model_path:=/path/to/custom_model.xml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for MuJoCo simulation."""
    pkg_openarm_description = get_package_share_directory("openarm_description")

    # Declare launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_type",
            default_value="v10",
            description="ARM type (e.g., v10)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "hand",
            default_value="false",
            description="Include hand/gripper in simulation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "bimanual",
            default_value="false",
            description="Use bimanual configuration",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz for visualization",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                pkg_openarm_description, "rviz", "mujoco_view.rviz"
            ),
            description="Path to RViz configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "mujoco_model_path",
            default_value="",
            description=(
                "Override path to MuJoCo XML model "
                "(default: auto-detect based on hand/bimanual)"
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )

    # Get launch configurations
    hand = LaunchConfiguration("hand")
    bimanual = LaunchConfiguration("bimanual")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    mujoco_model_path = LaunchConfiguration("mujoco_model_path")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Generate robot description with all control modes enabled
    # control_mode:=all enables dynamic switching between controllers
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            os.path.join(pkg_openarm_description, "urdf", "robot", "v10.urdf.xacro"),
            " ",
            "ros2_control:=true",
            " ",
            "use_mujoco:=true",
            " ",
            "control_mode:=all",
            " ",
            "hand:=",
            hand,
            " ",
            "bimanual:=",
            bimanual,
        ]),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    # Controller configurations for all three modes
    controller_config_path = os.path.join(pkg_openarm_description, "config", "mujoco")
    position_config = os.path.join(controller_config_path, "controllers_position.yaml")
    velocity_config = os.path.join(controller_config_path, "controllers_velocity.yaml")
    effort_config = os.path.join(controller_config_path, "controllers_effort.yaml")

    # Dynamically determine MuJoCo model path based on configuration
    def get_mujoco_model_path(context):
        """Select the appropriate MuJoCo model based on hand and bimanual parameters."""
        # Check if custom model path was provided
        custom_path = context.perform_substitution(mujoco_model_path)
        if custom_path:
            return custom_path

        # Auto-detect based on configuration
        hand_val = context.perform_substitution(hand)
        bimanual_val = context.perform_substitution(bimanual)

        # Build model filename based on configuration
        model_name = "openarm_v10"
        if bimanual_val.lower() == "true":
            model_name += "_bimanual"
        if hand_val.lower() == "true":
            model_name += "_hand"
        model_name += ".xml"

        # Check both install and source directories (for development)
        # First try install directory (for production use)
        install_path = os.path.join(
            pkg_openarm_description, "mujoco_models", model_name
        )

        # If not found in install, try source directory (for development)
        if not os.path.exists(install_path):
            # pkg_openarm_description is like:
            # /path/to/install/openarm_description/share/openarm_description
            # We need to get to:
            # /path/to/src/openarm_description/mujoco_models/model_name
            # Go up from install dir to workspace root, then to src
            install_share = pkg_openarm_description
            # .../install/openarm_description/share/openarm_description
            install_pkg = os.path.dirname(install_share)
            # .../install/openarm_description/share
            install_pkg = os.path.dirname(install_pkg)
            # .../install/openarm_description
            install_dir = os.path.dirname(install_pkg)  # .../install
            workspace_root = os.path.dirname(install_dir)  # .../ros2_ws

            src_path = os.path.join(
                workspace_root,
                "src",
                "openarm_description",
                "mujoco_models",
                model_name,
            )
            if os.path.exists(src_path):
                return src_path

        return install_path

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # MuJoCo node launcher with all controller configs
    def launch_mujoco_node(context):
        """Launch MuJoCo ROS2 Control node with all controllers."""
        # Get the appropriate MuJoCo model path
        model_path = get_mujoco_model_path(context)

        # Verify model exists and print info
        print(f"\n{'=' * 60}")
        print("MuJoCo Configuration:")
        print(f"  Model path: {model_path}")
        print(f"  Exists: {os.path.exists(model_path)}")
        hand_val = context.perform_substitution(hand)
        bimanual_val = context.perform_substitution(bimanual)
        print(f"  hand={hand_val}, bimanual={bimanual_val}")
        print("  Control mode: all (dynamic switching enabled)")
        print("  Controllers: position, velocity, effort")
        print(f"{'=' * 60}\n")

        if not os.path.exists(model_path):
            print("ERROR: MuJoCo model not found!")
            print("\nPlease generate the model first:")
            cmd = "python3 scripts/urdf_to_mjcf.py --arm-type v10"
            if hand_val.lower() == "true":
                cmd += " --hand"
            if bimanual_val.lower() == "true":
                cmd += " --bimanual"
            cmd += f" --output mujoco_models/{os.path.basename(model_path)}"
            print("  cd ~/ros2_ws/src/openarm_description")
            print(f"  {cmd}")
            print(f"{'=' * 60}\n")
            raise FileNotFoundError(f"MuJoCo model not found: {model_path}")

        return [
            Node(
                package="mujoco_ros2_control",
                executable="mujoco_ros2_control",
                output="screen",
                parameters=[
                    robot_description,
                    position_config,
                    velocity_config,
                    effort_config,
                    {
                        "mujoco_model_path": model_path,
                        "use_sim_time": use_sim_time,
                    },
                ],
            )
        ]

    mujoco_node_launcher = OpaqueFunction(function=launch_mujoco_node)

    # Static transform from world to robot base
    # This is needed for RViz to visualize the robot
    world_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "world", "openarm_link0"],
        output="screen",
        # static transform does not need sim time
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # Load joint state broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "--controller-manager",
            "/controller_manager",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    # Load all controllers as inactive first (for dynamic switching)
    load_position_controller_inactive = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "--controller-manager",
            "/controller_manager",
            "joint_trajectory_controller",
        ],
        output="screen",
    )

    load_velocity_controller_inactive = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "--controller-manager",
            "/controller_manager",
            "velocity_controller",
        ],
        output="screen",
    )

    load_effort_controller_inactive = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "inactive",
            "--controller-manager",
            "/controller_manager",
            "effort_controller",
        ],
        output="screen",
    )

    # Activate position controller by default
    activate_position_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "set_controller_state",
            "joint_trajectory_controller",
            "active",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Event handlers for sequential controller loading
    # Load joint state broadcaster when robot state publisher starts
    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[load_joint_state_broadcaster],
        )
    )

    # After joint_state_broadcaster loads, load all three controllers as inactive
    load_all_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[
                load_position_controller_inactive,
                load_velocity_controller_inactive,
                load_effort_controller_inactive,
            ],
        )
    )

    # After all controllers are loaded, activate position controller
    activate_position_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_effort_controller_inactive,
            on_exit=[activate_position_controller],
        )
    )

    nodes_to_start = [
        *declared_arguments,
        robot_state_publisher,
        world_to_base_link,
        mujoco_node_launcher,
        rviz_node,
        load_joint_state_broadcaster_event,
        load_all_controllers_event,
        activate_position_event,
    ]

    return LaunchDescription(nodes_to_start)
