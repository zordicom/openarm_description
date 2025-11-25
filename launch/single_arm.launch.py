"""
Copyright 2025 Zordi, Inc. All rights reserved.

Launch single OpenARM v10 in MuJoCo simulation with ros2_control.

This launch file:
1. Generates robot description with MuJoCo plugin enabled
2. Starts the MuJoCo ROS2 control node
3. Loads all controllers (position, velocity, effort) for dynamic switching
4. Optionally starts RViz for visualization

The system uses control_mode:=all in MuJoCo, which enables automatic controller
switching based on which ROS2 controller is active.

Usage Examples:
    # Basic launch (default: joint_trajectory_controller active, home keyframe)
    ros2 launch openarm_description single_arm.launch.py

    # Start with effort controller (pure torque/gravity comp)
    ros2 launch openarm_description single_arm.launch.py \
        default_controller:=effort_controller

    # Start with full MIT controller (gravity-comp trajectories)
    ros2 launch openarm_description single_arm.launch.py \
        default_controller:=zordi_ros_controllers

    # Start at pose1 keyframe
    ros2 launch openarm_description single_arm.launch.py \
        initial_keyframe:=pose1

    # With hand/gripper
    ros2 launch openarm_description single_arm.launch.py hand:=true

    # Without RViz
    ros2 launch openarm_description single_arm.launch.py use_rviz:=false

    # Headless mode (no viewer, no RViz - for servers/CI)
    ros2 launch openarm_description single_arm.launch.py headless:=true

    # Switch controllers at runtime (mode switches automatically):
    ros2 control switch_controllers \
        --deactivate joint_trajectory_controller \
        --activate effort_controller

    # Custom MuJoCo model
    ros2 launch openarm_description single_arm.launch.py \
        mujoco_model_path:=/path/to/custom_model.xml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Generate launch description for single arm v10 MuJoCo simulation."""
    pkg_openarm_description = get_package_share_directory("openarm_description")

    # Declare launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "hand",
            default_value="false",
            description="Include hand/gripper in simulation",
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
            "headless",
            default_value="false",
            description="Run without any viewer (headless mode for servers/CI)",
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
                "Override path to MuJoCo XML model (default: auto-detect based on hand)"
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_keyframe",
            default_value="home",
            choices=["home", "pose1"],
            description="Initial keyframe from MuJoCo XML model",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "default_controller",
            default_value="joint_trajectory_controller",
            choices=[
                "joint_trajectory_controller",
                "effort_controller",
                "zordi_ros_controllers",
            ],
            description=(
                "Which controller to activate at startup: "
                "joint_trajectory_controller (stiff position_servo), "
                "effort_controller (pure torque mit mode), "
                "zordi_ros_controllers (full MIT with pos+vel+eff)"
            ),
        )
    )

    # Get launch configurations
    hand = LaunchConfiguration("hand")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    mujoco_model_path = LaunchConfiguration("mujoco_model_path")
    initial_keyframe = LaunchConfiguration("initial_keyframe")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    default_controller = LaunchConfiguration("default_controller")

    # Generate robot description
    # MuJoCo dynamically switches modes based on active controller
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
            "hand:=",
            hand,
            " ",
            "bimanual:=false",
        ]),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    # Controller configuration - load single config with all controllers
    controller_config_path = os.path.join(pkg_openarm_description, "config", "mujoco")
    controllers_config = os.path.join(controller_config_path, "controllers_all.yaml")

    # Dynamically determine MuJoCo model path based on configuration
    def get_mujoco_model_path(context):
        """Select the appropriate MuJoCo model based on hand configuration."""
        # Check if custom model path was provided
        custom_path = context.perform_substitution(mujoco_model_path)
        if custom_path:
            return custom_path

        # Auto-detect based on configuration
        hand_val = context.perform_substitution(hand)

        # Build model filename based on configuration
        model_name = "openarm_v10"
        if hand_val.lower() == "true":
            model_name += "_hand"
        model_name += ".xml"

        return os.path.join(pkg_openarm_description, "mujoco_models", model_name)

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

        # Get initial keyframe name
        keyframe_name = context.perform_substitution(initial_keyframe)

        # Verify model exists and print info
        print(f"\n{'=' * 60}")
        print("MuJoCo Configuration (Single Arm v10):")
        print(f"  Model path: {model_path}")
        print(f"  Exists: {os.path.exists(model_path)}")
        hand_val = context.perform_substitution(hand)
        print(f"  hand={hand_val}")
        print(f"  Initial keyframe: {keyframe_name}")
        print("  Dynamic mode switching: ENABLED")
        print("    - position_servo: Trajectory controller (pos+vel)")
        print("    - mit: All other controllers (default)")
        print("  Controllers loaded:")
        print("    - joint_trajectory_controller (ACTIVE - position_servo mode)")
        print("    - effort_controller (inactive - mit mode, torque only)")
        print("    - zordi_ros_controllers (inactive - mit mode, full control)")
        print("  Note: CartesianController also provides full MIT mode")
        print("  Controller config: controllers_all.yaml")
        print(f"{'=' * 60}\n")

        if not os.path.exists(model_path):
            print("ERROR: MuJoCo model not found!")
            print("\nPlease generate the model first:")
            cmd = "python3 scripts/urdf_to_mjcf.py --arm-type v10"
            if hand_val.lower() == "true":
                cmd += " --hand"
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
                    controllers_config,  # All controllers (mode switches automatically)
                    {
                        "mujoco_model_path": model_path,
                        "use_sim_time": use_sim_time,
                        "headless": headless,
                        "initial_keyframe": keyframe_name,
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

    # RViz Node (disabled if headless OR use_rviz=false)
    # When headless:=true, we don't want any GUI (including RViz)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(AndSubstitution(use_rviz, NotSubstitution(headless))),
    )

    # Controller spawners - activation based on default_controller parameter
    def create_controller_spawners(context):
        """Create controller spawner nodes with one active based on parameter."""
        active_controller = context.perform_substitution(default_controller)

        controllers = [
            "joint_trajectory_controller",
            "effort_controller",
            "zordi_ros_controllers",
        ]

        spawners = [
            # Joint state broadcaster always active
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            )
        ]

        # Spawn trajectory controllers with one active
        for controller in controllers:
            args = [controller, "-c", "/controller_manager"]
            if controller != active_controller:
                args.insert(1, "--inactive")  # Make inactive if not the default

            spawners.append(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=args,
                    output="screen",
                )
            )

        return spawners

    controller_spawners = OpaqueFunction(function=create_controller_spawners)

    # Timing and sequencing
    # Add delay to ensure controller_manager thread is ready
    delayed_controller_spawners = TimerAction(
        period=1.0,
        actions=[controller_spawners],
    )

    nodes_to_start = [
        *declared_arguments,
        robot_state_publisher,
        world_to_base_link,
        mujoco_node_launcher,
        rviz_node,
        delayed_controller_spawners,
    ]

    return LaunchDescription(nodes_to_start)
