#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Test script to validate MuJoCo ROS2 control setup for OpenARM.

This script verifies:
1. URDF can be generated with MuJoCo settings
2. URDF contains required ros2_control tags
3. All required configuration files exist
4. MuJoCo model can be loaded (if available)

Usage:
    python3 test_mujoco_setup.py
    python3 test_mujoco_setup.py --bimanual
    python3 test_mujoco_setup.py --hand
"""

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


class Colors:
    """ANSI color codes for terminal output."""

    GREEN = "\033[92m"
    RED = "\033[91m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    RESET = "\033[0m"
    BOLD = "\033[1m"


def print_header(text: str) -> None:
    """Print formatted header."""
    print(f"\n{Colors.BOLD}{Colors.BLUE}{'=' * 60}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{text:^60}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.BLUE}{'=' * 60}{Colors.RESET}\n")


def print_success(text: str) -> None:
    """Print success message."""
    print(f"{Colors.GREEN}✓{Colors.RESET} {text}")


def print_error(text: str) -> None:
    """Print error message."""
    print(f"{Colors.RED}✗{Colors.RESET} {text}")


def print_warning(text: str) -> None:
    """Print warning message."""
    print(f"{Colors.YELLOW}⚠{Colors.RESET} {text}")


def print_info(text: str) -> None:
    """Print info message."""
    print(f"  {text}")


def test_urdf_generation(bimanual: bool, hand: bool, package_path: Path) -> bool:
    """Test URDF generation from xacro.

    Args:
        bimanual: Whether to test bimanual configuration
        hand: Whether to include hand
        package_path: Path to openarm_description package

    Returns:
        True if test passed, False otherwise
    """
    print_header("Test 1: URDF Generation")

    xacro_file = package_path / "urdf" / "robot" / "v10.urdf.xacro"

    if not xacro_file.exists():
        print_error(f"Xacro file not found: {xacro_file}")
        return False

    print_info(f"Testing xacro: {xacro_file}")

    cmd = [
        "xacro",
        str(xacro_file),
        "ros2_control:=true",
        "use_mujoco:=true",
        f"hand:={'true' if hand else 'false'}",
        f"bimanual:={'true' if bimanual else 'false'}",
    ]

    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, check=True, cwd=str(package_path)
        )
        urdf_content = result.stdout

        if not urdf_content:
            print_error("Generated URDF is empty")
            return False

        print_success("URDF generated successfully")
        print_info(f"URDF length: {len(urdf_content)} characters")

        return urdf_content

    except subprocess.CalledProcessError as e:
        print_error(f"Failed to generate URDF: {e.stderr}")
        return False


def test_ros2_control_tags(urdf_content: str, bimanual: bool) -> bool:
    """Test that URDF contains required ros2_control tags.

    Args:
        urdf_content: URDF XML string
        bimanual: Whether bimanual configuration is used

    Returns:
        True if test passed, False otherwise
    """
    print_header("Test 2: ros2_control Tags")

    try:
        root = ET.fromstring(urdf_content)

        # Find all ros2_control tags
        ros2_control_tags = root.findall(".//ros2_control")

        if not ros2_control_tags:
            print_error("No ros2_control tags found in URDF")
            return False

        expected_count = 2 if bimanual else 1
        if len(ros2_control_tags) != expected_count:
            print_warning(
                f"Expected {expected_count} ros2_control tag(s), found {len(ros2_control_tags)}"
            )

        print_success(f"Found {len(ros2_control_tags)} ros2_control tag(s)")

        # Check each ros2_control tag
        for i, tag in enumerate(ros2_control_tags, 1):
            print_info(f"\nros2_control #{i}:")
            name = tag.get("name", "unnamed")
            print_info(f"  Name: {name}")

            # Check for hardware plugin
            hardware_tags = tag.findall(".//hardware/plugin")
            if not hardware_tags:
                print_error("  No hardware plugin found")
                return False

            plugin_name = hardware_tags[0].text
            print_info(f"  Plugin: {plugin_name}")

            if "MujocoSystem" not in plugin_name:
                print_error(f"  Expected MujocoSystem plugin, found: {plugin_name}")
                return False

            print_success("  MuJoCo plugin configured correctly")

            # Check for joints
            joint_tags = tag.findall(".//joint")
            if not joint_tags:
                print_warning("  No joints found in this ros2_control block")
            else:
                print_info(f"  Joints: {len(joint_tags)}")

                # Check command interfaces
                position_cmds = len(
                    tag.findall(".//command_interface[@name='position']")
                )
                velocity_cmds = len(
                    tag.findall(".//command_interface[@name='velocity']")
                )
                effort_cmds = len(tag.findall(".//command_interface[@name='effort']"))

                print_info(f"    Position interfaces: {position_cmds}")
                print_info(f"    Velocity interfaces: {velocity_cmds}")
                print_info(f"    Effort interfaces: {effort_cmds}")

                if position_cmds == 0 and velocity_cmds == 0 and effort_cmds == 0:
                    print_error("  No command interfaces found")
                    return False

        print_success("All ros2_control tags valid")
        return True

    except ET.ParseError as e:
        print_error(f"Failed to parse URDF XML: {e}")
        return False


def test_config_files(package_path: Path) -> bool:
    """Test that all required configuration files exist.

    Args:
        package_path: Path to openarm_description package

    Returns:
        True if test passed, False otherwise
    """
    print_header("Test 3: Configuration Files")

    required_files = [
        "config/mujoco/v10_sim_params.yaml",
        "config/mujoco/controllers_position.yaml",
        "config/mujoco/controllers_velocity.yaml",
        "config/mujoco/controllers_effort.yaml",
        "launch/mujoco_sim.launch.py",
        "rviz/mujoco_view.rviz",
        "scripts/urdf_to_mjcf.py",
    ]

    all_exist = True
    for file_path in required_files:
        full_path = package_path / file_path
        if full_path.exists():
            print_success(f"{file_path}")
        else:
            print_error(f"{file_path} - NOT FOUND")
            all_exist = False

    if all_exist:
        print_success("All configuration files exist")
    else:
        print_error("Some configuration files are missing")

    return all_exist


def test_mujoco_model(package_path: Path) -> bool:
    """Test that MuJoCo model can be loaded (if it exists).

    Args:
        package_path: Path to openarm_description package

    Returns:
        True if test passed or model doesn't exist yet, False if model exists but can't be loaded
    """
    print_header("Test 4: MuJoCo Model")

    mujoco_model_path = package_path / "mujoco_models" / "openarm_v10.xml"

    if not mujoco_model_path.exists():
        print_warning(f"MuJoCo model not found: {mujoco_model_path}")
        print_info("Run urdf_to_mjcf.py script to generate the model")
        return True  # Not an error if model hasn't been generated yet

    try:
        import mujoco

        print_info(f"Loading model: {mujoco_model_path}")
        model = mujoco.MjModel.from_xml_path(str(mujoco_model_path))
        data = mujoco.MjData(model)

        print_success("MuJoCo model loaded successfully")
        print_info(f"  DOF: {model.nv}")
        print_info(f"  Bodies: {model.nbody}")
        print_info(f"  Joints: {model.njnt}")
        print_info(f"  Actuators: {model.nu}")

        return True

    except ImportError:
        print_warning("MuJoCo Python library not installed")
        print_info("Install with: pip install mujoco")
        return True  # Not a failure if library isn't installed

    except Exception as e:
        print_error(f"Failed to load MuJoCo model: {e}")
        return False


def main():
    """Run all tests."""
    parser = argparse.ArgumentParser(
        description="Test MuJoCo ROS2 control setup for OpenARM"
    )
    parser.add_argument(
        "--bimanual",
        action="store_true",
        help="Test bimanual configuration",
    )
    parser.add_argument(
        "--hand",
        action="store_true",
        help="Include hand in tests",
    )
    parser.add_argument(
        "--package-path",
        type=Path,
        default=None,
        help="Path to openarm_description package",
    )

    args = parser.parse_args()

    if args.package_path is None:
        args.package_path = Path(__file__).parent.parent

    print(f"\n{Colors.BOLD}OpenARM MuJoCo Setup Validation{Colors.RESET}")
    print(f"Package path: {args.package_path}")
    print(
        f"Configuration: {'Bimanual' if args.bimanual else 'Single arm'}, Hand: {args.hand}"
    )

    # Run tests
    tests_passed = []

    # Test 1: URDF Generation
    urdf_content = test_urdf_generation(args.bimanual, args.hand, args.package_path)
    tests_passed.append(bool(urdf_content))

    if urdf_content:
        # Test 2: ros2_control Tags (only if URDF generation succeeded)
        tests_passed.append(test_ros2_control_tags(urdf_content, args.bimanual))

    # Test 3: Configuration Files
    tests_passed.append(test_config_files(args.package_path))

    # Test 4: MuJoCo Model
    tests_passed.append(test_mujoco_model(args.package_path))

    # Summary
    print_header("Test Summary")
    total_tests = len(tests_passed)
    passed_tests = sum(tests_passed)

    print(f"Tests passed: {passed_tests}/{total_tests}")

    if all(tests_passed):
        print(f"\n{Colors.GREEN}{Colors.BOLD}✓ All tests passed!{Colors.RESET}\n")
        sys.exit(0)
    else:
        print(f"\n{Colors.RED}{Colors.BOLD}✗ Some tests failed{Colors.RESET}\n")
        sys.exit(1)


if __name__ == "__main__":
    main()
