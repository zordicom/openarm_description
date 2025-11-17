#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Comprehensive build script to convert OpenARM xacro → URDF → MuJoCo XML.

This script:
1. Generates URDF from xacro with MuJoCo-specific settings
2. Converts URDF to MJCF using MuJoCo's compile utility
3. Optionally applies custom simulation parameters
4. Validates the output MJCF file
5. Can regenerate all common configurations at once

Usage:
    # Build all common configurations
    python3 urdf_to_mjcf.py --build-all

    # Single configuration
    python3 urdf_to_mjcf.py --arm-type v10 --output mujoco_models/openarm_v10.xml

    # Build with specific options
    python3 urdf_to_mjcf.py --arm-type v10 --bimanual --hand --output custom.xml
"""

import argparse
import math
import os
import subprocess
import sys
from pathlib import Path

import mujoco


def generate_urdf_from_xacro(
    arm_type: str,
    bimanual: bool = False,
    hand: bool = False,
    package_path: Path = None,
) -> str:
    """Generate URDF from xacro file with MuJoCo settings.

    Args:
        arm_type: Type of arm (e.g., 'v10')
        bimanual: Whether to generate bimanual configuration
        hand: Whether to include hand/gripper
        package_path: Path to openarm_description package

    Returns:
        URDF string
    """
    if package_path is None:
        package_path = Path(__file__).parent.parent

    xacro_file = package_path / "urdf" / "robot" / f"{arm_type}.urdf.xacro"

    if not xacro_file.exists():
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")

    # Build xacro command with MuJoCo-specific arguments
    cmd = [
        "xacro",
        str(xacro_file),
        "ros2_control:=true",
        "use_mujoco:=true",
        f"hand:={'true' if hand else 'false'}",
        f"bimanual:={'true' if bimanual else 'false'}",
    ]

    print(f"Generating URDF from xacro: {' '.join(cmd)}")

    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, check=True, cwd=str(package_path)
        )
        return result.stdout
    except subprocess.CalledProcessError as e:
        print(f"Error generating URDF: {e.stderr}", file=sys.stderr)
        raise


def disable_gravity_in_mjcf(mjcf_path: Path) -> None:
    """Disable gravity in the MJCF file (useful for PID tuning without gravity).

    Sets gravity to "0 0 0" in the <option> tag.
    """
    import xml.etree.ElementTree as ET

    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    # Find and modify gravity in option tag
    for option in root.findall(".//option[@gravity]"):
        option.set("gravity", "0 0 0")
        print("  ✓ Disabled gravity in MJCF (set to 0 0 0)")

    # Format and write back
    ET.indent(tree, space="  ", level=0)
    tree.write(mjcf_path, encoding="utf-8", xml_declaration=True)


def add_multi_mode_actuators(
    root, joint_limits_dict, kp_pos=100.0, kv_pos=0.0, kv_vel=10.0
):
    """Add three actuators per joint for multi-mode control.

    Based on validated 1-DoF test results:
    - Position actuator (kp=100, kv=0): For position_servo mode
    - Velocity actuator (kv=10): For velocity control and damping
    - Motor actuator: For MIT mode (pure torque)

    Args:
        root: XML root element
        joint_limits_dict: Dictionary of joint limits from YAML
        kp_pos: Position actuator stiffness (default: 100.0, validated)
        kv_pos: Position actuator damping (default: 0.0, damping via velocity actuator)
        kv_vel: Velocity actuator gain (default: 10.0, validated)

    Returns:
        ET.Element: The actuator element
    """
    import xml.etree.ElementTree as ET

    # Create actuator element
    actuator_elem = ET.SubElement(root, "actuator")

    num_joints = 0
    for joint_name, joint_data in joint_limits_dict.items():
        if not joint_name.startswith("joint"):
            continue  # Skip non-joint entries (like gripper)

        limits = joint_data.get("limit", {})
        lower = limits.get("lower", -math.pi)
        upper = limits.get("upper", math.pi)
        effort = limits.get("effort", 40.0)

        # Estimate max velocity (if not specified, use default 3.0 rad/s)
        velocity = limits.get("velocity", 3.0)

        # 1. Position actuator: For position_servo mode
        ET.SubElement(
            actuator_elem,
            "position",
            name=f"act_pos_openarm_{joint_name}",
            joint=f"openarm_{joint_name}",
            kp=str(kp_pos),
            kv=str(kv_pos),
            ctrlrange=f"{lower} {upper}",
            forcerange=f"-{effort} {effort}",
        )

        # 2. Velocity actuator: For velocity control and damping
        ET.SubElement(
            actuator_elem,
            "velocity",
            name=f"act_vel_openarm_{joint_name}",
            joint=f"openarm_{joint_name}",
            kv=str(kv_vel),
            ctrlrange=f"-{velocity} {velocity}",
            forcerange=f"-{effort} {effort}",
        )

        # 3. Motor actuator: For MIT mode (torque control)
        ET.SubElement(
            actuator_elem,
            "motor",
            name=f"act_tau_openarm_{joint_name}",
            joint=f"openarm_{joint_name}",
            ctrlrange=f"-{effort} {effort}",
            forcerange=f"-{effort} {effort}",
        )

        num_joints += 1

    print(
        f"✓ Added {num_joints * 3} actuators ({num_joints} joints × 3 types: position/velocity/motor)"
    )
    print(f"  Position: kp={kp_pos}, kv={kv_pos}")
    print(f"  Velocity: kv={kv_vel}")
    print("  Motor: direct torque")
    return actuator_elem


def fix_mjcf_mesh_paths(mjcf_path: Path) -> None:
    """Fix mesh file paths in the generated MJCF to match copied mesh locations.

    urdf2mjcf creates mesh references like "link0_symp.stl" but copies files to
    subdirectories like "meshes/arm/v10/collision/link0_symp.stl". This function
    finds the actual mesh files and updates the MJCF paths.

    Also disables all collisions by default to prevent self-collision issues.
    """
    import xml.etree.ElementTree as ET

    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    # Get the directory containing the MJCF
    mjcf_dir = mjcf_path.parent

    # Remove meshdir attribute from compiler to avoid double "meshes/" prefix
    for compiler in root.findall(".//compiler[@meshdir]"):
        del compiler.attrib["meshdir"]

    # Add balanceinertia to fix inertia matrix violations (common in hand/gripper models)
    for compiler in root.findall(".//compiler"):
        compiler.set("balanceinertia", "true")

    # Remove freejoint from root body (for fixed-base robots)
    for body in root.findall(".//body"):
        for freejoint in body.findall("./freejoint[@name='root']"):
            body.remove(freejoint)

    # Remove auto-generated actuators (we'll add custom ones with proper gains)
    for actuator_elem in root.findall(".//actuator"):
        root.remove(actuator_elem)

    # Remove actuator sensors (since actuators are removed)
    sensor_elem = root.find(".//sensor")
    if sensor_elem is not None:
        actuator_sensors_removed = 0
        for sensor in list(sensor_elem):
            if sensor.tag in ["actuatorpos", "actuatorvel", "actuatorfrc"]:
                sensor_elem.remove(sensor)
                actuator_sensors_removed += 1
        if actuator_sensors_removed > 0:
            print(f"✓ Removed {actuator_sensors_removed} actuator sensors")

    # DISABLE ALL COLLISIONS: Set contype=0 conaffinity=0 for all geoms
    # This prevents self-collision issues that interfere with gravity compensation
    # 1. Set in defaults
    for default in root.findall(".//default"):
        for geom in default.findall("./geom"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")

    # 2. Remove explicit contype/conaffinity from individual geoms (they override defaults)
    for geom in root.findall(".//geom"):
        if "contype" in geom.attrib:
            geom.set("contype", "0")
        if "conaffinity" in geom.attrib:
            geom.set("conaffinity", "0")

    print("✓ Disabled all collisions (contype=0, conaffinity=0)")

    # Find all mesh elements
    for mesh in root.findall(".//mesh[@file]"):
        mesh_filename = mesh.get("file")

        # Search for the actual mesh file
        mesh_search = list(mjcf_dir.rglob(mesh_filename))
        if mesh_search:
            # Update to relative path from MJCF location
            actual_mesh = mesh_search[0]
            relative_path = actual_mesh.relative_to(mjcf_dir)
            mesh.set(
                "file", str(relative_path).replace("\\", "/")
            )  # Use forward slashes

    # Format and save the updated MJCF
    ET.indent(tree, space="  ", level=0)
    tree.write(mjcf_path, encoding="utf-8", xml_declaration=True)
    print("✓ Fixed mesh paths in MJCF")


def read_pid_gains_from_urdf(urdf_str: str) -> dict:
    """Extract PID gains from URDF ros2_control parameters.

    Returns dict mapping joint names to (kp, kd) tuples.
    If no gains found, returns empty dict (will use defaults).
    """
    import xml.etree.ElementTree as ET

    try:
        urdf_tree = ET.fromstring(urdf_str)
        gains = {}

        # Find all joints in ros2_control
        for joint in urdf_tree.findall(".//ros2_control//joint"):
            joint_name = joint.get("name", "")
            if not joint_name:
                continue

            kp = None
            kd = None

            # Look for position_kp and position_kd parameters
            for param in joint.findall("./param"):
                param_name = param.get("name", "")
                if param_name == "position_kp":
                    kp = float(param.text)
                elif param_name == "position_kd":
                    kd = float(param.text)

            if kp is not None and kd is not None:
                # Strip prefix (e.g., "openarm_joint1" -> "joint1")
                simple_name = joint_name.replace("openarm_", "")
                gains[simple_name] = (kp, kd)

        if gains:
            print(f"✓ Read PID gains from URDF for {len(gains)} joints")
        else:
            print("  No PID gains found in URDF, using defaults")

        return gains

    except Exception as e:
        print(f"Warning: Could not read PID gains from URDF: {e}")
        return {}


def add_keyframes_from_yaml(
    mjcf_path: Path, poses_yaml_path: Path, package_path: Path
) -> None:
    """Add keyframes to MJCF from initial_poses.yaml file.

    Args:
        mjcf_path: Path to MJCF file
        poses_yaml_path: Path to initial_poses.yaml (if None, auto-detect)
        package_path: Path to package root
    """
    import xml.etree.ElementTree as ET
    import yaml

    # Auto-detect poses file if not provided
    if poses_yaml_path is None:
        poses_yaml_path = package_path / "config" / "mujoco" / "initial_poses.yaml"

    if not poses_yaml_path.exists():
        print(f"  No keyframe poses file found at {poses_yaml_path}")
        return

    try:
        # Load poses from YAML
        with open(poses_yaml_path) as f:
            poses_data = yaml.safe_load(f)

        if not poses_data or "poses" not in poses_data:
            print("  No poses defined in YAML file")
            return

        poses = poses_data["poses"]

        # Read MJCF to get joint information
        tree = ET.parse(mjcf_path)
        root = tree.getroot()

        # Get list of joints from MJCF
        mjcf_joints = []
        for joint_elem in root.findall(".//joint"):
            joint_name = joint_elem.get("name", "")
            if joint_name and joint_name.startswith("openarm_"):
                # Strip prefix for matching with YAML
                simple_name = joint_name.replace("openarm_", "")
                mjcf_joints.append(simple_name)

        if not mjcf_joints:
            print("  No joints found in MJCF")
            return

        num_joints = len(mjcf_joints)
        print(f"  Found {num_joints} joints in MJCF: {mjcf_joints}")

        # Remove any existing keyframe element
        for kf in root.findall("keyframe"):
            root.remove(kf)

        # Create new keyframe element
        keyframe_elem = ET.SubElement(root, "keyframe")

        added_keyframes = 0
        for pose_name, pose_data in poses.items():
            if not isinstance(pose_data, dict):
                continue

            # Build qpos vector in MJCF joint order
            qpos_values = []
            missing_joints = []

            for joint_name in mjcf_joints:
                if joint_name in pose_data:
                    qpos_values.append(str(pose_data[joint_name]))
                else:
                    missing_joints.append(joint_name)
                    qpos_values.append("0.0")  # Default to 0 if missing

            # Warn if joints are missing
            if missing_joints:
                print(f"  Warning: Pose '{pose_name}' missing joints: {missing_joints}")

            # Check if we have the right number of values
            if len(qpos_values) != num_joints:
                print(
                    f"  Warning: Pose '{pose_name}' has {len(qpos_values)} values but MJCF has {num_joints} joints, skipping"
                )
                continue

            # Add keyframe
            qpos_str = " ".join(qpos_values)
            ET.SubElement(keyframe_elem, "key", name=pose_name, qpos=qpos_str)
            added_keyframes += 1
            print(f"    Added keyframe '{pose_name}': {qpos_str}")

        if added_keyframes == 0:
            print("  No valid keyframes added")
            return

        # Format and write back
        ET.indent(tree, space="  ", level=0)
        tree.write(mjcf_path, encoding="utf-8", xml_declaration=True)

        print(f"✓ Added {added_keyframes} keyframes from {poses_yaml_path.name}")

    except Exception as e:
        print(f"Warning: Could not add keyframes from YAML: {e}")
        import traceback

        traceback.print_exc()


def add_actuators_to_mjcf(
    mjcf_path: Path,
    joint_limits_path: Path,
    urdf_str: str = None,
    kp_pos=100.0,
    kv_pos=0.0,
    kv_vel=10.0,
) -> None:
    """Add three actuators per joint to MJCF file after conversion.

    Args:
        mjcf_path: Path to MJCF file
        joint_limits_path: Path to joint limits YAML file
        urdf_str: URDF string to extract PID gains from (optional)
        kp_pos: Default position actuator stiffness (used if not in URDF)
        kv_pos: Default position actuator damping
        kv_vel: Default velocity actuator gain
    """
    import xml.etree.ElementTree as ET

    import yaml

    if not joint_limits_path.exists():
        print(f"Warning: Joint limits file not found: {joint_limits_path}")
        print("  Skipping actuator generation")
        return

    # Load joint limits
    with open(joint_limits_path) as f:
        joint_limits = yaml.safe_load(f)

    # Read PID gains from URDF if provided
    urdf_gains = {}
    if urdf_str:
        urdf_gains = read_pid_gains_from_urdf(urdf_str)

    # Parse MJCF
    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    # Add multi-mode actuators with per-joint gains from URDF
    actuator_elem = ET.SubElement(root, "actuator")
    num_joints = 0

    for joint_name, joint_data in joint_limits.items():
        if not joint_name.startswith("joint"):
            continue

        limits = joint_data.get("limit", {})
        lower = limits.get("lower", -math.pi)
        upper = limits.get("upper", math.pi)
        effort = limits.get("effort", 40.0)
        velocity = limits.get("velocity", 3.0)

        # Use URDF gains if available, otherwise defaults
        if joint_name in urdf_gains:
            joint_kp, joint_kd = urdf_gains[joint_name]
            print(f"  Using URDF gains for {joint_name}: Kp={joint_kp}, Kd={joint_kd}")
        else:
            joint_kp = kp_pos
            joint_kd = kv_vel  # Use velocity gain for damping

        # 1. Position actuator
        ET.SubElement(
            actuator_elem,
            "position",
            name=f"act_pos_openarm_{joint_name}",
            joint=f"openarm_{joint_name}",
            kp=str(joint_kp),
            kv=str(kv_pos),  # Position actuator damping (usually 0)
            ctrlrange=f"{lower} {upper}",
            forcerange=f"-{effort} {effort}",
        )

        # 2. Velocity actuator (use Kd from URDF if available)
        ET.SubElement(
            actuator_elem,
            "velocity",
            name=f"act_vel_openarm_{joint_name}",
            joint=f"openarm_{joint_name}",
            kv=str(joint_kd),  # Use URDF Kd as velocity actuator gain
            ctrlrange=f"-{velocity} {velocity}",
            forcerange=f"-{effort} {effort}",
        )

        # 3. Motor actuator
        ET.SubElement(
            actuator_elem,
            "motor",
            name=f"act_tau_openarm_{joint_name}",
            joint=f"openarm_{joint_name}",
            ctrlrange=f"-{effort} {effort}",
            forcerange=f"-{effort} {effort}",
        )

        num_joints += 1

    print(f"✓ Added {num_joints * 3} actuators ({num_joints} joints × 3 types)")

    # Format and write back
    ET.indent(tree, space="  ", level=0)
    tree.write(mjcf_path, encoding="utf-8", xml_declaration=True)
    print(f"✓ Added multi-mode actuators to {mjcf_path.name}")


def convert_urdf_to_mjcf(urdf_str: str, output_path: Path) -> None:
    """Convert URDF string to MJCF XML and save to file.

    MuJoCo automatically converts URDF to MJCF when loading with MjModel.from_xml_path().
    This is the official recommended approach for MuJoCo 3.x.

    Args:
        urdf_str: URDF XML string
        output_path: Path to save MJCF file

    Raises:
        RuntimeError: If conversion fails
    """
    print("Converting URDF to MJCF using MuJoCo's automatic converter...")

    # Convert mesh paths to absolute paths before conversion
    # urdf2mjcf copies URDF to /tmp, so relative paths break
    package_root = Path(__file__).parent.parent
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Replace package:// URIs with absolute paths
    urdf_with_absolute_paths = urdf_str.replace(
        "package://openarm_description/", f"{package_root}/"
    )

    # Add MuJoCo compiler options to help with conversion (especially for hand/gripper inertia issues)
    import xml.etree.ElementTree as ET

    try:
        urdf_tree = ET.fromstring(urdf_with_absolute_paths)
        # Check if mujoco element already exists
        mujoco_elem = urdf_tree.find("mujoco")
        if mujoco_elem is None:
            mujoco_elem = ET.SubElement(urdf_tree, "mujoco")
        compiler_elem = mujoco_elem.find("compiler")
        if compiler_elem is None:
            compiler_elem = ET.SubElement(mujoco_elem, "compiler")
        compiler_elem.set("balanceinertia", "true")
        urdf_with_absolute_paths = ET.tostring(urdf_tree, encoding="unicode")
    except Exception as e:
        print(f"Warning: Could not add MuJoCo compiler options: {e}")

    # Save URDF temporarily
    urdf_path = package_root / "temp_robot.urdf"
    urdf_path.write_text(urdf_with_absolute_paths)

    from urdf2mjcf.convert import convert_urdf_to_mjcf as urdf2mjcf_convert

    print("Converting URDF to MJCF using urdf2mjcf...")
    # copy_meshes=True will copy mesh files next to the MJCF output
    try:
        urdf2mjcf_convert(str(urdf_path), str(output_path), copy_meshes=True)
        print(f"✓ Successfully converted to MJCF: {output_path}")
    except Exception as e:
        # If urdf2mjcf fails (e.g., inertia violations), try direct MuJoCo with balanceinertia
        if "inertia must satisfy" in str(e):
            print(
                "urdf2mjcf failed with inertia error, retrying with balanceinertia..."
            )
            # Change to package root and use direct MuJoCo loading with balanceinertia
            original_dir = os.getcwd()
            os.chdir(str(package_root))
            try:
                # Load with balanceinertia option (already set in URDF)
                model = mujoco.MjModel.from_xml_path(str(urdf_path))
                mujoco.mj_saveLastXML(str(output_path), model)
                print(
                    f"✓ Successfully converted to MJCF with balanceinertia: {output_path}"
                )

                # Manually copy meshes since we're not using urdf2mjcf
                import shutil

                mesh_src = package_root / "meshes"
                mesh_dst = output_path.parent / "meshes"
                if mesh_src.exists():
                    shutil.copytree(mesh_src, mesh_dst, dirs_exist_ok=True)
                    print(f"✓ Copied meshes to {mesh_dst}")
            finally:
                os.chdir(original_dir)
        else:
            raise

    # Fix mesh paths in the generated MJCF
    # urdf2mjcf creates paths like "link0_symp.stl" but copies to "meshes/arm/v10/collision/link0_symp.stl"
    fix_mjcf_mesh_paths(output_path)

    # Validate the generated MJCF
    validate_mjcf(output_path)

    # Clean up temporary URDF file
    if urdf_path.exists():
        urdf_path.unlink(missing_ok=True)


def validate_mjcf(mjcf_path: Path) -> None:
    """Validate that the MJCF file can be loaded by MuJoCo.

    Args:
        mjcf_path: Path to MJCF file

    Raises:
        RuntimeError: If validation fails
    """
    print("Validating MJCF file...")

    try:
        model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        data = mujoco.MjData(model)

        print("✓ MJCF is valid")
        print(f"  - DOF: {model.nv}")
        print(f"  - Bodies: {model.nbody}")
        print(f"  - Joints: {model.njnt}")
        print(f"  - Actuators: {model.nu}")

        # Print joint names
        print("\n  Joint names:")
        for i in range(model.njnt):
            joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            print(f"    - {joint_name}")

    except Exception as e:
        print(f"✗ MJCF validation failed: {e}", file=sys.stderr)
        raise


def apply_simulation_parameters(mjcf_path: Path, sim_params_path: Path = None) -> None:
    """Apply custom simulation parameters to MJCF file.

    Args:
        mjcf_path: Path to MJCF file
        sim_params_path: Path to simulation parameters YAML file

    Note:
        This modifies the MJCF file in-place.
        Simulation parameters include damping, friction, solver settings, etc.
    """
    if sim_params_path is None or not sim_params_path.exists():
        print("No simulation parameters file provided, using defaults")
        return

    # TODO: Implement custom parameter application
    # This could read a YAML file with damping, friction, solver settings
    # and modify the MJCF XML accordingly
    print(f"Applying simulation parameters from: {sim_params_path}")
    print("  (Custom parameter application not yet implemented)")


def build_configuration(
    arm_type: str,
    output_xml: Path,
    output_urdf: Path,
    bimanual: bool = False,
    hand: bool = False,
    no_gravity: bool = False,
    kp_pos: float = 100.0,
    kv_pos: float = 0.0,
    kv_vel: float = 10.0,
    package_path: Path = None,
):
    """Build a single configuration: xacro → URDF → XML.

    Args:
        arm_type: Arm type (e.g., 'v10')
        output_xml: Path for MuJoCo XML file
        output_urdf: Path for URDF file
        bimanual: Whether to generate bimanual configuration
        hand: Whether to include hand/gripper
        no_gravity: Whether to disable gravity
        kp_pos: Position actuator stiffness (default: 100.0, validated)
        kv_pos: Position actuator damping (default: 0.0)
        kv_vel: Velocity actuator gain (default: 10.0, validated)
        package_path: Path to package root
    """
    config_name = output_xml.stem
    print(f"\n{'=' * 80}")
    print(f"Building: {config_name}")
    print(f"{'=' * 80}")

    # Step 1: Generate URDF from xacro
    urdf_str = generate_urdf_from_xacro(
        arm_type=arm_type, bimanual=bimanual, hand=hand, package_path=package_path
    )

    # Step 2: Save URDF file
    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    output_urdf.write_text(urdf_str)
    print(f"✓ Saved URDF: {output_urdf}")

    # Step 3: Convert URDF to MuJoCo XML
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    convert_urdf_to_mjcf(urdf_str, output_xml)

    # Step 4: Disable gravity if requested
    if no_gravity:
        print("  Disabling gravity...")
        disable_gravity_in_mjcf(output_xml)

    # Step 5: Add multi-mode actuators (with URDF gains)
    joint_limits_path = package_path / "config" / "arm" / arm_type / "joint_limits.yaml"
    add_actuators_to_mjcf(
        output_xml, joint_limits_path, urdf_str, kp_pos, kv_pos, kv_vel
    )

    # Step 6: Add keyframes from initial_poses.yaml
    poses_yaml_path = package_path / "config" / "mujoco" / "initial_poses.yaml"
    add_keyframes_from_yaml(output_xml, poses_yaml_path, package_path)

    print(f"✓ Complete: {config_name}\n")


def build_all_configurations(
    arm_type: str = "v10",
    kp_pos: float = 100.0,
    kv_pos: float = 0.0,
    kv_vel: float = 10.0,
    package_path: Path = None,
):
    """Build all common configurations.

    Generates:
    - openarm_v10.xml / openarm_v10.urdf (single arm)
    - openarm_v10_no_gravity.xml (single arm, no gravity for tuning)
    - openarm_v10_hand.xml / openarm_v10_hand.urdf (single arm + hand)
    - openarm_v10_bimanual.xml / openarm_v10_bimanual.urdf (dual arm)
    - openarm_v10_bimanual_hand.xml / openarm_v10_bimanual_hand.urdf (dual + hands)
    """
    if package_path is None:
        package_path = Path(__file__).parent.parent

    models_dir = package_path / "mujoco_models"

    configurations = [
        {
            "name": "Single Arm",
            "xml": models_dir / f"openarm_{arm_type}.xml",
            "urdf": models_dir / f"openarm_{arm_type}.urdf",
            "bimanual": False,
            "hand": False,
            "no_gravity": False,
        },
        {
            "name": "Single Arm (No Gravity)",
            "xml": models_dir / f"openarm_{arm_type}_no_gravity.xml",
            "urdf": None,
            "bimanual": False,
            "hand": False,
            "no_gravity": True,
        },
        {
            "name": "Single Arm + Hand",
            "xml": models_dir / f"openarm_{arm_type}_hand.xml",
            "urdf": models_dir / f"openarm_{arm_type}_hand.urdf",
            "bimanual": False,
            "hand": True,
            "no_gravity": False,
        },
        {
            "name": "Bimanual",
            "xml": models_dir / f"openarm_{arm_type}_bimanual.xml",
            "urdf": models_dir / f"openarm_{arm_type}_bimanual.urdf",
            "bimanual": True,
            "hand": False,
            "no_gravity": False,
        },
        {
            "name": "Bimanual + Hands",
            "xml": models_dir / f"openarm_{arm_type}_bimanual_hand.xml",
            "urdf": models_dir / f"openarm_{arm_type}_bimanual_hand.urdf",
            "bimanual": True,
            "hand": True,
            "no_gravity": False,
        },
    ]

    print(f"\n{'=' * 80}")
    print(f"Building ALL configurations for {arm_type}")
    print(f"{'=' * 80}\n")

    errors = []

    for config in configurations:
        try:
            if config["no_gravity"]:
                base_xml = models_dir / f"openarm_{arm_type}.xml"
                if not base_xml.exists():
                    print(f"⚠ Skipping {config['name']}: base file not found")
                    continue

                import shutil

                shutil.copy(base_xml, config["xml"])
                disable_gravity_in_mjcf(config["xml"])
                print(f"✓ Created no-gravity variant: {config['xml'].name}\n")
            else:
                urdf_output = (
                    config["urdf"] if config["urdf"] else models_dir / "temp.urdf"
                )

                build_configuration(
                    arm_type=arm_type,
                    output_xml=config["xml"],
                    output_urdf=urdf_output,
                    bimanual=config["bimanual"],
                    hand=config["hand"],
                    no_gravity=config["no_gravity"],
                    kp_pos=kp_pos,
                    kv_pos=kv_pos,
                    kv_vel=kv_vel,
                    package_path=package_path,
                )

                if config["urdf"] is None and urdf_output.exists():
                    urdf_output.unlink()

        except Exception as e:
            error_msg = f"✗ Failed to build {config['name']}: {e}"
            print(error_msg)
            errors.append(error_msg)

    print(f"\n{'=' * 80}")
    if errors:
        print(f"Build completed with {len(errors)} error(s):")
        for error in errors:
            print(f"  {error}")
    else:
        print("✓ All configurations built successfully!")
    print(f"{'=' * 80}\n")

    if errors:
        sys.exit(1)


def main():
    """Main conversion pipeline."""
    parser = argparse.ArgumentParser(
        description="Convert OpenARM xacro → URDF → MuJoCo XML"
    )
    parser.add_argument(
        "--arm-type",
        type=str,
        default="v10",
        help="ARM type (default: v10)",
    )
    parser.add_argument(
        "--build-all",
        action="store_true",
        help="Build all common configurations (single, bimanual, with/without hand)",
    )
    parser.add_argument(
        "--bimanual",
        action="store_true",
        help="Generate bimanual configuration",
    )
    parser.add_argument(
        "--hand",
        action="store_true",
        help="Include hand/gripper",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        required=False,
        help="Output MJCF file path (required unless --build-all is used)",
    )
    parser.add_argument(
        "--package-path",
        type=Path,
        default=None,
        help="Path to openarm_description package (default: auto-detect)",
    )
    parser.add_argument(
        "--sim-params",
        type=Path,
        default=None,
        help="Path to simulation parameters YAML file",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Only validate existing MJCF file",
    )
    parser.add_argument(
        "--no-gravity",
        action="store_true",
        help="Disable gravity in the generated MJCF (useful for PID tuning)",
    )
    parser.add_argument(
        "--kp-pos",
        type=float,
        default=100.0,
        help="Position actuator stiffness gain (default: 100.0, validated)",
    )
    parser.add_argument(
        "--kv-pos",
        type=float,
        default=0.0,
        help="Position actuator damping gain (default: 0.0, damping via velocity actuator)",
    )
    parser.add_argument(
        "--kv-vel",
        type=float,
        default=10.0,
        help="Velocity actuator gain (default: 10.0, validated)",
    )

    args = parser.parse_args()

    # Auto-detect package path if not provided
    if args.package_path is None:
        args.package_path = Path(__file__).parent.parent

    # Handle validate-only mode
    if args.validate_only:
        if not args.output or not args.output.exists():
            print(f"Error: File not found: {args.output}", file=sys.stderr)
            sys.exit(1)
        validate_mjcf(args.output)
        sys.exit(0)

    # Handle build-all mode
    if args.build_all:
        build_all_configurations(
            arm_type=args.arm_type,
            kp_pos=args.kp_pos,
            kv_pos=args.kv_pos,
            kv_vel=args.kv_vel,
            package_path=args.package_path,
        )
        sys.exit(0)

    # Single configuration mode - require output
    if not args.output:
        print("Error: --output is required unless --build-all is used", file=sys.stderr)
        parser.print_help()
        sys.exit(1)

    try:
        # Generate URDF output path alongside XML
        urdf_output = args.output.parent / args.output.name.replace(".xml", ".urdf")

        # Build single configuration
        build_configuration(
            arm_type=args.arm_type,
            output_xml=args.output,
            output_urdf=urdf_output,
            bimanual=args.bimanual,
            hand=args.hand,
            no_gravity=args.no_gravity,
            kp_pos=args.kp_pos,
            kv_pos=args.kv_pos,
            kv_vel=args.kv_vel,
            package_path=args.package_path,
        )

        # Apply custom simulation parameters if provided
        if args.sim_params:
            apply_simulation_parameters(args.output, args.sim_params)

        print("\n✓ Build complete!")
        print(f"  XML:  {args.output}")
        print(f"  URDF: {urdf_output}")

    except Exception as e:
        print(f"\n✗ Build failed: {e}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
