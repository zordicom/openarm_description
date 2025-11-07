#!/usr/bin/env python3
"""
Copyright 2025 Zordi, Inc. All rights reserved.

Script to convert OpenARM URDF to MuJoCo MJCF format.

This script:
1. Generates URDF from xacro with MuJoCo-specific settings
2. Converts URDF to MJCF using MuJoCo's compile utility
3. Optionally applies custom simulation parameters
4. Validates the output MJCF file

Usage:
    python3 urdf_to_mjcf.py --arm-type v10 --output openarm_v10.xml
    python3 urdf_to_mjcf.py --arm-type v10 --bimanual --output openarm_v10_bimanual.xml
    python3 urdf_to_mjcf.py --arm-type v10 --hand --output openarm_v10_with_hand.xml
"""

import argparse
import os
import subprocess
import sys
import tempfile
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

    # Write back
    tree.write(mjcf_path, encoding="utf-8", xml_declaration=True)


def fix_mjcf_mesh_paths(mjcf_path: Path) -> None:
    """Fix mesh file paths in the generated MJCF to match copied mesh locations.

    urdf2mjcf creates mesh references like "link0_symp.stl" but copies files to
    subdirectories like "meshes/arm/v10/collision/link0_symp.stl". This function
    finds the actual mesh files and updates the MJCF paths.
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

    # Save the updated MJCF
    tree.write(mjcf_path, encoding="utf-8", xml_declaration=True)
    print("✓ Fixed mesh paths in MJCF")


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


def main():
    """Main conversion pipeline."""
    parser = argparse.ArgumentParser(
        description="Convert OpenARM URDF to MuJoCo MJCF format"
    )
    parser.add_argument(
        "--arm-type",
        type=str,
        default="v10",
        help="ARM type (default: v10)",
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
        required=True,
        help="Output MJCF file path",
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

    args = parser.parse_args()

    if args.validate_only:
        if not args.output.exists():
            print(f"Error: File not found: {args.output}", file=sys.stderr)
            sys.exit(1)
        validate_mjcf(args.output)
        sys.exit(0)

    # Auto-detect package path if not provided
    if args.package_path is None:
        args.package_path = Path(__file__).parent.parent

    try:
        # Step 1: Generate URDF from xacro
        urdf_str = generate_urdf_from_xacro(
            arm_type=args.arm_type,
            bimanual=args.bimanual,
            hand=args.hand,
            package_path=args.package_path,
        )

        # Step 2: Convert URDF to MJCF
        args.output.parent.mkdir(parents=True, exist_ok=True)
        convert_urdf_to_mjcf(urdf_str, args.output)

        # Step 3: Disable gravity if requested (useful for PID tuning)
        if args.no_gravity:
            print("\nDisabling gravity...")
            disable_gravity_in_mjcf(args.output)

        # Step 4: Apply custom simulation parameters (if provided)
        if args.sim_params:
            apply_simulation_parameters(args.output, args.sim_params)

        print(f"\n✓ Conversion complete: {args.output}")

    except Exception as e:
        print(f"\n✗ Conversion failed: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
