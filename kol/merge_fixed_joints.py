"""Defines functions for merging urdf parts at fixed joints."""

import logging
from pathlib import Path

import numpy as np
from lxml import etree

from kol.formats.common import save_xml
from kol.geometry import (
    Dynamics,
    combine_dynamics,
    combine_meshes,
    get_mesh_convex_hull,
    matrix_to_moments,
    moments_to_matrix,
    origin_and_rpy_to_transform,
    scale_mesh,
)
from kol.mesh import load_file

logger = logging.getLogger(__name__)


def parse_urdf(file_path: Path) -> etree:
    with open(file_path, "r") as file:
        urdf_xml = file.read()
    if urdf_xml.startswith("<?xml"):
        urdf_xml = urdf_xml.split("?>", 1)[1].strip()
    return etree.fromstring(urdf_xml)


def find_fixed_joint(urdf_etree: etree) -> etree.Element:
    """Finds the first fixed joint in the assembly."""
    for joint in urdf_etree.findall(".//joint"):
        if joint.attrib["type"] == "fixed":
            return joint
    return None


def get_link_by_name(urdf_etree: etree._Element, link_name: str) -> etree._Element:
    """Finds the link element by its name."""
    for link in urdf_etree.findall(".//link"):
        if link.attrib["name"] == link_name:
            return link
    return None


def string_to_nparray(string: str) -> np.ndarray:
    return np.array([float(item) for item in string.split(" ")])


def combine_inertial_properties(rpy1, inertia1, com1, rpy2, inertia2, com2):
    def rpy_to_rotation_matrix(rpy):
        """Convert RPY to rotation matrix."""
        roll, pitch, yaw = rpy
        r_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        r_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        r_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        r = r_z @ r_y @ r_x
        return r

    def rotation_matrix_to_rpy(r):
        """Convert rotation matrix to RPY."""
        sy = np.sqrt(r[0, 0] ** 2 + r[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(r[2, 1], r[2, 2])
            pitch = np.arctan2(-r[2, 0], sy)
            yaw = np.arctan2(r[1, 0], r[0, 0])
        else:
            roll = np.arctan2(-r[1, 2], r[1, 1])
            pitch = np.arctan2(-r[2, 0], sy)
            yaw = 0

        return np.array([roll, pitch, yaw])

    def combine_inertia_tensors(inertia1, rotation1, com1, inertia2, rotation2, com2, combined_com):
        """Combine inertia tensors of two bodies."""
        rotation1_inv = np.linalg.inv(rotation1)
        rotation2_inv = np.linalg.inv(rotation2)

        # Transform inertia tensors to the new frame
        inertia1_transformed = rotation1 @ inertia1 @ rotation1_inv
        inertia2_transformed = rotation2 @ inertia2 @ rotation2_inv

        # Apply parallel axis theorem
        com1_offset = com1 - combined_com
        com2_offset = com2 - combined_com

        inertia1_parallel = (
            inertia1_transformed
            + (np.dot(com1_offset, com1_offset) * np.eye(3) - np.outer(com1_offset, com1_offset)) * inertia1
        )
        inertia2_parallel = (
            inertia2_transformed
            + (np.dot(com2_offset, com2_offset) * np.eye(3) - np.outer(com2_offset, com2_offset)) * inertia2
        )

        # Combined inertia tensor
        inertia_combined = inertia1_parallel + inertia2_parallel

        return inertia_combined

    # Convert RPY to rotation matrices
    rotation1 = rpy_to_rotation_matrix(rpy1)
    rotation2 = rpy_to_rotation_matrix(rpy2)

    # Combine centers of mass
    total_mass = inertia1[0][0] + inertia2[0][0]
    combined_com = (inertia1[0][0] * com1 + inertia2[0][0] * com2) / total_mass

    # Combine inertia tensors
    combined_inertia = combine_inertia_tensors(
        np.array(inertia1), rotation1, com1, np.array(inertia2), rotation2, com2, combined_com
    )

    # Convert combined inertia tensor back to RPY
    combined_rpy = rotation_matrix_to_rpy(combined_inertia)

    return combined_rpy


def combine_parts(
    parent: etree.Element,
    child: etree.Element,
    relative_origin: np.ndarray,
    relative_rpy: np.ndarray,
    urdf_path: Path,
    scaling: float,
) -> etree.Element:
    # Get new part name
    parent_name = parent.attrib.get("name")
    if parent_name.startswith("link_"):
        parent_name = parent_name[5:]
    child_name = child.attrib.get("name")
    if child_name.startswith("link_"):
        child_name = child_name[5:]
    new_part_name = f"{parent_name}_{child_name}"
    new_part_name = new_part_name.replace("fused_", "", 1)
    new_part_name = "fused_" + new_part_name
    if len(new_part_name) > 50:
        new_part_name = new_part_name[:46] + "..."

    # Get pathing and mesh files
    mesh_dir = urdf_path.parent / "meshes"
    combined_stl_file_name = f"{new_part_name}.stl"
    combined_stl_file_path = mesh_dir / combined_stl_file_name
    combined_stl_file_path.unlink(missing_ok=True)

    # Get parent and child meshes
    parent_mesh = load_file(mesh_dir / f"{parent_name}.stl")
    child_mesh = load_file(mesh_dir / f"{child_name}.stl")

    # Combine the meshes using the relative origins
    relative_transform = origin_and_rpy_to_transform(relative_origin, relative_rpy)
    combined_mesh = combine_meshes(parent_mesh, child_mesh, relative_transform)
    combined_mesh.save(combined_stl_file_path)

    # Get convex hull of combined mesh, and scale to good size.
    combined_collision = get_mesh_convex_hull(combined_mesh)
    combined_collision = scale_mesh(combined_mesh, scaling)

    # Save collision mesh to filepath
    combined_collision_stl_name = f"{new_part_name}_collision.stl"
    collision_stl_file_path = mesh_dir / combined_collision_stl_name
    combined_collision.save(collision_stl_file_path)

    # Get combined dynamics
    parent_dynamics = parent.find("inertial")
    child_dynamics = child.find("inertial")
    parent_mass = float(parent_dynamics.find("mass").attrib["value"])
    child_mass = float(child_dynamics.find("mass").attrib["value"])
    parent_com = string_to_nparray(parent_dynamics.find("origin").attrib["xyz"])
    child_com = string_to_nparray(child_dynamics.find("origin").attrib["xyz"])
    parent_inertia = np.array([float(x) for x in parent_dynamics.find("inertia").attrib.values()])
    child_inertia = np.array([float(x) for x in child_dynamics.find("inertia").attrib.values()])
    parent_dynamics = Dynamics(parent_mass, parent_com, moments_to_matrix(parent_inertia))
    child_dynamics = Dynamics(child_mass, child_com, moments_to_matrix(child_inertia))
    combined_dynamics = combine_dynamics([parent_dynamics, child_dynamics])

    # Create new part element
    new_part = etree.Element("link", attrib={"name": new_part_name})

    # Get combined visual mesh
    new_visual = etree.SubElement(new_part, "visual")
    visual_origin = etree.SubElement(new_visual, "origin", attrib={"xyz": "0 0 0", "rpy": "0 0 0"})
    visual_geometry = etree.SubElement(new_visual, "geometry")
    visual_mesh = etree.SubElement(visual_geometry, "mesh", attrib={"filename": combined_stl_file_name})
    visual_material = etree.SubElement(new_visual, "material", attrib={"name": combined_stl_file_name + "_material"})
    visual_color = etree.SubElement(visual_material, "color", attrib={"rgba": "0.5 0.5 0.5 1"})

    # Get combined collision mesh
    new_collision = etree.SubElement(new_part, "collision")
    collision_origin = etree.SubElement(new_collision, "origin", attrib={"xyz": "0 0 0", "rpy": "0 0 0"})
    collision_geometry = etree.SubElement(new_collision, "geometry")
    collision_mesh = etree.SubElement(collision_geometry, "mesh", attrib={"filename": combined_collision_stl_name})
    logger.info("Got combined meshes and dynamics.")

    # Create inertial element
    new_inertial = etree.SubElement(new_part, "inertial")
    inertial_mass = etree.SubElement(new_inertial, "mass", attrib={"value": str(combined_dynamics.mass)})
    inertial_inertia = etree.SubElement(new_inertial, "inertia", attrib=matrix_to_moments(combined_dynamics.inertia))
    # Get initial rpy of parent and child
    # combined_rpy = combine_inertial_properties(
    #     string_to_nparray(parent.find("inertial").find("origin").attrib["rpy"]),
    #     parent_inertia,
    #     parent_com,
    #     string_to_nparray(child.find("inertial").find("origin").attrib["rpy"]),
    #     child_inertia,
    #     child_com,
    # )
    # Get inertial matrix of parent and child
    inertial_origin = etree.SubElement(
        new_inertial,
        "origin",
        attrib={
            "xyz": " ".join(map(str, combined_dynamics.com.tolist())),
            "rpy": "0 0 0",
        },
    )  # Check in on this
    return new_part


def process_fixed_joints(urdf_etree: etree.ElementTree, scaling: float, urdf_path: str) -> etree.ElementTree:
    """Processes the fixed joints in the assembly."""
    root = urdf_etree.getroot()
    # While there still exists fixed joints, fuse the parts they connect.
    while (joint := find_fixed_joint(root)) is not None:
        # Get the parent and child of the fixed joint
        parent_name = joint.find("parent").attrib["link"]
        parent = get_link_by_name(root, parent_name)
        child_name = joint.find("child").attrib["link"]
        child = get_link_by_name(root, child_name)
        logger.info("Fusing parts: %s, %s.", parent_name, child_name)
        # Get the relative transform between the two joints
        relative_origin = string_to_nparray(joint.find("origin").attrib["xyz"])
        relative_rpy = string_to_nparray(joint.find("origin").attrib["rpy"])
        # Fuse the parts and add to second index of etree to preserve central node
        new_part = combine_parts(parent, child, relative_origin, relative_rpy, urdf_path, scaling)
        root.append(new_part)
        # Replace the parent and child at all joints with the new part
        root.remove(joint)
        for joint in root.findall(".//joint"):
            if joint.find("parent").attrib["link"] == parent_name or joint.find("parent").attrib["link"] == child_name:
                joint.find("parent").attrib["link"] = new_part.attrib["name"]
            if joint.find("child").attrib["link"] == child_name or joint.find("child").attrib["link"] == parent_name:
                joint.find("child").attrib["link"] = new_part.attrib["name"]
        # Remove the fixed joint and parent and child links
        for link in root.findall(".//link"):
            if link.attrib["name"] in [parent_name, child_name]:
                root.remove(link)
    return etree.ElementTree(root)


def get_merged_urdf(
    urdf_path: Path,
    scaling: float,
    cleanup_fused_meshes: bool,
) -> Path:
    """Merges meshes at each fixed joints to avoid collision issues.

    Args:
        urdf_path: The path to the urdf file.
        scaling: The scaling factor to apply to the meshes.
        cleanup_fused_meshes: Whether to remove the fused meshes after merging.

    Returns:
        The path to the merged urdf file.
    """
    if scaling < 0:
        raise ValueError(f"Scaling {scaling} should be greater than 0.")

    # Load the URDF file
    logger.info("Getting element tree from mesh filepath.")
    urdf = parse_urdf(urdf_path)
    urdf = etree.ElementTree(urdf)
    # Process the fixed joints
    logger.info("Processing fixed joints, starting joint count: %d", len(urdf.findall(".//joint")))
    merged_urdf = process_fixed_joints(urdf, scaling, urdf_path)
    logger.info("Finished processing fixed joints, ending joint count: %d", len(merged_urdf.findall(".//joint")))
    # Cleanup the meshes directory by removing all meshes not referenced in urdf
    deleted = 0
    if cleanup_fused_meshes:
        logger.info("Cleaning up obsoletemeshes.")
        mesh_dir = urdf_path.parent / "meshes"
        for mesh_file in mesh_dir.glob("*.stl"):
            if mesh_file.name not in [link.attrib["filename"] for link in merged_urdf.findall(".//link")]:
                mesh_file.unlink()
                deleted += 1
        logger.info("Cleaned up %d meshes.", deleted)
    # Save the merged URDF
    merged_urdf_path = urdf_path.parent / f"{urdf_path.stem}_merged.urdf"
    save_xml(merged_urdf_path, merged_urdf)
    logger.info("Saved merged URDF to %s.", merged_urdf_path)
