"""Defines functions for merging urdf parts at fixed joints."""

import logging
from pathlib import Path

import numpy as np
from lxml import etree

from kol.geometry import (
    Dynamics,
    combine_dynamics,
    combine_meshes,
    get_mesh_convex_hull,
    matrix_to_moments,
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


def combine_parts(
    parent: etree.Element,
    child: etree.Element,
    relative_origin: np.ndarray,
    relative_rpy: np.ndarray,
    urdf_path: Path,
    scaling: float,
) -> etree.Element:
    # Get new part name
    parent_name = parent.attrib.get("name")[5:]
    child_name = child.attrib.get("name")[5:]
    if parent_name.startswith("fused_"):
        parent_name = parent_name[6:]
    if child_name.startswith("fused_"):
        child_name = child_name[6:]
    new_part_name = f"fused_{parent_name}_{child_name}"
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

    # Get combined dynamic mesh
    parent_dynamics = parent.find("inertial")
    child_dynamics = child.find("inertial")
    parent_mass = float(parent_dynamics.find("mass").attrib["value"])
    child_mass = float(child_dynamics.find("mass").attrib["value"])
    parent_com = np.array([float(x) for x in parent_dynamics.find("origin").attrib["xyz"]])
    child_com = np.array([float(x) for x in child_dynamics.find("origin").attrib["xyz"]])
    parent_inertia = np.array([float(x) for x in parent_dynamics.find("inertia").attrib.values()]).reshape(3, 3)
    child_inertia = np.array([float(x) for x in child_dynamics.find("inertia").attrib.values()]).reshape(3, 3)
    parent_moments = matrix_to_moments(parent_inertia)
    child_moments = matrix_to_moments(child_inertia)
    parent_dynamics = Dynamics(parent_mass, parent_com, parent_moments)
    child_dynamics = Dynamics(child_mass, child_com, child_moments)
    combined_dynamics = combine_dynamics(parent_dynamics, child_dynamics, relative_transform)

    # Create new part element
    new_part = etree.Element("link", attrib={"name": new_part_name})

    # Get combined visual mesh
    new_visual = etree.SubElement(new_part, "visual")
    new_visual_origin = etree.SubElement(new_visual, "origin", attrib={"xyz": "0 0 0", "rpy": "0 0 0"})
    new_visual_geometry = etree.SubElement(new_visual, combined_stl_file_path)

    # Get combined collision mesh

    logger.info("Got combined meshes and dynamics.")

    # Create new part element and populate


def process_fixed_joints(urdf_etree: etree, scaling: float, urdf_path: str) -> etree:
    """Processes the fixed joints in the assembly."""
    # While there still exists fixed joints, fuse the parts they connect.
    while (joint := find_fixed_joint(urdf_etree)) is not None:
        # Get the parent and child of the fixed joint
        parent_name = joint.find("parent").attrib["link"]
        parent = get_link_by_name(urdf_etree, parent_name)
        child_name = joint.find("child").attrib["link"]
        child = get_link_by_name(urdf_etree, child_name)
        logger.info("Fusing parts: %s, %s.", parent_name, child_name)
        # Get the relative transform between the two joints
        relative_origin = np.array([float(item) for item in (joint.find("origin").attrib["xyz"].split(" "))])
        relative_rpy = np.array([float(item) for item in (joint.find("origin").attrib["rpy"].split(" "))])
        # Fuse the parts and add to second index of etree to preserve central node
        new_part = combine_parts(parent, child, relative_origin, relative_rpy, urdf_path, scaling)
        urdf_etree.insert(1, new_part)
        # Replace the parent and child at all joints with the new part
        joint.find("parent").attrib["link"] = new_part.attrib["name"]
        joint.find("child").attrib["link"] = new_part.attrib["name"]
        # Remove the fixed joint
        urdf_etree.remove(joint)
        # Remove parent and child links from urdf
        for link in urdf_etree.findall(".//link"):
            if link.attrib["name"] in [parent, child]:
                urdf_etree.remove(link)
    return urdf_etree


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
    # Process the fixed joints
    logger.info("Processing fixed joints. Starting joint count: %d", len(urdf.findall(".//joint")))
    merged_urdf = process_fixed_joints(urdf, scaling, urdf_path)
    logger.info("Finished processing fixed joints. Ending joint count:" + len(merged_urdf.findall(".//joint")))
    # Cleanup the meshes directory by removing all meshes not referenced in urdf
    deleted = 0
    if cleanup_fused_meshes:
        mesh_dir = urdf_path.parent / "meshes"
        for mesh_file in mesh_dir.glob("*.stl"):
            if mesh_file.name not in [link.attrib["filename"] for link in merged_urdf.findall(".//link")]:
                mesh_file.unlink()
                deleted += 1
    logger.info("Cleaned up %d meshes.", deleted)
    # Save the merged URDF
    merged_urdf_path = urdf_path.parent / f"{urdf_path.stem}_merged.urdf"
    with open(merged_urdf_path, "w") as file:
        file.write(etree.tostring(merged_urdf).decode())
    logger.info("Saved merged URDF to %s.", merged_urdf_path)
