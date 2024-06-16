"""Defines functions for merging urdf parts at fixed joints."""

import logging
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path
from typing import Any, Optional, Union
from uuid import uuid4

import numpy as np
from scipy.spatial.transform import Rotation as R

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
from kol.mesh import Mesh, load_file

logger = logging.getLogger(__name__)

MAX_NAME_LENGTH: int = 64


def find_fixed_joint(urdf_etree: ET.Element) -> Optional[ET.Element]:
    for joint in urdf_etree.findall(".//joint"):
        if joint.attrib["type"] == "fixed":
            return joint
    return None


def get_link_by_name(urdf_etree: ET.Element, link_name: str | bytes) -> Optional[ET.Element]:
    for link in urdf_etree.findall(".//link"):
        if link.attrib["name"] == link_name:
            return link
    return None


def string_to_nparray(string: Union[str, bytes, Any]) -> np.ndarray:  # noqa: ANN401
    if isinstance(string, bytes):
        string = string.decode("utf-8")
    return np.array([float(item) for item in string.split(" ")])


def get_new_rpy(
    mass_1: float,
    mass_2: float,
    rpy_1: np.ndarray,
    rpy_2: np.ndarray,
) -> np.ndarray:
    rotation_1 = R.from_euler("xyz", rpy_1).as_matrix()
    rotation_2 = R.from_euler("xyz", rpy_2).as_matrix()
    combined_rotation_matrix = (mass_1 * rotation_1 + mass_2 * rotation_2) / (mass_1 + mass_2)
    u, _, vt = np.linalg.svd(combined_rotation_matrix)
    corrected_rotation_matrix = np.dot(u, vt)
    new_rpy = R.from_matrix(corrected_rotation_matrix).as_euler("xyz")
    return new_rpy


def get_color(element: ET.Element) -> np.ndarray:
    visual_elem = element.find("visual")
    if visual_elem is not None:
        material_elem = visual_elem.find("material")
        if material_elem is not None:
            color_elem = material_elem.find("color")
            if color_elem is not None:
                rgba = color_elem.attrib["rgba"].split()
                return np.array([float(x) for x in rgba])
    return np.array([0.5, 0.5, 0.5, 1.0])  # Default color if not found


def combine_parts(
    parent: ET.Element,
    child: ET.Element,
    relative_origin: np.ndarray,
    relative_rpy: np.ndarray,
    urdf_path: Path,
    scaling: float,
) -> ET.Element:
    def get_part_name(element: ET.Element) -> str:
        name = element.attrib.get("name")
        if name is None:
            raise ValueError(f"{element.tag} link does not have a name")
        return name[5:] if name.startswith("link_") else name

    def create_mesh_files(parent_name: str, child_name: str, combined_mesh_name: str) -> tuple[Path, Any]:
        mesh_dir = urdf_path.parent / "meshes"
        parent_mesh = load_file(mesh_dir / f"{parent_name}.stl")
        child_mesh = load_file(mesh_dir / f"{child_name}.stl")
        combined_mesh = combine_meshes(parent_mesh, child_mesh, relative_transform)
        combined_mesh.save(mesh_dir / f"{combined_mesh_name}.stl")
        return mesh_dir, combined_mesh

    def create_collision_mesh(mesh: Mesh, mesh_name: str) -> None:
        collision_mesh = deepcopy(mesh)
        # collision_mesh = get_mesh_convex_hull(collision_mesh)
        collision_mesh = scale_mesh(collision_mesh, scaling)
        collision_mesh.save(mesh_dir / f"{mesh_name}_collision.stl")

    def get_dynamics(element: ET.Element) -> Dynamics:
        dynamics_elem = element.find("inertial")
        if dynamics_elem is None:
            raise ValueError("Inertial elements not found")

        mass = float(dynamics_elem.find("mass").attrib["value"])
        com = string_to_nparray(dynamics_elem.find("origin").attrib["xyz"])
        inertia_values = [float(x) for x in dynamics_elem.find("inertia").attrib.values()]
        inertia = np.matrix(moments_to_matrix(inertia_values))

        return Dynamics(mass, com, inertia)

    parent_name = get_part_name(parent)
    child_name = get_part_name(child)
    new_part_name = "fused_part_" + str(uuid4())[:8]

    relative_transform = origin_and_rpy_to_transform(relative_origin, relative_rpy)
    mesh_dir, combined_mesh = create_mesh_files(parent_name, child_name, new_part_name)
    create_collision_mesh(combined_mesh, new_part_name)

    # parent_dynamics = get_dynamics(parent)
    # child_dynamics = get_dynamics(child)
    # combined_dynamics = combine_dynamics([parent_dynamics, child_dynamics])

    new_part = ET.Element("link", attrib={"name": new_part_name})

    def create_visual_and_collision_elements(tag: str, file_name: str) -> None:
        element = ET.SubElement(new_part, tag, {})
        ET.SubElement(element, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geometry = ET.SubElement(element, "geometry", {})
        ET.SubElement(geometry, "mesh", {"filename": file_name})
        if tag == "visual":
            parent_color = get_color(parent)
            child_color = get_color(child)
            combined_color = (parent_color + child_color) / 2
            material = ET.SubElement(element, "material", {"name": f"{file_name}_material"})  # same as parent joint
            ET.SubElement(
                material, "color", {"rgba": " ".join(map(str, combined_color))}
            )  # average of parent and child colors

    create_visual_and_collision_elements("visual", f"{new_part_name}.stl")
    create_visual_and_collision_elements("collision", f"{new_part_name}_collision.stl")

    # new_inertial = ET.SubElement(new_part, "inertial", {})
    # ET.SubElement(new_inertial, "mass", {"value": str(combined_dynamics.mass)})
    # ET.SubElement(new_inertial, "inertia", matrix_to_moments(combined_dynamics.inertia))

    # parent_rpy = string_to_nparray(parent.find("inertial/origin").attrib["rpy"])
    # child_rpy = string_to_nparray(child.find("inertial/origin").attrib["rpy"])
    # new_rpy = get_new_rpy(parent_dynamics.mass, child_dynamics.mass, parent_rpy, child_rpy)

    # ET.SubElement(
    #     new_inertial,
    #     "origin",
    #     {"xyz": " ".join(map(str, combined_dynamics.com.tolist())), "rpy": " ".join(map(str, new_rpy.tolist()))},
    # )
    return new_part


def process_fixed_joints(urdf_etree: ET.ElementTree, scaling: float, urdf_path: Path) -> ET.ElementTree:
    """Processes the fixed joints in the assembly."""
    if urdf_etree is None:
        raise ValueError("Invalid URDF etree.")
    root = urdf_etree.getroot()

    # While there still exists fixed joints, fuse the parts they connect.
    while True:
        joint = find_fixed_joint(root)
        if joint is None:
            break

        # Get the parent and child of the fixed joint
        parent_element = joint.find("parent")
        if parent_element is None:
            raise ValueError("Parent element not found in joint")
        parent_name = parent_element.attrib["link"]
        parent = get_link_by_name(root, parent_name)
        if parent is None:
            raise ValueError(f"Parent link {parent_name!r} not found in the URDF")

        child_element = joint.find("child")
        if child_element is None:
            raise ValueError("Child element not found in joint")
        child_name = child_element.attrib["link"]
        child = get_link_by_name(root, child_name)
        if child is None:
            raise ValueError(f"Child link {child_name!r} not found in the URDF")

        # Get the relative transform between the two joints
        logger.info("Fusing parts: %s, %s.", parent_name, child_name)
        origin_element = joint.find("origin")
        if origin_element is None:
            raise ValueError("Origin element not found in joint")
        relative_origin = string_to_nparray(origin_element.attrib["xyz"])
        relative_rpy = string_to_nparray(origin_element.attrib["rpy"])

        # Fuse the parts and add to second index of etree to preserve central node
        new_part = combine_parts(parent, child, relative_origin, relative_rpy, urdf_path, scaling)
        root.insert(1, new_part)

        # Replace the parent and child at all auxillary joints with the new part
        root.remove(joint)
        for aux_joint in root.findall(".//joint"):
            if aux_joint is None:
                raise ValueError("Corrupted joint element found in urdf.")
            parent_element = aux_joint.find("parent")
            child_element = aux_joint.find("child")
            if parent_element is None or child_element is None:
                raise ValueError("Parent or child element not found in joint during update")
            if parent_element.attrib["link"] in [parent_name, child_name]:
                parent_element.attrib["link"] = new_part.attrib["name"]
                # Remap joint to be relative to the new part
                origin_element = joint.find("origin")
                if origin_element is None:
                    raise ValueError("Origin element not found in joint during update")
                # Origin is concat of current joint origin and origin of child link that's been fused
                origin = string_to_nparray(origin_element.attrib["xyz"])
                origin = origin + relative_origin
                origin_element.attrib["xyz"] = " ".join(map(str, origin))
                # RPY calculated by multiplying the rotation matrices of the two parts
                rpy = string_to_nparray(origin_element.attrib["rpy"])
                # Convert to rotation matrices
                rotation = R.from_euler("xyz", rpy).as_matrix()
                relative_rotation = R.from_euler("xyz", relative_rpy).as_matrix()
                # Multiply the rotation matrices
                combined_rotation = np.dot(rotation, relative_rotation)
                # Convert back to euler angles
                combined_rpy = R.from_matrix(combined_rotation).as_euler("xyz")
                origin_element.attrib["rpy"] = " ".join(map(str, combined_rpy))
            if child_element.attrib["link"] in [child_name, parent_name]:
                child_element.attrib["link"] = new_part.attrib["name"]

        # Remove the fixed joint and parent and child links
        for link in root.findall(".//link"):
            if link.attrib["name"] in [parent_name, child_name]:
                root.remove(link)

    return urdf_etree


def get_merged_urdf(
    urdf_path: Path,
    scaling: float,
    cleanup_fused_meshes: bool,
) -> None:
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
    urdf_tree = ET.parse(urdf_path)
    # Process the fixed joints
    logger.info("Processing fixed joints, starting joint count: %d", len(urdf_tree.findall(".//joint")))
    merged_urdf = process_fixed_joints(urdf_tree, scaling, urdf_path)
    if merged_urdf is None:
        raise ValueError("Failed to merge fixed joints.")
    logger.info("Finished processing fixed joints, ending joint count: %d", len(merged_urdf.findall(".//joint")))
    # Cleanup the meshes directory by removing all meshes not referenced in urdf
    deleted = 0
    if cleanup_fused_meshes:
        logger.info("Cleaning up obsolete meshes.")
        mesh_dir = urdf_path.parent / "meshes"
        all_links = merged_urdf.findall(".//link")
        all_visuals = (link.find("visual") for link in all_links)
        all_stl_names = [
            visual.find("geometry/mesh").attrib["filename"] for visual in all_visuals if visual is not None
        ]
        all_collisions = (link.find("collision") for link in all_links)
        all_collision_names = [
            collision.find("geometry/mesh").attrib["filename"] for collision in all_collisions if collision is not None
        ]
        all_stls = all_stl_names + all_collision_names
        all_stls = [
            filepath[len("./meshes/") :] if filepath.startswith("./meshes/") else filepath for filepath in all_stls
        ]
        for mesh_file in mesh_dir.glob("*.stl"):
            filename = mesh_file.name
            link_filename = "link_" + filename
            if filename not in all_stls and link_filename not in all_stls:
                mesh_file.unlink()
                deleted += 1
        logger.info("Cleaned up %d meshes.", deleted)

    # Save the merged URDF
    merged_urdf_path = urdf_path.parent / f"{urdf_path.stem}_merged.urdf"
    save_xml(merged_urdf_path, merged_urdf)
    logger.info("Saved merged URDF to %s.", merged_urdf_path)
