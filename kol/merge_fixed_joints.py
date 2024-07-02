"""Defines functions for merging urdf parts at fixed joints."""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Optional, Tuple, Union

import numpy as np
from scipy.spatial.transform import Rotation as R

from kol.formats.common import save_xml
from kol.geometry import combine_meshes

# from kol.geometry import (
#     # Dynamics,
#     # combine_dynamics,
#     combine_meshes,
#     # get_mesh_convex_hull,
#     # matrix_to_moments,
#     # moments_to_matrix,
#     scale_mesh,
# )
from kol.mesh import load_file

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


def get_part_name(element: ET.Element) -> str:
    name = element.attrib.get("name")
    if name is None:
        raise ValueError(f"{element.tag} link does not have a name")
    return name[5:] if name.startswith("link_") else name


def origin_and_rpy_to_transform(relative_origin: np.ndarray, relative_rpy: np.ndarray) -> np.ndarray:
    """Converts an origin and rpy to a transformation matrix.

    Args:
        relative_origin: A 3-element numpy array representing the relative origin.
        relative_rpy: A 3-element numpy array representing the relative rpy.

    Returns:
        A (4, 4) transformation matrix.
    """
    if relative_origin.shape != (3,):
        raise ValueError("relative_origin must be a 3-element numpy array")
    if relative_rpy.shape != (3,):
        raise ValueError("relative_rpy must be a 3-element numpy array")

    x, y, z = relative_origin
    roll, pitch, yaw = relative_rpy

    translation = np.array(
        [
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1],
        ]
    )
    roll_mat = np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(roll), -np.sin(roll), 0],
            [0, np.sin(roll), np.cos(roll), 0],
            [0, 0, 0, 1],
        ]
    )
    pitch_mat = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch), 0],
            [0, 1, 0, 0],
            [-np.sin(pitch), 0, np.cos(pitch), 0],
            [0, 0, 0, 1],
        ]
    )
    yaw_mat = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0, 0],
            [np.sin(yaw), np.cos(yaw), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )

    rpy = np.dot(yaw_mat, np.dot(pitch_mat, roll_mat))
    transform = np.dot(translation, rpy)
    return transform


def find_all_mesh_transforms(root: ET.Element) -> Tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Goes joint by joint and gets the transforms of each link relative to root."""
    link_transforms = {}
    joint_transforms = {}
    # add first link and joint to the dictionaries
    first_link = root.find(".//link")
    if first_link is None or "name" not in first_link.attrib:
        raise ValueError("Root link not found or has no name attribute")
    link_transforms[first_link.attrib["name"]] = np.eye(4)

    # get all joints from root
    joints = root.findall(".//joint")
    for joint in joints:
        # get parent info
        parent_element = joint.find("parent")
        if parent_element is None or "link" not in parent_element.attrib:
            raise ValueError("Parent element not found or has no link attribute in joint")
        parent_name = parent_element.attrib["link"]
        parent = get_link_by_name(root, parent_name)
        if parent is None:
            raise ValueError(f"Parent link {parent_name!r} not found in the URDF")
        # get child info
        child_element = joint.find("child")
        if child_element is None or "link" not in child_element.attrib:
            raise ValueError("Child element not found or has no link attribute in joint")
        child_name = child_element.attrib["link"]
        child = get_link_by_name(root, child_name)
        if child is None:
            raise ValueError(f"Child link {child_name!r} not found in the URDF")

        # get the relative transform between the two joints
        origin_element = joint.find("origin")
        if origin_element is None or "xyz" not in origin_element.attrib or "rpy" not in origin_element.attrib:
            raise ValueError("Origin element not found or missing required attributes in joint")
        relative_origin = string_to_nparray(origin_element.attrib["xyz"])
        relative_rpy = string_to_nparray(origin_element.attrib["rpy"])
        relative_transform = origin_and_rpy_to_transform(relative_origin, relative_rpy)
        # populate dictionary for each link
        if parent_name in link_transforms:
            link_transforms[child_name] = np.dot(link_transforms[parent_name], relative_transform)
            joint_transforms[joint.attrib["name"]] = link_transforms[child_name]
        else:
            link_transforms[parent_name] = np.eye(4)
            link_transforms[child_name] = np.dot(link_transforms[parent_name], relative_transform)
            joint_transforms[joint.attrib["name"]] = link_transforms[child_name]
    return link_transforms, joint_transforms


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
    mesh_transforms: dict[str, np.ndarray],
    urdf_path: Path,
    scaling: float,
) -> ET.Element:
    parent_name = get_part_name(parent)
    child_name = get_part_name(child)
    # make sure name is always oldest parent name
    if parent_name.startswith("fused_component_"):
        new_part_name = parent_name
    else:
        new_part_name = "fused_component_" + parent_name

    mesh_dir = urdf_path.parent / "meshes"
    parent_mesh = load_file(mesh_dir / f"{parent_name}.stl")
    child_mesh = load_file(mesh_dir / f"{child_name}.stl")
    # pare down parent name to just the name of the mesh
    if parent_name.startswith("fused_component_"):
        parent_name = parent_name[16:]
    if not parent_name.startswith("link_"):
        parent_name = "link_" + parent_name
    # process child name similarly
    if child_name.startswith("fused_component_"):
        child_name = child_name[16:]
    if not child_name.startswith("link_"):
        child_name = "link_" + child_name
    parent_transform_inv = np.linalg.inv(mesh_transforms[parent_name])
    relative_transform = np.dot(parent_transform_inv, mesh_transforms[child_name])
    combined_mesh = combine_meshes(parent_mesh, child_mesh, relative_transform)
    combined_mesh.save(mesh_dir / f"{new_part_name}.stl")

    # We don't do anything with collision meshes for now
    # collision_mesh = deepcopy(combined_mesh)
    # collision_mesh = scale_mesh(collision_mesh, scaling)
    # collision_mesh.save(mesh_dir / f"{new_part_name}_collision.stl")

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
    create_visual_and_collision_elements("collision", f"{new_part_name}.stl")

    return new_part


def process_fixed_joints(urdf_etree: ET.ElementTree, scaling: float, urdf_path: Path) -> ET.ElementTree:
    """Processes the fixed joints in the assembly."""
    if urdf_etree is None:
        raise ValueError("Invalid URDF etree.")
    root = urdf_etree.getroot()
    # Create dictionary to store each mesh position relative to its oldest parent
    mesh_positions, joint_positions = find_all_mesh_transforms(root)

    # While there still exists fixed joints, fuse the parts they connect.
    while True:
        joint = find_fixed_joint(root)
        if joint is None:
            break

        # Get the parent and child of the fixed joint
        parent_element = joint.find("parent")
        if parent_element is None or "link" not in parent_element.attrib:
            raise ValueError("Parent element not found or has no link attribute in joint")
        parent_name = parent_element.attrib["link"]
        parent = get_link_by_name(root, parent_name)
        if parent is None:
            raise ValueError(f"Parent link {parent_name!r} not found in the URDF")

        child_element = joint.find("child")
        if child_element is None or "link" not in child_element.attrib:
            raise ValueError("Child element not found or has no link attribute in joint")
        child_name = child_element.attrib["link"]
        child = get_link_by_name(root, child_name)
        if child is None:
            raise ValueError(f"Child link {child_name!r} not found in the URDF")

        # Get the relative transform between the two joints
        logger.info("Fusing parts: %s, %s.", parent_name, child_name)
        origin_element = joint.find("origin")
        if origin_element is None:
            raise ValueError("Origin element not found in joint")

        # Remove the fixed child link
        for link in root.findall(".//link"):
            if "name" in link.attrib and link.attrib["name"] in [parent_name, child_name]:
                root.remove(link)

        # Fuse the parts and add to second index of etree to preserve central node
        new_part = combine_parts(parent, child, mesh_positions, urdf_path, scaling)
        root.insert(1, new_part)

        # Replace the parent and child at all auxiliary joints with the new part
        for aux_joint in root.findall(".//joint"):
            if aux_joint == joint:
                continue
            if aux_joint is None:
                raise ValueError("Corrupted joint element found in urdf.")
            parent_element = aux_joint.find("parent")
            child_element = aux_joint.find("child")
            if (
                parent_element is None
                or "link" not in parent_element.attrib
                or child_element is None
                or "link" not in child_element.attrib
            ):
                raise ValueError("Parent or child element not found or has no link attribute in joint during update")
            if parent_element.attrib["link"] in [parent_name, child_name]:
                parent_element.attrib["link"] = new_part.attrib["name"]
                # Remap joint to be relative to the new part
                if aux_joint.attrib["name"] in joint_positions:
                    parent_name = parent_element.attrib["link"]
                    if parent_name.startswith("fused_component_"):
                        parent_name = parent_name[16:]
                    if not parent_name.startswith("link_"):
                        parent_name = "link_" + parent_name
                    new_joint_transform = np.dot(
                        np.linalg.inv(mesh_positions[parent_name]),
                        joint_positions[aux_joint.attrib["name"]],
                    )
                    # get the new relative origin and rpy
                    new_relative_origin = new_joint_transform[:3, 3]
                    new_relative_rpy = R.from_matrix(new_joint_transform[:3, :3]).as_euler("xyz")
                    origin_elem = aux_joint.find("origin")
                    if origin_elem is None:
                        raise ValueError("Origin element not found in aux joint during update")
                    origin_elem.attrib["xyz"] = " ".join(map(str, new_relative_origin))
                    origin_elem.attrib["rpy"] = " ".join(map(str, new_relative_rpy))
            if child_element.attrib["link"] in [child_name, parent_name]:
                child_element.attrib["link"] = new_part.attrib["name"]
        root.remove(joint)

    return urdf_etree


def get_merged_urdf(
    urdf_path: Path,
    scaling: float,
) -> None:
    """Merges meshes at each fixed joints to avoid collision issues.

    Args:
        urdf_path: The path to the urdf file.
        scaling: The scaling factor to apply to the meshes.

    Returns:
        The path to the merged urdf file.
    """
    if scaling < 0:
        raise ValueError(f"Scaling {scaling} should be greater than 0.")

    # Load the URDF file
    logger.info("Getting element tree from mesh filepath.")
    urdf_tree = ET.parse(urdf_path)
    # Process the fixed joints
    starting_joint_count = len(urdf_tree.findall(".//joint"))
    logger.info("Processing fixed joints, starting joint count: %d", starting_joint_count)
    merged_urdf = process_fixed_joints(urdf_tree, scaling, urdf_path)
    if merged_urdf is None:
        raise ValueError("Failed to merge fixed joints.")
    ending_joint_count = len(merged_urdf.findall(".//joint"))
    logger.info("Finished processing fixed joints, ending joint count: %d", ending_joint_count)
    logger.info("Removed %d fixed joints.", starting_joint_count - ending_joint_count)
    logger.info("Percent reduction: %.4f%%", 100 * (starting_joint_count - ending_joint_count) / starting_joint_count)

    # Save the merged URDF
    merged_urdf_path = urdf_path.parent / f"{urdf_path.stem}_merged.urdf"
    save_xml(merged_urdf_path, merged_urdf)
    logger.info("Saved merged URDF to %s.", merged_urdf_path)
