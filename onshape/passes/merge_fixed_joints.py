"""Defines functions for merging URDF parts at fixed joints."""

import argparse
import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, get_args

import numpy as np
from scipy.spatial.transform import Rotation as R

from onshape.formats.common import save_xml
from onshape.passes.utils import string_to_nparray
from onshape.utils.geometry import (
    Dynamics,
    apply_transform,
    combine_dynamics,
    combine_meshes,
    matrix_to_moments,
    moments_to_matrix,
    transform_inertia_tensor,
)
from onshape.utils.mesh import load_file, save_file

logger = logging.getLogger(__name__)

MAX_NAME_LENGTH: int = 64

MeshKind = Literal["visual", "collision"]


@dataclass
class JointOrigin:
    xyz: np.ndarray
    rpy: np.ndarray


def mesh_kinds() -> list[MeshKind]:
    return list(get_args(MeshKind))


def find_not_null(element: ET.Element, key: str) -> ET.Element:
    value = element.find(key)
    if value is None:
        raise ValueError(f"{key} not found in {element.tag}")
    return value


def get_part_mesh_path(element: ET.Element, urdf_dir: Path, kind: MeshKind | None = None) -> Path:
    if kind is None:
        mesh_paths = [get_part_mesh_path(element, urdf_dir, mesh_kind) for mesh_kind in mesh_kinds()]
        if any(mesh_path != mesh_paths[0] for mesh_path in mesh_paths):
            raise ValueError(f"Mesh paths for {element.tag} link do not match")
        return mesh_paths[0]

    mesh_elem = find_not_null(element, kind)
    geometry_elem = find_not_null(mesh_elem, "geometry")
    mesh_elem = find_not_null(geometry_elem, "mesh")
    filename = mesh_elem.attrib["filename"]
    return (urdf_dir / filename).resolve()


def get_new_rpy(
    mass_1: float,
    mass_2: float,
    rpy_1: np.ndarray,
    rpy_2: np.ndarray,
) -> np.ndarray:
    """Gets the new roll, pitch and yaw of the combined parts.

    Args:
        mass_1: The mass of the first part.
        mass_2: The mass of the second part.
        rpy_1: The roll, pitch and yaw of the first part.
        rpy_2: The roll, pitch and yaw of the second part.

    Returns:
        The new roll, pitch and yaw of the combined parts.
    """
    rotation_1 = R.from_euler("xyz", rpy_1).as_matrix()
    rotation_2 = R.from_euler("xyz", rpy_2).as_matrix()
    combined_rotation_matrix = (mass_1 * rotation_1 + mass_2 * rotation_2) / (mass_1 + mass_2)
    u, _, vt = np.linalg.svd(combined_rotation_matrix)
    corrected_rotation_matrix = np.dot(u, vt)
    new_rpy = R.from_matrix(corrected_rotation_matrix).as_euler("xyz")
    return new_rpy


def get_color(element: ET.Element) -> np.ndarray:
    visual_elem = find_not_null(element, "visual")
    material_elem = find_not_null(visual_elem, "material")
    color_elem = find_not_null(material_elem, "color")
    rgba = color_elem.attrib["rgba"].split()
    return np.array([float(x) for x in rgba])


def get_child_to_grandchild_joints(root: ET.Element, link: ET.Element) -> list[ET.Element]:
    link_name = link.attrib["name"]
    grandchild_links = []
    for joint in root.findall(".//joint"):
        parent = find_not_null(joint, "parent")
        if parent.attrib["link"] == link_name:
            grandchild_links.append(joint)
    return grandchild_links


def get_link(root: ET.Element, joint: ET.Element, key: str) -> ET.Element:
    link = find_not_null(joint, key)
    link_name = link.attrib["link"]
    link = root.find(f".//link[@name='{link_name}']")
    if link is None:
        raise ValueError(f"Link {link_name} not found")
    return link


def get_dynamics(inertial_link: ET.Element) -> Dynamics:
    mass_elem = find_not_null(inertial_link, "mass")
    origin_elem = find_not_null(inertial_link, "origin")
    inertia_elem = find_not_null(inertial_link, "inertia")

    # Gets the mass from the link.
    mass = float(mass_elem.attrib["value"])

    # Gets the center of mass from the link.
    xyz = string_to_nparray(origin_elem.attrib["xyz"])
    rpy = string_to_nparray(origin_elem.attrib["rpy"])

    # Gets the inertia matrix from the link.
    ixx = float(inertia_elem.attrib["ixx"])
    iyy = float(inertia_elem.attrib["iyy"])
    izz = float(inertia_elem.attrib["izz"])
    ixy = float(inertia_elem.attrib.get("ixy", "0"))
    ixz = float(inertia_elem.attrib.get("ixz", "0"))
    iyz = float(inertia_elem.attrib.get("iyz", "0"))
    inertia = moments_to_matrix(np.array([ixx, iyy, izz, ixy, ixz, iyz]))

    # Converts to the expected dynamics format.
    com = np.array(xyz)
    rotation_matrix = R.from_euler("xyz", rpy).as_matrix()
    inertia = transform_inertia_tensor(inertia, rotation_matrix)

    return Dynamics(mass, com, inertia)


def update_dynamics(dynamics: Dynamics, relative_transform: np.matrix) -> Dynamics:
    com = apply_transform(dynamics.com[None, :], relative_transform)[0]
    inertia = transform_inertia_tensor(dynamics.inertia, relative_transform[:3, :3])
    return Dynamics(dynamics.mass, com, inertia)


def set_dynamics(inertial_link: ET.Element, dynamics: Dynamics) -> None:
    mass_elem = find_not_null(inertial_link, "mass")
    origin_elem = find_not_null(inertial_link, "origin")
    inertia_elem = find_not_null(inertial_link, "inertia")

    mass_elem.attrib["value"] = str(dynamics.mass)
    origin_elem.attrib["xyz"] = " ".join(str(x) for x in dynamics.com)
    inertia_elem.attrib.update(matrix_to_moments(dynamics.inertia))


def get_origin(joint: ET.Element) -> tuple[ET.Element, JointOrigin]:
    origin_element = joint.find("origin")
    if origin_element is None:
        raise ValueError("Origin element not found in joint")
    xyz = string_to_nparray(origin_element.attrib.get("xyz", "0 0 0"))
    rpy = string_to_nparray(origin_element.attrib.get("rpy", "0 0 0"))
    return origin_element, JointOrigin(xyz, rpy)


def set_origin(origin_element: ET.Element, origin: JointOrigin) -> None:
    origin_element.attrib["xyz"] = " ".join(str(x) for x in origin.xyz)
    origin_element.attrib["rpy"] = " ".join(str(x) for x in origin.rpy)


def origin_and_rpy_to_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.matrix:
    relative_rotation_inner = R.from_euler("xyz", rpy).as_matrix()
    relative_rotation = np.eye(4)
    relative_rotation[:3, :3] = relative_rotation_inner
    relative_translation = np.eye(4)
    relative_translation[:3, 3] = xyz
    relative_transform = np.dot(relative_translation, relative_rotation)
    return relative_transform


def fuse_child_into_parent(root: ET.Element, joint: ET.Element, urdf_dir: Path) -> None:
    parent_link = get_link(root, joint, "parent")
    child_link = get_link(root, joint, "child")
    _, origin = get_origin(joint)

    # Gets the mesh paths for the parent and child.
    parent_mesh_path = get_part_mesh_path(parent_link, urdf_dir)
    child_mesh_path = get_part_mesh_path(child_link, urdf_dir)

    parent_mesh = load_file(parent_mesh_path)
    child_mesh = load_file(child_mesh_path)

    # Gets the transformation matrix.
    relative_transform = origin_and_rpy_to_transform(origin.xyz, origin.rpy)

    # Combines the two meshes into a single mesh.
    combined_mesh = combine_meshes(parent_mesh, child_mesh, relative_transform)

    # Combines the inertia of the parent and child links.
    parent_inertial = find_not_null(parent_link, "inertial")
    child_inertial = find_not_null(child_link, "inertial")
    parent_dynamics = get_dynamics(parent_inertial)
    child_dynamics = get_dynamics(child_inertial)
    child_dynamics = update_dynamics(child_dynamics, relative_transform)
    combined_dynamics = combine_dynamics(parent_dynamics, child_dynamics)
    set_dynamics(parent_inertial, combined_dynamics)

    # Delete the parent and chlid meshes and save the new mesh to the parent
    # mesh location.
    parent_mesh_path.unlink()
    child_mesh_path.unlink()
    save_file(combined_mesh, parent_mesh_path)

    # Removes the child link and joint from the URDF.
    root.remove(joint)
    root.remove(child_link)

    # For each child of the child link, update the parent link to be the parent
    # link of the child link, and update the relative origin.
    grandchild_joints = get_child_to_grandchild_joints(root, child_link)
    for grandchild_joint in grandchild_joints:
        if (grandchild_parent := grandchild_joint.find("parent")) is None:
            raise ValueError("Parent not found in grandchild link")
        grandchild_origin_element, grandchild_origin = get_origin(grandchild_joint)

        grandchild_relative_transform = origin_and_rpy_to_transform(grandchild_origin.xyz, grandchild_origin.rpy)
        new_relative_transform = np.dot(relative_transform, grandchild_relative_transform)

        grandchild_parent.attrib["link"] = parent_link.attrib["name"]
        grandchild_origin.xyz = new_relative_transform[:3, 3]
        grandchild_origin.rpy = R.from_matrix(new_relative_transform[:3, :3]).as_euler("xyz")
        set_origin(grandchild_origin_element, grandchild_origin)


def process_fixed_joints(
    urdf_etree: ET.ElementTree,
    urdf_path: Path,
    ignore_merging_fixed_joints: list[str] | None = None,
) -> ET.ElementTree:
    """Iteratively fuses all fixed joints until none remain.

    This greedily fuses all child links into their parent links at fixed joints,
    removing the fixed joints in the process.

    Args:
        urdf_etree: The URDF element tree.
        urdf_path: The path to the URDF file.
        ignore_merging_fixed_joints: The names of the fixed joints to ignore
            when merging.

    Returns:
        The URDF element tree with all fixed joints fused.
    """
    root = urdf_etree.getroot()

    ignore_set = set([] if ignore_merging_fixed_joints is None else ignore_merging_fixed_joints)
    visited_set: set[str] = set()

    starting_fixed_joint_names = [j.attrib["name"] for j in urdf_etree.findall(".//joint[@type='fixed']")]

    while True:
        joints = [j for j in urdf_etree.findall(".//joint[@type='fixed']") if j.attrib["name"] not in visited_set]
        if not joints:
            break
        for joint in joints:
            if joint.attrib["name"] in ignore_set:
                visited_set.add(joint.attrib["name"])
                continue
            fuse_child_into_parent(root, joint, urdf_path.parent)

    missed_ignored_joints = ignore_set - visited_set
    if missed_ignored_joints:
        raise ValueError(
            f"The following fixed joints were not processed: {missed_ignored_joints}. "
            f"The available fixed joints in the URDF are {sorted(starting_fixed_joint_names)}"
        )

    return urdf_etree


def get_merged_urdf(urdf_path: Path, ignore_merging_fixed_joints: list[str] | None = None) -> None:
    """Merges meshes at each fixed joints to avoid collision issues.

    Args:
        urdf_path: The path to the urdf file.
        ignore_merging_fixed_joints: The names of the fixed joints to ignore
            when merging.

    Returns:
        The path to the merged urdf file.
    """
    urdf_tree = ET.parse(urdf_path)

    # Process the fixed joints
    starting_joint_count = len(urdf_tree.findall(".//joint"))
    merged_urdf = process_fixed_joints(urdf_tree, urdf_path, ignore_merging_fixed_joints)
    ending_joint_count = len(merged_urdf.findall(".//joint"))
    logger.info(
        "Removed %d / %d fixed joints (%.4f%% reduction).",
        starting_joint_count - ending_joint_count,
        starting_joint_count,
        100 * (starting_joint_count - ending_joint_count) / starting_joint_count,
    )

    # Save the merged URDF
    save_xml(urdf_path, merged_urdf)


def main() -> None:
    parser = argparse.ArgumentParser(description="Merge fixed joints in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    get_merged_urdf(args.urdf_path)


if __name__ == "__main__":
    # python -m onshape.passes.merge_fixed_joints
    main()
