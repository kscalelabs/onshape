"""Defines a pass to update the names in a URDF."""

import argparse
import logging
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Mapping

from onshape.formats.common import save_xml

logger = logging.getLogger(__name__)


def find_not_null(element: ET.Element, key: str) -> ET.Element:
    value = element.find(key)
    if value is None:
        raise ValueError(f"{key} not found in {element.tag}")
    return value


def update_urdf_names(
    urdf_path: Path,
    joint_name_map: Mapping[str, str] | None = None,
    link_name_map: Mapping[str, str] | None = None,
) -> None:
    """Updates the names in a URDF.

    Args:
        urdf_path: The path to the URDF file.
        joint_name_map: A mapping from old joint names to new joint names.
        link_name_map: A mapping from old link names to new link names.
    """
    if joint_name_map is None:
        joint_name_map = {}
    if link_name_map is None:
        link_name_map = {}

    urdf_tree = ET.parse(urdf_path)
    root = urdf_tree.getroot()

    # By default, the link and joint names are quite verbose - they will look
    # something like "link:Upper_Body_(1):Torso_(1):Battery_Container_(1)".
    # Instead, we want to just use "Battery Container". However, we need to
    # make sure that there are no collisions with other names in the URDF, so
    # if there are duplicates then we add a suffix to make them unique.
    # Addtionally, we need to keep track of the original names so that we can
    # update all the links and joints which reference those names to be
    # correct.
    name_mapping: dict[str, str] = {}
    name_set: set[str] = set()

    def cleanup_name(name: str) -> str:
        base_name = name.split(":")[-1]
        base_name = re.sub(r"_\(\d+\)", "", base_name)
        base_name = re.sub(r"-", "_", base_name)
        return base_name

    def get_unique_name(name: str, name_set: set[str]) -> str:
        base_name = cleanup_name(name)
        count = 2
        unique_name = base_name
        while unique_name in name_set:
            unique_name = f"{base_name}_{count}"
            count += 1
        name_set.add(unique_name)
        return unique_name

    for joint in root.findall(".//joint"):
        if "name" in joint.attrib:
            old_name = joint.attrib["name"]
            new_name = joint_name_map.get(old_name, get_unique_name(old_name, name_set))
            name_mapping[old_name] = new_name
            joint.attrib["name"] = new_name

    for link in root.findall(".//link"):
        if "name" in link.attrib:
            old_name = link.attrib["name"]
            new_name = link_name_map.get(old_name, get_unique_name(old_name, name_set))
            name_mapping[old_name] = new_name
            link.attrib["name"] = new_name

    # Cleans up material names.
    for tag_name in ("material", "geometry", "visual", "inertial", "collision"):
        for tag in root.findall(f".//{tag_name}"):
            if "name" in tag.attrib:
                tag.attrib["name"] = get_unique_name(tag.attrib["name"], name_set)

    # Cleans up mesh STL names (note, this requires renaming the mesh files).
    new_mesh_paths: dict[str, Path] = {}
    mesh_name_set: set[str] = set()
    for mesh in root.findall(".//mesh"):
        if "filename" in mesh.attrib:
            mesh_relpath = mesh.attrib["filename"]
            if mesh_relpath not in new_mesh_paths:
                mesh_path = urdf_path.parent / mesh_relpath
                if not mesh_path.exists():
                    raise FileNotFoundError(mesh_path)
                new_mesh_path = mesh_path.with_name(get_unique_name(mesh_path.stem, mesh_name_set) + mesh_path.suffix)
                new_mesh_paths[mesh_relpath] = new_mesh_path
                mesh_path.rename(new_mesh_path)
            mesh.attrib["filename"] = new_mesh_paths[mesh_relpath].relative_to(urdf_path.parent).as_posix()

    # Now we need to update all the other links and joints which reference
    # these names to be correct.
    for joint in root.findall(".//joint"):
        parent = find_not_null(joint, "parent")
        child = find_not_null(joint, "child")
        parent.attrib["link"] = name_mapping[parent.attrib["link"]]
        child.attrib["link"] = name_mapping[child.attrib["link"]]

    # Registers the namespaces to avoid validation errors.
    ET.register_namespace("", "http://wiki.ros.org/urdf")
    ET.register_namespace("visual", "http://wiki.ros.org/collada_urdf")
    ET.register_namespace("collision", "http://wiki.ros.org/collada_urdf")

    save_xml(urdf_path, root)


def main() -> None:
    parser = argparse.ArgumentParser(description="Merge fixed joints in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    update_urdf_names(args.urdf_path)


if __name__ == "__main__":
    # python -m onshape.passes.simplify_meshes
    main()
