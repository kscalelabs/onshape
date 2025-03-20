"""Adds a new base linkage to the URDF."""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from onshape.formats.common import save_xml

logger = logging.getLogger(__name__)


def add_base_linkage(
    urdf_path: Path,
    base_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    link_name: str = "floating_base_link",
    joint_name: str = "floating_base_joint",
    joint_type: str = "fixed",
) -> None:
    """Adds a new base linkage to the URDF.

    Args:
        urdf_path: The path to the URDF file.
        base_rpy: The RPY to apply to the base linkage to orient the robot.
        link_name: The name of the new link to add.
        joint_name: The name of the joint connecting the new link.
        joint_type: The type of the joint (default is 'fixed').
    """
    # Parse the URDF file
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Gets the child link name as the first link in the URDF.
    if (child_link := root.find("link")) is None:
        raise ValueError("No child link found in URDF")
    child_link_name = child_link.attrib["name"]

    # Create a new link element
    new_link = ET.Element("link", name=link_name)
    visual = ET.SubElement(new_link, "visual", name=f"{link_name}_visual")
    geometry = ET.SubElement(visual, "geometry", name=f"{link_name}_geometry")
    ET.SubElement(geometry, "sphere", radius="0.01")
    material = ET.SubElement(visual, "material", name=f"{link_name}_material")
    ET.SubElement(material, "color", rgba="1 0 0 1")
    ET.SubElement(visual, "origin", xyz="0 0 0", rpy=" ".join(f"{r:.2f}" for r in base_rpy))

    inertial = ET.SubElement(new_link, "inertial", name=f"{link_name}_inertial")
    ET.SubElement(inertial, "mass", value="0.001")
    ET.SubElement(inertial, "inertia", ixx="0.000001", iyy="0.000001", izz="0.000002", ixy="0", ixz="0", iyz="0")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy=" ".join(f"{r:.2f}" for r in base_rpy))

    # Create a new joint element
    new_joint = ET.Element("joint", name=joint_name, type=joint_type)
    ET.SubElement(new_joint, "parent", link=link_name)
    ET.SubElement(new_joint, "child", link=child_link_name)
    ET.SubElement(new_joint, "origin", xyz="0 0 0", rpy=" ".join(f"{r:.2f}" for r in base_rpy))

    # Insert the new link and joint at the start of the URDF
    root.insert(0, new_link)
    root.insert(1, new_joint)

    # Save the modified URDF
    logger.info("Added new base linkage %s with joint %s to %s", link_name, joint_name, urdf_path)

    save_xml(urdf_path, root)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Add a new base linkage to a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    parser.add_argument("link_name", type=str, help="The name of the new link to add.")
    parser.add_argument("joint_name", type=str, help="The name of the joint connecting the new link.")
    parser.add_argument(
        "--joint_type",
        type=str,
        default="fixed",
        help="The type of the joint (default is 'fixed')",
    )
    args = parser.parse_args()

    add_base_linkage(args.urdf_path, args.link_name, args.joint_name, args.joint_type)


if __name__ == "__main__":
    # python -m onshape.passes.add_base_linkage
    main()
