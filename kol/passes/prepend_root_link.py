"""Prepends a dummy root link to the URDF."""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from kol.formats.common import save_xml

logger = logging.getLogger(__name__)

DEFAULT_ORIGIN = {"rpy": "0 0 0", "xyz": "0 0 0"}
DEFAULT_MASS = {"value": "0.001"}
DEFAULT_INERTIA = {"ixx": "0.0001", "ixy": "0", "ixz": "0", "iyy": "0.0001", "iyz": "0", "izz": "0.0001"}


def prepend_root_link(urdf_path: Path) -> None:
    """Prepends a dummy root link to the URDF.

    Args:
        urdf_path: The path to the URDF file.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    if (first_link := root.find(".//link")) is None:
        raise ValueError("No link found in URDF")
    first_link_name = first_link.attrib["name"]

    # Create the root link
    root_link = ET.Element("link", {"name": "base"})
    inertial = ET.Element("inertial")
    origin = ET.Element("origin", DEFAULT_ORIGIN)
    mass = ET.Element("mass", DEFAULT_MASS)
    inertia = ET.Element("inertia", DEFAULT_INERTIA)
    inertial.append(origin)
    inertial.append(mass)
    inertial.append(inertia)
    root_link.append(inertial)

    # Create the floating base joint
    floating_base_joint = ET.Element("joint", {"name": "floating_base", "type": "fixed"})
    origin = ET.Element("origin", {"rpy": "0 0 0", "xyz": "0 0 0"})
    parent = ET.Element("parent", {"link": "base"})
    child = ET.Element("child", {"link": first_link_name})
    floating_base_joint.append(origin)
    floating_base_joint.append(parent)
    floating_base_joint.append(child)

    root.insert(0, root_link)
    root.insert(1, floating_base_joint)

    save_xml(urdf_path, root)


def main() -> None:
    parser = argparse.ArgumentParser(description="Prepend a dummy root link to a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    prepend_root_link(args.urdf_path)


if __name__ == "__main__":
    # python -m kol.passes.simplify_meshes
    main()
