"""Prepends a dummy root link to the URDF."""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from scipy.spatial.transform import Rotation

from kol.formats.common import save_xml

logger = logging.getLogger(__name__)


def prepend_root_link(
    urdf_path: Path,
    base_quaternion: tuple[float, float, float, float],
    default_mass: float = 0.001,
    default_inertia: float = 0.000001,
) -> None:
    """Prepends a dummy root link to the URDF.

    Args:
        urdf_path: The path to the URDF file.
        base_quaternion: The quaternion to apply to the base linkage to orient
            the robot, in (x, y, z, w) format.
        default_mass: The default mass to use for the root link.
        default_inertia: The default inertia to use for the root link.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    if (first_link := root.find(".//link")) is None:
        raise ValueError("No link found in URDF")
    first_link_name = first_link.attrib["name"]

    # Gets the RPY and XYZ from the quaternion.
    rpy = Rotation.from_quat(base_quaternion).as_euler("xyz", degrees=False)

    # Create the root link
    root_link = ET.Element("link", {"name": "base"})
    inertial = ET.Element("inertial")
    origin = ET.Element("origin", {"rpy": "0 0 0", "xyz": "0 0 0"})
    mass = ET.Element("mass", {"value": str(default_mass)})
    inertia = ET.Element(
        "inertia",
        {
            "ixx": str(default_inertia),
            "ixy": "0",
            "ixz": "0",
            "iyy": str(default_inertia),
            "iyz": "0",
            "izz": str(default_inertia),
        },
    )
    inertial.append(origin)
    inertial.append(mass)
    inertial.append(inertia)
    root_link.append(inertial)

    # Create the floating base joint
    floating_base_joint = ET.Element("joint", {"name": "floating_base", "type": "fixed"})
    origin = ET.Element("origin", {"rpy": " ".join(f"{r:.2f}" for r in rpy), "xyz": "0 0 0"})
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
    parser.add_argument(
        "-q",
        "--quaternion",
        type=float,
        nargs=4,
        default=(0.0, 0.0, 0.0, 1.0),
        help="The quaternion to apply to the base linkage to orient the robot, in (x, y, z, w) format.",
    )
    parser.add_argument(
        "--default-mass",
        type=float,
        default=0.001,
        help="The default mass to use for the root link.",
    )
    parser.add_argument(
        "--default-inertia",
        type=float,
        default=0.000001,
        help="The default inertia to use for the root link.",
    )
    args = parser.parse_args()

    prepend_root_link(
        args.urdf_path,
        base_quaternion=tuple(args.quaternion),
        default_mass=args.default_mass,
        default_inertia=args.default_inertia,
    )


if __name__ == "__main__":
    # python -m kol.passes.simplify_meshes
    main()
