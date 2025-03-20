"""Defines a pass to add a small separation between adjacent joints."""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from onshape.formats.common import save_xml
from onshape.passes.utils import string_to_nparray

logger = logging.getLogger(__name__)


def add_joint_separation(urdf_path: Path, joint_separation_distance: float) -> None:
    """Adds a small separation between joints in a URDF.

    For each joint, moves the joint origin along its local z-axis by the specified distance.
    This helps prevent numerical issues in physics simulation.

    The separation is applied in the joint's local coordinate frame, so it takes into account
    the joint's RPY rotation before applying the translation.

    Args:
        urdf_path: The path to the URDF file.
        joint_separation_distance: Distance in meters to separate joints by.
    """
    urdf_tree = ET.parse(urdf_path)
    root = urdf_tree.getroot()

    for joint in root.findall(".//joint"):
        # Get or create the origin element
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin")
            origin.attrib["xyz"] = "0 0 0"
            origin.attrib["rpy"] = "0 0 0"

        # Get the joint axis, defaulting to z-axis if not specified
        axis_elem = joint.find("axis")
        if axis_elem is not None:
            axis = string_to_nparray(axis_elem.attrib.get("xyz", "0 0 1"))
            # Normalize the axis vector
            axis = axis / np.linalg.norm(axis)
        else:
            axis = np.array([0, 0, 1])

        # Get current xyz and rpy
        xyz = string_to_nparray(origin.attrib.get("xyz", "0 0 0"))
        rpy = string_to_nparray(origin.attrib.get("rpy", "0 0 0"))

        # Create rotation matrix from rpy
        rotation = R.from_euler("xyz", rpy)

        # Transform the axis vector by the rotation
        transformed_axis = rotation.apply(axis)

        # Scale to desired length and add to current position
        separation_vector = transformed_axis * -joint_separation_distance
        new_xyz = xyz + separation_vector

        # Update the origin xyz
        origin.attrib["xyz"] = " ".join(str(x) for x in new_xyz)

    # Save the modified URDF
    save_xml(urdf_path, urdf_tree)
    logger.info(
        "Added %.3f meter separation to all joints in %s",
        joint_separation_distance,
        urdf_path.name,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Add separation between joints in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    parser.add_argument(
        "--separation",
        type=float,
        default=0.001,
        help="Distance in meters to separate joints by",
    )
    args = parser.parse_args()

    add_joint_separation(args.urdf_path, args.separation)


if __name__ == "__main__":
    # python -m onshape.passes.add_joint_separation
    main()
