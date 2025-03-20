"""Defines a pass that rotates certain joints."""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Collection, Mapping

from scipy.spatial.transform import Rotation as R

from onshape.formats.common import save_xml

logger = logging.getLogger(__name__)


def rotate_joints(urdf_path: Path, joints: Mapping[str, Collection[float | int]]) -> None:
    """Rotates the specified joints by the given RPY values.

    Args:
        urdf_path: The path to the URDF file.
        joints: The joints to rotate, and the RPY values to rotate them by.
    """
    urdf_tree = ET.parse(urdf_path)
    root = urdf_tree.getroot()
    joint_elems = {joint.attrib["name"]: joint for joint in root.findall("joint") if "name" in joint.attrib}

    for joint_name, rpy_vals in joints.items():
        if len(rpy_vals) != 3:
            raise ValueError(f"Expected 3 RPY values for joint {joint_name}, got {len(rpy_vals)}")
        if joint_name not in joint_elems:
            options = "\n".join(joint_elems.keys())
            raise ValueError(f"Joint {joint_name} not found in {urdf_path}. Options are:\n{options}")
        joint = joint_elems[joint_name]
        rpy_tf = R.from_euler("xyz", rpy_vals, degrees=True)
        if (origin := joint.find("origin")) is None:
            raise ValueError(f"Joint {joint_name} has no origin")
        joint_rpy = R.from_euler("xyz", [float(i) for i in origin.attrib["rpy"].split()], degrees=False)
        rpy_tf = rpy_tf * joint_rpy
        rpy_str = " ".join(map(str, rpy_tf.as_euler("xyz", degrees=False)))
        origin.attrib["rpy"] = rpy_str

    save_xml(urdf_path, root)


def main() -> None:
    parser = argparse.ArgumentParser(description="Rotate joints in a URDF file.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    parser.add_argument("joint", type=str, help="The joint name to rotate.")
    parser.add_argument("rpy", type=float, nargs=3, help="The RPY values to rotate the joint by.")
    args = parser.parse_args()

    r, p, y = args.rpy
    rotate_joints(args.urdf_path, {args.joint: (r, p, y)})


if __name__ == "__main__":
    # python -m onshape.passes.rotate_joints
    main()
