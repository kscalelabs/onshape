"""Defines a pass to flip the orientation of specific joints."""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Collection

from kol.formats.common import save_xml

logger = logging.getLogger(__name__)


def _str(i: float) -> str:
    return str(0 if i == 0 else i)


def flip_joints(urdf_path: Path, joint_names: Collection[str]) -> None:
    joint_names = set(joint_names)
    urdf_tree = ET.parse(urdf_path)
    for joint in urdf_tree.iter("joint"):
        if (joint_name := joint.attrib["name"]) in joint_names:
            if (axis := joint.find("axis")) is None or "xyz" not in axis.attrib:
                raise ValueError(f"Joint {joint_name} has invalid axis.")
            new_xyz = [-float(i) for i in axis.attrib["xyz"].strip().split()]
            axis.attrib["xyz"] = " ".join([_str(i) for i in new_xyz])
            if (limit := joint.find("limit")) is not None:
                if "lower" in limit.attrib:
                    lower = float(limit.attrib["lower"])
                    limit.attrib["upper"] = _str(-lower)
                if "upper" in limit.attrib:
                    upper = float(limit.attrib["upper"])
                    limit.attrib["lower"] = _str(-upper)
            joint_names.remove(joint_name)
    if joint_names:
        raise ValueError(f"Joints not found: {joint_names}")
    save_xml(urdf_path, urdf_tree)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF to flip.")
    parser.add_argument("joint_names", nargs="+", help="The names of the joints to flip.")
    args = parser.parse_args()
    flip_joints(args.urdf_path, args.joint_names)
