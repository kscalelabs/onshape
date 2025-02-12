"""Rotates the base of the URDF."""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from scipy.spatial.transform import Rotation as R

logger = logging.getLogger(__name__)


def rotate_base(urdf_path: Path, quaternion: tuple[float, float, float, float]) -> None:
    """Rotates the base of the URDF.

    Args:
        urdf_path: The path to the URDF file.
        quaternion: The quaternion to rotate the base by (x, y, z, w).
    """
    if quaternion == (0.0, 0.0, 0.0, 1.0):
        return

    # Parse the URDF file
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Find the first link (base link)
    base_link = root.find("link")
    if base_link is None:
        logger.warning("No base link found in URDF")
        return
    base_link_name = base_link.attrib.get("name")
    if base_link_name is None:
        logger.warning("No name found for base link in URDF")
        return

    rotation = R.from_quat(quaternion)

    # Updates the rotation of the base link.
    for element_type in ["visual", "collision", "inertial"]:
        element = base_link.find(element_type)
        if element is not None:
            origin = element.find("origin")
            if origin is None:
                origin = ET.SubElement(element, "origin")

            rpy_str = origin.get("rpy", "0 0 0")
            rpy = R.from_euler("xyz", [float(r.strip()) for r in rpy_str.split()], degrees=False)
            rpy = rotation * rpy
            origin.set("rpy", " ".join(f"{r:.2f}" for r in rpy.as_euler("xyz", degrees=False)))

    # Updates the rotation of each of the base joints connected to the base link.
    for joint in root.findall("joint"):
        if (parent := joint.find("parent")) is None or parent.attrib.get("link") != base_link_name:
            continue
        logger.info("Rotating joint %s", joint.attrib.get("name"))
        if (origin := joint.find("origin")) is None:
            origin = ET.SubElement(joint, "origin")

        rpy_str = origin.get("rpy", "0 0 0")
        xyz_str = origin.get("xyz", "0 0 0")
        rpy = R.from_euler("xyz", [float(r.strip()) for r in rpy_str.split()], degrees=False)
        xyz = [float(x.strip()) for x in xyz_str.split()]
        rpy = rotation * rpy
        origin.set("rpy", " ".join(f"{r:.2f}" for r in rpy.as_euler("xyz", degrees=False)))
        origin.set("xyz", " ".join(f"{x:.2f}" for x in rotation.apply(xyz)))

    # Save the modified URDF
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
    logger.info("Updated base link rotation in %s", urdf_path)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="Rotate the base of a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    parser.add_argument(
        "--quaternion",
        type=float,
        nargs=4,
        default=[1.0, 0.0, 0.0, 0.0],
        help="The quaternion to rotate by (w, x, y, z)",
    )
    args = parser.parse_args()

    rotate_base(args.urdf_path, tuple(args.quaternion))


if __name__ == "__main__":
    # python -m kol.passes.rotate_base
    main()
