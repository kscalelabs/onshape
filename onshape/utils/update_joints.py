"""Defines functions for updating joint properties in a URDF."""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from onshape.formats.common import save_xml

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


def update_joints(
    urdf_path: Path,
    name_dict: dict[str, str] | None,
    override_fixed: list[str] | None,
    override_limits: dict[str, str] | None,
    override_torques: dict[str, int] | None,
) -> None:
    # Load the URDF file.
    logger.info("Updating joints.")
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Sort keys so we match joints to longest first.
    if name_dict:
        sorted_keys = sorted(name_dict.keys(), key=len, reverse=True)

    # Collect all joints in urdf and get their meshes.
    for joint in root.iter("joint"):
        joint_name = joint.attrib["name"]

        # Remove "joint_" from beginning of joint name.
        if joint_name.startswith("joint_"):
            joint_name = joint_name[6:]

        # Check if name_dict is not None and if the joint_name is a substring
        # of any key in name_dict.
        if name_dict:
            for key in sorted_keys:
                if key in joint_name:
                    joint.attrib["name"] = name_dict[key]
                    break

            # If joint has a mimic that needs to be updated as well.
            mimic = joint.find("mimic")
            if mimic is not None:
                mimic_name = mimic.attrib["joint"]
                for key in sorted_keys:
                    if key in mimic_name:
                        mimic.attrib["joint"] = name_dict[key]
                        break

        # Check if override_dict is not None and if the any element of override
        # is substring of joint_name then fix.
        if override_fixed and any(ov in joint_name for ov in override_fixed):
            joint.attrib["type"] = "fixed"

        if override_torques:
            for key in override_torques.keys():
                if key in joint_name:
                    limit = joint.find("limit")
                    if limit is not None:
                        limit.attrib["effort"] = str(override_torques[key])
                    break

    for joint in root.iter("joint"):
        joint_name = joint.attrib["name"]
        # Check if override_limits is not None and if any key in
        # override_limits is a substring of joint_name.
        # Note: we do this with the new joint names
        if override_limits:
            for key in override_limits.keys():
                if key in joint_name:
                    limit = joint.find("limit")
                    if limit:
                        limit.attrib["lower"] = str(override_limits[key][0])
                        limit.attrib["upper"] = str(override_limits[key][1])
                    break

    # Save the updated URDF to the same file.
    save_xml(urdf_path, tree)
