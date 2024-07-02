"""Defines functions for updating joints in a urdf."""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from kol.formats.common import save_xml

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


def update_joints(
    urdf_path: Path,
    name_dict: dict[str, str] | None,
    override: list[str] | None,
) -> None:
    # Load the URDF file
    logger.info("Updating joints.")
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    # Collect all joints in urdf and get their meshes
    for joint in root.iter("joint"):
        joint_name = joint.attrib["name"]
        # Remove "joint_" from beginning of joint name
        if joint_name.startswith("joint_"):
            joint_name = joint_name[6:]

        # Check if name_dict is not None and if the joint_name is a substring of any key in name_dict
        if name_dict:
            for key in name_dict:
                if key in joint_name:
                    joint.attrib["name"] = name_dict[key]
            # If joint has a mimic that needs to be updated as well
            mimic = joint.find("mimic")
            if mimic is not None:
                mimic_name = mimic.attrib["joint"]
                for key in name_dict:
                    if key in mimic_name:
                        mimic.attrib["joint"] = name_dict[key]

        # Check if override_dict is not None and if the any element of override is substring of joint_name then fix
        if override and any(ov in joint_name for ov in override):
            joint.attrib["type"] = "fixed"

    # Save the updated URDF to the same file
    save_xml(urdf_path, tree)
