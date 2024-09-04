"""Removes collision meshes from the URDF."""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional

logger = logging.getLogger(__name__)


def remove_collision_meshes(urdf_path: Path | str, keep_collisions: Optional[List[str]] = None) -> None:
    """Removes collision meshes from the URDF file, except for specified parts.

    Args:
        urdf_path (str): Path to the URDF file.
        keep_collisions (Optional[List[str]]): List of link names where collisions should be kept.
    """
    if keep_collisions is None:
        keep_collisions = []

    # Parse the URDF file
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Iterate through all links
    for link in root.findall("link"):
        link_name = link.get("name")

        # If the link is not in the keep_collisions list, remove its collision elements
        if link_name not in keep_collisions:
            collisions = link.findall("collision")
            for collision in collisions:
                link.remove(collision)

    # Save the modified URDF
    output_path = Path(urdf_path)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    logger.info("Modified URDF saved to %s", output_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Remove collision meshes from URDF.")
    parser.add_argument("urdf_path", type=str, help="Path to the input URDF file")
    parser.add_argument("--keep", nargs="*", default=[], help="List of link names to keep collisions")

    args = parser.parse_args()

    remove_collision_meshes(args.urdf_path, args.keep)
