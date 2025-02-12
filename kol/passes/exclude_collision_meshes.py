"""Excludes specified collision meshes from the URDF."""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from kol.formats.common import save_xml

logger = logging.getLogger(__name__)


def exclude_collision_meshes(urdf_path: Path, exclude_names: list[str]) -> None:
    """Removes collision meshes with specified names from the URDF.

    Args:
        urdf_path: The path to the URDF file.
        exclude_names: List of collision mesh names to exclude.
    """
    if not exclude_names:
        return

    urdf_tree = ET.parse(urdf_path)
    root = urdf_tree.getroot()

    excluded = []
    for link in root.findall(".//link"):
        for collision in link.findall("collision"):
            if "name" in collision.attrib and collision.attrib["name"].strip("_collision") in exclude_names:
                link.remove(collision)
                excluded.append(collision.attrib["name"].strip("_collision"))
                logger.info(
                    "Removed collision mesh '%s' from link '%s'",
                    collision.attrib["name"],
                    link.attrib.get("name", "unknown"),
                )

    for not_excluded in set(exclude_names) - set(excluded):
        logger.warning("Collision mesh '%s' not found in URDF", not_excluded)

    save_xml(urdf_path, urdf_tree)


def main() -> None:
    parser = argparse.ArgumentParser(description="Exclude collision meshes from a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    parser.add_argument("--exclude", nargs="+", help="Names of collision meshes to exclude.")
    args = parser.parse_args()

    exclude_collision_meshes(args.urdf_path, args.exclude)


if __name__ == "__main__":
    main()
