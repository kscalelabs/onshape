"""Excludes specified collision meshes from the URDF."""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Collection

from onshape.formats.common import save_xml

logger = logging.getLogger(__name__)


def exclude_collision_meshes(urdf_path: Path, exclude_names: Collection[str]) -> None:
    """Removes collision meshes with specified names from the URDF.

    Args:
        urdf_path: The path to the URDF file.
        exclude_names: List of collision mesh names to exclude.
    """
    if len(exclude_names) == 0:
        return

    exclude_names = set(exclude_names)

    urdf_tree = ET.parse(urdf_path)
    root = urdf_tree.getroot()

    all_names: set[str] = set()
    processed: set[str] = set()
    for link in root.findall(".//link"):
        if (name := link.attrib.get("name")) is None:
            continue
        all_names.add(name)
        if name not in exclude_names:
            continue
        for collision in link.findall("collision"):
            link.remove(collision)
            processed.add(name)
            logger.info("Removed collision mesh from link '%s'", name)

    not_processed = exclude_names - processed
    if not_processed:
        all_possible_names_str = "\n".join(sorted(all_names))
        raise ValueError(
            f"Collision meshes not found in URDF: {not_processed}\n"
            f"Here are the possible names:\n{all_possible_names_str}"
        )

    save_xml(urdf_path, urdf_tree)


def main() -> None:
    parser = argparse.ArgumentParser(description="Exclude collision meshes from a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    parser.add_argument("--exclude", nargs="+", help="Names of collision meshes to exclude.")
    args = parser.parse_args()

    exclude_collision_meshes(args.urdf_path, args.exclude)


if __name__ == "__main__":
    main()
