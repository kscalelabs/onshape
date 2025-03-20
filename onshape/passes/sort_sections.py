"""Sorts the sections in the URDF, putting the links at the end."""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from onshape.formats.common import save_xml

logger = logging.getLogger(__name__)


def sort_sections(urdf_path: Path) -> None:
    urdf_tree = ET.parse(urdf_path)
    root = urdf_tree.getroot()
    links = root.findall("link")
    for link in links:
        root.remove(link)
        root.append(link)
    save_xml(urdf_path, urdf_tree)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()
    sort_sections(args.urdf_path)
