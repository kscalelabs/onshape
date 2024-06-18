"""Removes all meshes not mentioned in a urdf from the ./meshes directory."""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Sequence

logger = logging.getLogger(__name__)


def main(args: Sequence[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser(description="Cleanup meshes directory")
    parser.add_argument("filepath", type=str, help="The path to the urdf file")
    parsed_args = parser.parse_args(args)

    filepath = Path(parsed_args.filepath)
    urdf_tree = ET.parse(filepath)
    deleted = 0
    logger.info("Cleaning up obsolete meshes.")
    mesh_dir = filepath.parent / "meshes"

    all_links = urdf_tree.findall(".//link")
    all_stl_names = []
    for link in all_links:
        visual = link.find("visual")
        if visual is not None:
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                all_stl_names.append(mesh.attrib["filename"])
    for link in all_links:
        collision = link.find("collision")
        if collision is not None:
            mesh = collision.find("geometry/mesh")
            if mesh is not None:
                all_stl_names.append(mesh.attrib["filename"])
    all_stl_names = [name.split("/")[-1] for name in all_stl_names]
    for mesh_file in mesh_dir.glob("*.stl"):
        if mesh_file.name not in all_stl_names:
            mesh_file.unlink()
            deleted += 1
    logger.info("Cleaned up %d meshes.", deleted)