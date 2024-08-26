"""Defines common utility functions used by multiple passes."""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterator


def iter_meshes(urdf_path: Path) -> Iterator[Path]:
    urdf_tree = ET.parse(urdf_path)

    # Iterates through each link in the URDF and simplifies the meshes.
    for link in urdf_tree.iter("link"):
        for visual in link.iter("visual"):
            for geometry in visual.iter("geometry"):
                for mesh in geometry.iter("mesh"):
                    mesh_path = (urdf_path.parent / mesh.attrib["filename"]).resolve()
                    yield mesh_path
