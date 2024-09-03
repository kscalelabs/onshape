"""Defines common utility functions used by multiple passes."""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterator


def iter_meshes(
    urdf_path: Path, remove_collision_meshes: bool = False
) -> Iterator[tuple[tuple[ET.Element, Path] | None, tuple[ET.Element, Path] | tuple[None, None]]]:
    urdf_tree = ET.parse(urdf_path)

    def get_mesh(visual_or_collision: ET.Element | None) -> tuple[ET.Element, Path] | None:
        if visual_or_collision is None:
            return None
        if (geometry := visual_or_collision.find("geometry")) is None:
            return None
        if (mesh := geometry.find("mesh")) is None:
            return None
        return mesh, (urdf_path.parent / mesh.attrib["filename"]).resolve()

    for link in urdf_tree.iter("link"):
        visual_link = link.find("visual")
        collision_link = link.find("collision")

        if remove_collision_meshes:
            if collision_link is not None:
                raise ValueError("Collision links should not exist.")
            visual_mesh = get_mesh(visual_link)
            yield visual_mesh, (None, None)
        else:
            if visual_link is None or collision_link is None:
                if visual_link is not None or collision_link is not None:
                    raise ValueError("Visual and collision links must be present together.")
                continue
            visual_mesh = get_mesh(visual_link)
            collision_mesh = get_mesh(collision_link)
            if visual_mesh is None or collision_mesh is None:
                if visual_mesh is not None or collision_mesh is not None:
                    raise ValueError("Visual and collision meshes must be present together.")
                continue

            yield visual_mesh, collision_mesh
