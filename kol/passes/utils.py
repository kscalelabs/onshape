"""Defines common utility functions used by multiple passes."""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Iterator

import numpy as np


def iter_meshes(
    urdf_path: Path,
    no_collision_mesh: bool = False,
    save_when_done: bool = False,
) -> Iterator[
    tuple[
        ET.Element,
        tuple[ET.Element, Path] | tuple[None, None],
        tuple[ET.Element, Path] | tuple[None, None],
    ]
]:
    urdf_tree = ET.parse(urdf_path)

    def get_mesh(visual_or_collision: ET.Element | None) -> tuple[ET.Element, Path] | tuple[None, None]:
        if visual_or_collision is None:
            return (None, None)
        if (geometry := visual_or_collision.find("geometry")) is None:
            return (None, None)
        if (mesh := geometry.find("mesh")) is None:
            return (None, None)
        return mesh, (urdf_path.parent / mesh.attrib["filename"]).resolve()

    for link in urdf_tree.iter("link"):
        visual_link = link.find("visual")
        collision_link = link.find("collision")

        if no_collision_mesh:
            if collision_link is not None:
                raise ValueError("Collision links should not exist.")
            visual_mesh = get_mesh(visual_link)
            yield link, visual_mesh, (None, None)

        else:
            if visual_link is None or collision_link is None:
                if visual_link is not None or collision_link is not None:
                    raise ValueError("Visual and collision links must be present together.")
                continue
            visual_mesh = get_mesh(visual_link)
            collision_mesh = get_mesh(collision_link)

            if visual_mesh[0] is None or collision_mesh[0] is None:
                if visual_mesh[0] is not None or collision_mesh[0] is not None:
                    raise ValueError("Visual and collision meshes must be present together.")
                continue

            yield link, visual_mesh, collision_mesh

    if save_when_done:
        urdf_tree.write(urdf_path, encoding="utf-8", xml_declaration=True)


def string_to_nparray(string: str | bytes | Any) -> np.ndarray:  # noqa: ANN401
    """Converts a string to a numpy array.

    Args:
        string: The string to convert.

    Returns:
        The numpy array.
    """
    if isinstance(string, bytes):
        string = string.decode("utf-8")
    return np.array([float(item) for item in string.split(" ")])
