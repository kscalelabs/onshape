"""Moves collision meshes by specified offsets."""

import logging
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import trimesh

from onshape.passes.utils import iter_meshes
from onshape.utils.mesh import COLLISION_SUFFIX

logger = logging.getLogger(__name__)


def separate_collision_mesh(visual_mesh_path: Path) -> Path:
    """Creates a separate collision mesh file from a visual mesh.

    Args:
        visual_mesh_path: Path to the visual mesh file.

    Returns:
        Path to the new collision mesh file.
    """
    collision_mesh_name = f"{visual_mesh_path.stem}{COLLISION_SUFFIX}{visual_mesh_path.suffix}"
    collision_mesh_path = visual_mesh_path.parent / collision_mesh_name
    if collision_mesh_path.exists():
        logger.warning("Collision mesh %s already exists for %s", collision_mesh_path, visual_mesh_path)
    shutil.copy2(visual_mesh_path, collision_mesh_path)
    return collision_mesh_path


def move_collision_mesh(collision_mesh_path: Path, translation: Sequence[float]) -> None:
    """Moves a collision mesh by a specified translation.

    Args:
        collision_mesh_path: Path to the collision mesh file.
        translation: [x, y, z] translation to apply to the mesh.
    """
    mesh = trimesh.load(collision_mesh_path)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Loaded mesh {collision_mesh_path} is not a Trimesh")
    translation = np.array(translation, dtype=np.float64)
    mesh.apply_translation(translation)
    mesh.export(collision_mesh_path)


def move_collision_meshes(
    urdf_path: Path,
    translations: Mapping[str, Sequence[float]],
) -> None:
    """Moves collision meshes in the URDF by specified translations.

    Args:
        urdf_path: The path to the URDF file.
        translations: A dictionary mapping link names to [x, y, z] translations.
    """
    # Make a copy that we can use to keep track of which links have been processed
    translations = {k: v for k, v in translations.items()}

    num_separated = 0
    for link, (_, visual_mesh_path), (col_link, col_mesh_path) in iter_meshes(
        urdf_path,
        save_when_done=True,
    ):
        name = link.attrib["name"]

        if name not in translations:
            continue

        if col_link is None or col_mesh_path is None:
            raise ValueError(f"No collision mesh found for {name}")

        # If the collision mesh is the same as the visual mesh, we need to make
        # a new collision mesh
        if visual_mesh_path is not None and visual_mesh_path == col_mesh_path:
            col_mesh_path = separate_collision_mesh(visual_mesh_path)
            rel_filename = col_mesh_path.relative_to(urdf_path.parent).as_posix()
            col_link.attrib["filename"] = rel_filename
            num_separated += 1

        # After doing this, we can move the collision mesh geometry
        translation = translations.pop(name)
        if len(translation) != 3:
            raise ValueError(f"Expected translation to be a list of 3 floats, got {translation}")
        logger.info("Moving collision mesh %s by %s", col_mesh_path, translation)
        move_collision_mesh(col_mesh_path, translation)

    if len(translations) > 0:
        missing_keys = list(translations.keys())
        urdf_tree = ET.parse(urdf_path)
        options = [link.attrib["name"] for link in urdf_tree.findall(".//link")]
        raise ValueError(f"The following links were not processed: {missing_keys}. Options are {options}")

    if num_separated > 0:
        logger.info(
            "Created %d separate collision meshes for %s",
            num_separated,
            urdf_path.name,
        )
