"""Makes separate collision meshes for each link in the URDF."""

import logging
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import trimesh

from onshape.passes.utils import iter_meshes
from onshape.utils.mesh import COLLISION_SUFFIX

logger = logging.getLogger(__name__)


def separate_collision_mesh(visual_mesh_path: Path) -> Path:
    collision_mesh_name = f"{visual_mesh_path.stem}{COLLISION_SUFFIX}{visual_mesh_path.suffix}"
    collision_mesh_path = visual_mesh_path.parent / collision_mesh_name
    if collision_mesh_path.exists():
        logger.warning("Collision mesh %s already exists for %s", collision_mesh_path, visual_mesh_path)
    shutil.copy2(visual_mesh_path, collision_mesh_path)
    return collision_mesh_path


def shrink_collision_mesh(collision_mesh_path: Path, shrink_factor: float) -> None:
    mesh = trimesh.load(collision_mesh_path)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Loaded mesh {collision_mesh_path} is not a Trimesh")
    mesh.apply_scale(shrink_factor)
    mesh.export(collision_mesh_path)


def shrink_collision_meshes(
    urdf_path: Path,
    shrink_factors: dict[str, float],
) -> None:
    """Shrinks some of the collision meshes in the URDF.

    Args:
        urdf_path: The path to the URDF file.
        shrink_factors: A dictionary mapping link names to shrink factors.
            If the shrink factor is <= 0, the collision mesh will be removed entirely.
    """
    # Make a copy that we can use to keep track of which links have been processed.
    shrink_factors = {k: v for k, v in shrink_factors.items()}

    num_separated = 0
    for link, (_, visual_mesh_path), (col_link, col_mesh_path) in iter_meshes(
        urdf_path,
        save_when_done=True,
    ):
        name = link.attrib["name"]

        if name not in shrink_factors:
            continue

        if col_link is None or col_mesh_path is None:
            raise ValueError(f"No collision mesh found for {name}")

        shrink_factor = shrink_factors.pop(name)

        # If shrink factor is <= 0, remove the collision mesh
        if shrink_factor <= 0:
            logger.info("Removing collision mesh for %s", name)
            col_link_parent = link.find(".//collision")
            if col_link_parent is None:
                raise ValueError(f"No collision link found for {name}")
            link.remove(col_link_parent)
            continue

        # If the collision mesh is the same as the visual mesh, we need to make
        # a new collision mesh.
        if visual_mesh_path is not None and visual_mesh_path == col_mesh_path:
            col_mesh_path = separate_collision_mesh(visual_mesh_path)
            rel_filename = col_mesh_path.relative_to(urdf_path.parent).as_posix()
            col_link.attrib["filename"] = rel_filename
            num_separated += 1

        # After doing this, we can shrink the collision mesh geometry.
        logger.info("Shrinking collision mesh %s by %.2f", col_mesh_path, shrink_factor)
        shrink_collision_mesh(col_mesh_path, shrink_factor)

    if len(shrink_factors) > 0:
        missing_keys = list(shrink_factors.keys())
        urdf_tree = ET.parse(urdf_path)
        options = [link.attrib["name"] for link in urdf_tree.findall(".//link")]
        raise ValueError(f"The following links were not processed: {missing_keys}. Options are {options}")

    if num_separated > 0:
        logger.info(
            "Created %d separate collision meshes for %s",
            num_separated,
            urdf_path.name,
        )
