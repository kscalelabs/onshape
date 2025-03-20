"""Makes separate collision meshes for each link in the URDF."""

import argparse
import logging
import shutil
from pathlib import Path

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


def separate_collision_meshes_in_urdf(urdf_path: Path) -> None:
    """Creates separate collision mesh files for each link in a URDF.

    For each link that has both visual and collision meshes pointing to the same file,
    creates a separate collision mesh file and updates the URDF accordingly.

    Args:
        urdf_path: The path to the URDF file.
    """
    num_separated = 0
    for _, (_, visual_mesh_path), (collision_link_elem, collision_mesh_path) in iter_meshes(
        urdf_path,
        save_when_done=True,
    ):
        if visual_mesh_path is None or collision_link_elem is None or collision_mesh_path is None:
            continue

        if visual_mesh_path == collision_mesh_path:
            new_collision_path = separate_collision_mesh(visual_mesh_path)
            rel_filename = new_collision_path.relative_to(urdf_path.parent).as_posix()
            collision_link_elem.attrib["filename"] = rel_filename
            num_separated += 1

    if num_separated > 0:
        logger.info(
            "Created %d separate collision meshes for %s",
            num_separated,
            urdf_path.name,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Create separate collision meshes for each link in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    separate_collision_meshes_in_urdf(args.urdf_path)


if __name__ == "__main__":
    # python -m onshape.passes.separate_collision_meshes
    main()
