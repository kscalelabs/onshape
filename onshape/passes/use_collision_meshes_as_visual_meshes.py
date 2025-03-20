"""Uses collision meshes as visual meshes in the URDF."""

import argparse
import logging
from pathlib import Path

from onshape.passes.utils import iter_meshes

logger = logging.getLogger(__name__)


def use_collision_meshes_as_visual_meshes(urdf_path: Path) -> None:
    """Updates specified links to use their collision meshes as visual meshes.

    If a link has no collision mesh, its visual mesh will be removed.

    Args:
        urdf_path: The path to the URDF file.
    """
    for link, (visual_mesh, visual_mesh_path), (col_mesh, col_mesh_path) in iter_meshes(
        urdf_path,
        save_when_done=True,
    ):
        name = link.attrib["name"]
        if col_mesh_path is None:
            logger.warning("No collision mesh found for %s, removing visual mesh", name)
            if visual_mesh is not None:
                visual_parent = link.find("visual")
                if visual_parent is not None:
                    link.remove(visual_parent)
            continue

        if visual_mesh is None:
            logger.warning("No visual mesh found for %s", name)
            continue

        if col_mesh is None:
            raise ValueError(f"Missing collision mesh element for {name}")

        if visual_mesh_path != col_mesh_path:
            visual_mesh.attrib["filename"] = col_mesh.attrib["filename"]


def main() -> None:
    parser = argparse.ArgumentParser(description="Use collision meshes as visual meshes in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    use_collision_meshes_as_visual_meshes(args.urdf_path)


if __name__ == "__main__":
    main()
