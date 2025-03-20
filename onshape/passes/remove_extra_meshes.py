"""Removes any extra mesh files in the URDF directory."""

import argparse
import logging
from pathlib import Path

from onshape.passes.utils import iter_meshes

logger = logging.getLogger(__name__)


def remove_extra_meshes(urdf_path: Path) -> None:
    """Removes any extra mesh files in the URDF directory."""
    urdf_path = urdf_path.resolve()
    valid_paths = {urdf_path}
    mesh_dirs: set[Path] = set()
    for _, (_, visual_mesh_path), (_, collision_mesh_path) in iter_meshes(urdf_path):
        for path in list({visual_mesh_path, collision_mesh_path}):
            if path is not None:
                valid_paths.add(path)
                mesh_dirs.add(path.parent)

    all_meshes = {path for mesh_dir in mesh_dirs for path in mesh_dir.glob("**/*.stl")}
    for path in all_meshes:
        if path.is_file() and path not in valid_paths:
            logger.info("Removing %s", path)
            path.unlink()


def main() -> None:
    parser = argparse.ArgumentParser(description="Removes any extra mesh files in the URDF directory.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    remove_extra_meshes(args.urdf_path)


if __name__ == "__main__":
    # python -m onshape.passes.remove_extra_meshes
    main()
