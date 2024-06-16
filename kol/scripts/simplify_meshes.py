"""Improve simulation speed by reducing the number of faces and vertices in each mesh of a urdf."""

import argparse
from pathlib import Path
from typing import Sequence

from kol.simplify_all import simplify_all


def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Simplify meshes in a urdf.")
    parser.add_argument("filepath", type=str, help="The path to the urdf file")
    parser.add_argument("--voxel-size", type=float, default=0.001, help="Size of the voxels that will be merged.")
    parsed_args = parser.parse_args(args)

    urdf_path = Path(parsed_args.filepath)
    simplify_all(
        urdf_path=urdf_path,
        voxel_size=parsed_args.voxel_size,
    )
