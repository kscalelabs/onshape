"""Runs mesh simpification on a single stl."""

import argparse
import logging
from pathlib import Path
from typing import Sequence

import open3d as o3d

logger = logging.getLogger(__name__)


def main(args: Sequence[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser(description="Simplify a mesh")
    parser.add_argument("filepath", type=str, help="The path to the mesh file")
    parser.add_argument("output", type=str, help="The path to save the simplified mesh")
    parser.add_argument("voxel_size", type=float, help="The voxel size for simplification")
    parsed_args = parser.parse_args(args)

    mesh = o3d.io.read_triangle_mesh(parsed_args.filepath)
    simple_mesh = mesh.simplify_vertex_clustering(
        voxel_size=parsed_args.voxel_size,
        contraction=o3d.geometry.SimplificationContraction.Average,
    )
    logger.info("Simplified mesh from %d to %d vertices", len(mesh.vertices), len(simple_mesh.vertices))
    output_file = Path(parsed_args.output)

    match output_file.suffix.lower():
        case ".ply":
            o3d.io.write_triangle_mesh(parsed_args.output, simple_mesh, write_ascii=True)
        case ".stl":
            simple_mesh.compute_vertex_normals()
            o3d.io.write_triangle_mesh(parsed_args.output, simple_mesh)
        case _:
            o3d.io.write_triangle_mesh(parsed_args.output, simple_mesh)
