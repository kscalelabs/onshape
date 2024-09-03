"""Defines a pass to reduce the number of vertices in each mesh."""

import argparse
import logging
from pathlib import Path

from kol.passes.utils import iter_meshes

logger = logging.getLogger(__name__)


def simplify_mesh(mesh_path: Path, voxel_size: float) -> tuple[int, int]:
    try:
        import open3d as o3d
    except ImportError:
        logger.error(
            "Open3D is required to run this script. Install it with `pip install "
            "'kscale-onshape-library[open3d]'` to install the required dependencies."
        )
        raise

    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    simple_mesh = mesh.simplify_vertex_clustering(
        voxel_size=voxel_size,
        contraction=o3d.geometry.SimplificationContraction.Average,
    )
    pre_num_vertices = len(mesh.vertices)
    post_num_vertices = len(simple_mesh.vertices)

    match mesh_path.suffix.lower():
        case ".ply":
            o3d.io.write_triangle_mesh(str(mesh_path), simple_mesh, write_ascii=True)
        case ".stl":
            simple_mesh.compute_vertex_normals()
            o3d.io.write_triangle_mesh(str(mesh_path), simple_mesh)
        case _:
            o3d.io.write_triangle_mesh(str(mesh_path), simple_mesh)

    return pre_num_vertices, post_num_vertices


def get_simplified_urdf(
    urdf_path: Path,
    voxel_size: float = 0.002,
) -> None:
    """Merges meshes at each fixed joints to avoid collision issues.

    Args:
        urdf_path: The path to the urdf file.
        voxel_size: The voxel size to use for simplifying the meshes.

    Returns:
        The path to the merged urdf file.
    """
    # Iterates through each link in the URDF and simplifies the meshes.
    total_pre_num_vertices = 0
    total_post_num_vertices = 0
    for (_, visual_mesh_path), (_, collision_mesh_path) in iter_meshes(urdf_path):
        for mesh_path in list({visual_mesh_path, collision_mesh_path}):
            if mesh_path is not None:
                pre_num_vertices, post_num_vertices = simplify_mesh(mesh_path, voxel_size=voxel_size)
                total_pre_num_vertices += pre_num_vertices
                total_post_num_vertices += post_num_vertices

    if total_pre_num_vertices > 0:
        percent_reduction = (1 - total_post_num_vertices / total_pre_num_vertices) * 100
        logger.info(
            "Simplified meshes from %d to %d vertices (%.2f%% reduction)",
            total_pre_num_vertices,
            total_post_num_vertices,
            percent_reduction,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Merge fixed joints in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    parser.add_argument("--voxel-size", type=float, default=0.002, help="The voxel size for simplification.")
    args = parser.parse_args()

    get_simplified_urdf(args.urdf_path, voxel_size=args.voxel_size)


if __name__ == "__main__":
    # python -m kol.passes.simplify_meshes
    main()
