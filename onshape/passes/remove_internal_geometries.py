"""Defines a pass to remove internal geometry from meshes."""

import argparse
import logging
from pathlib import Path

from onshape.passes.utils import iter_meshes

logger = logging.getLogger(__name__)


def remove_internal_geometry(mesh_path: Path) -> None:
    """Removes internal geometry from a mesh file by keeping only the outer shell.

    Args:
        mesh_path: Path to the mesh file to process.
    """
    try:
        import open3d as o3d
    except ImportError:
        raise ImportError(
            "Open3D is required to run this script. Install it with `pip install "
            "'onshape[open3d]'` to install the required dependencies."
        )

    mesh = o3d.io.read_triangle_mesh(str(mesh_path))

    # Count initial triangles
    initial_triangles = len(mesh.triangles)

    # Ensure normals are computed
    mesh.compute_vertex_normals()

    # Clean up the resulting mesh
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()

    # Count final triangles and log if any were removed
    final_triangles = len(mesh.triangles)
    if final_triangles < initial_triangles:
        logger.info(
            "Removed %d triangles from %s (%.1f%% reduction)",
            initial_triangles - final_triangles,
            mesh_path.name,
            100 * (initial_triangles - final_triangles) / initial_triangles,
        )

    # Save the mesh back
    match mesh_path.suffix.lower():
        case ".ply":
            o3d.io.write_triangle_mesh(str(mesh_path), mesh, write_ascii=True)
        case ".stl":
            mesh.compute_vertex_normals()
            o3d.io.write_triangle_mesh(str(mesh_path), mesh)
        case _:
            o3d.io.write_triangle_mesh(str(mesh_path), mesh)


def remove_internal_geometries_from_urdf(urdf_path: Path) -> None:
    """Removes internal geometries from all meshes in a URDF.

    Args:
        urdf_path: The path to the URDF file.
    """
    # Iterate through each link in the URDF and process the meshes
    for _, (_, visual_mesh_path), (_, collision_mesh_path) in iter_meshes(urdf_path):
        for mesh_path in list({visual_mesh_path, collision_mesh_path}):
            if mesh_path is not None:
                remove_internal_geometry(mesh_path)


def main() -> None:
    parser = argparse.ArgumentParser(description="Remove internal geometries from meshes in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    remove_internal_geometries_from_urdf(args.urdf_path)


if __name__ == "__main__":
    # python -m onshape.passes.remove_internal_geometries
    main()
