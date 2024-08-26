"""Defines a pass to convert the collision mesh into a convex hull."""

import argparse
import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from kol.formats.common import save_xml
from kol.utils.geometry import get_mesh_convex_hull
from kol.utils.mesh import load_file, save_file

logger = logging.getLogger(__name__)


def make_convex_collision_mesh(mesh_path: Path, output_mesh_path: Path) -> tuple[int, int]:
    mesh = load_file(mesh_path)
    num_vertices_pre = len(mesh.points)
    hull_mesh = get_mesh_convex_hull(mesh)
    save_file(hull_mesh, output_mesh_path)
    num_vertices_post = len(hull_mesh.points)
    return num_vertices_pre, num_vertices_post


def get_convex_collision_meshes(urdf_path: Path) -> None:
    """Converts the collision mesh into convex hulls.

    Args:
        urdf_path: The path to the urdf file.

    Returns:
        The path to the merged urdf file.
    """
    urdf_tree = ET.parse(urdf_path)

    def get_mesh(visual_or_collision: ET.Element) -> tuple[ET.Element, Path] | None:
        if (geometry := visual_or_collision.find("geometry")) is None:
            return None
        if (mesh := geometry.find("mesh")) is None:
            return None
        return mesh, (urdf_path.parent / mesh.attrib["filename"]).resolve()

    # Iterates through each link in the URDF and simplifies the meshes.
    total_pre_num_vertices = 0
    total_post_num_vertices = 0
    has_change = False
    for link in urdf_tree.iter("link"):
        if (visual_link := link.find("visual")) is None or (collision_link := link.find("collision")) is None:
            continue
        if (visual_mesh := get_mesh(visual_link)) is None or (collision_mesh := get_mesh(collision_link)) is None:
            continue
        _, visual_mesh_path = visual_mesh
        collision_mesh_elem, collision_mesh_path = collision_mesh

        # If the visual mesh and the collision mesh are the same, we need to
        # make a new mesh for the collision mesh. Otherwise, we can just
        # overwrite the existing collision mesh.
        if visual_mesh_path == collision_mesh_path:
            collision_output_mesh_path = collision_mesh_path.with_suffix(".convex.stl")
            collision_mesh_elem.attrib["filename"] = collision_output_mesh_path.relative_to(urdf_path.parent).as_posix()
            has_change = True
        else:
            collision_output_mesh_path = collision_mesh_path

        pre_vertices, post_vertices = make_convex_collision_mesh(collision_mesh_path, collision_output_mesh_path)
        total_pre_num_vertices += pre_vertices
        total_post_num_vertices += post_vertices

    if total_pre_num_vertices > 0:
        percent_reduction = (1 - total_post_num_vertices / total_pre_num_vertices) * 100
        logger.info(
            "Convex hull reduces meshes from %d to %d vertices (%.2f%% reduction)",
            total_pre_num_vertices,
            total_post_num_vertices,
            percent_reduction,
        )

    if has_change:
        save_xml(urdf_path, urdf_tree)


def main() -> None:
    parser = argparse.ArgumentParser(description="Merge fixed joints in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    get_convex_collision_meshes(args.urdf_path)


if __name__ == "__main__":
    # python -m kol.passes.simplify_meshes
    main()
