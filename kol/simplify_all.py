"""Defines functions for simplifying all meshes in a urdf file."""

import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Self

from kol.formats.common import save_xml

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


@dataclass
class Stats:
    total_removed: int
    total_starting_vertices: int
    total_ending_vertices: int

    @classmethod
    def start(cls) -> Self:
        return cls(0, 0, 0)


def simplify_all(urdf_path: Path, voxel_size: float, stats: Stats | None = None) -> Stats:
    try:
        import open3d as o3d
    except ImportError:
        logger.error(
            "Open3D is required to run this script. Install it with `pip install "
            "'kscale-onshape-library[open3d]'` to install the required dependencies."
        )
        raise

    def simplify_mesh(filepath: str, voxel_size: float) -> tuple[str, o3d.geometry.TriangleMesh, int, int]:
        """Simplifies a single mesh by clustering vertices."""
        mesh = o3d.io.read_triangle_mesh(filepath)
        starting_vertices = len(mesh.vertices)
        simple_mesh = mesh.simplify_vertex_clustering(
            voxel_size=voxel_size,
            contraction=o3d.geometry.SimplificationContraction.Average,
        )
        ending_vertices = len(simple_mesh.vertices)
        logger.info("Simplified mesh from %d to %d vertices", starting_vertices, ending_vertices)

        # Remove old mesh and save the simplified one
        ext = Path(filepath).suffix
        if ext.lower() == ".stl":
            simple_mesh.compute_vertex_normals()
        filepath = Path(filepath)
        new_filepath = filepath.parent / f"{filepath.stem}_simple{filepath.suffix}"
        new_filepath = str(new_filepath)

        return new_filepath, simple_mesh, starting_vertices, ending_vertices

    if stats is None:
        stats = Stats.start()
    if voxel_size <= 0:
        raise ValueError(f"Voxel size must be non-negative, got {voxel_size}")
    mesh_dir = urdf_path.parent / "meshes"

    # Load the URDF file
    logger.info("Getting element tree from URDF file.")
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    # Collect all links in urdf and get their meshes
    links = root.findall(".//link")
    logger.info("Found %d links in URDF", len(links))

    # Collect unique filepaths from visual and collision meshes
    filepaths = set()
    for link in links:
        for tag in ["visual", "collision"]:
            elements = link.findall(f"{tag}/geometry/mesh")
            for element in elements:
                if "filename" in element.attrib:
                    name = element.attrib["filename"]
                    if name.startswith("./meshes/"):
                        name = name.replace("./meshes/", "")
                    filepaths.add(mesh_dir / name)

    new_filepaths = {}
    new_meshes = {}
    for filepath in filepaths:
        logger.info("Simplifying mesh %s", filepath)
        new_filepath, new_mesh, starting_vertices, ending_vertices = simplify_mesh(str(filepath), voxel_size)
        new_filepaths[str(filepath)] = new_filepath
        new_meshes[new_filepath] = new_mesh
        stats.total_starting_vertices += starting_vertices
        stats.total_ending_vertices += ending_vertices
        stats.total_removed += starting_vertices - ending_vertices

    # Update the URDF file with new file paths
    logger.info("Updating URDF with new mesh file paths")
    for link in links:
        for tag in ["visual", "collision"]:
            elements = link.findall(f"{tag}/geometry/mesh")
            for element in elements:
                if "filename" in element.attrib:
                    old_filepath = str(mesh_dir / element.attrib["filename"].replace("./meshes/", ""))
                    new_filepath = new_filepaths.get(old_filepath, "")
                    if new_filepath:
                        relative_path = Path(new_filepath).relative_to(mesh_dir)
                        element.attrib["filename"] = f"./meshes/{relative_path}"
                    else:
                        logger.warning("No new filepath found for %s tag in link %s", tag, link.attrib["name"])

    # Save the updated URDF file
    new_urdf_path = urdf_path.with_name(urdf_path.stem + "_simplified" + urdf_path.suffix)
    save_xml(new_urdf_path, tree)
    logger.info("Simplification complete. Updated URDF saved as %s", new_urdf_path)

    # Save all the new meshes
    for new_filepath, new_mesh in new_meshes.items():
        match Path(new_filepath).suffix.lower():
            case ".ply":
                o3d.io.write_triangle_mesh(new_filepath, new_mesh, write_ascii=True)
            case _:
                o3d.io.write_triangle_mesh(new_filepath, new_mesh)

    # Log the total vertex reduction and percentage
    logger.info("Total starting vertices: %d", stats.total_starting_vertices)
    logger.info("Total ending vertices: %d", stats.total_ending_vertices)
    logger.info("Total vertices removed: %d", stats.total_removed)
    if stats.total_starting_vertices > 0:
        reduction_percentage = (stats.total_removed / stats.total_starting_vertices) * 100
        logger.info("Percentage reduction: %.4f%%", reduction_percentage)

    return stats
