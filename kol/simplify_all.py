"""Defines functions for simplifying all meshes in a urdf file."""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

import open3d as o3d

from kol.formats.common import save_xml

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

total_removed = 0
total_starting_vertices = 0
total_ending_vertices = 0


def simplify_mesh(filepath: str, voxel_size: float) -> tuple[str, str, int, int]:
    """Simplifies a single mesh by clustering vertices."""
    global total_starting_vertices, total_ending_vertices
    mesh = o3d.io.read_triangle_mesh(filepath)
    starting_vertices = len(mesh.vertices)
    simple_mesh = mesh.simplify_vertex_clustering(
        voxel_size=voxel_size,
        contraction=o3d.geometry.SimplificationContraction.Average,
    )
    ending_vertices = len(simple_mesh.vertices)
    total_starting_vertices += starting_vertices
    total_ending_vertices += ending_vertices

    logger.info("Simplified mesh from %d to %d vertices", starting_vertices, ending_vertices)

    # Remove old mesh and save the simplified one
    ext = Path(filepath).suffix
    if ext.lower() == ".stl":
        simple_mesh.compute_vertex_normals()
    filepath = Path(filepath)
    new_filepath = filepath.parent / f"{filepath.stem}_simple{filepath.suffix}"
    new_filepath = str(new_filepath)

    return new_filepath, simple_mesh, starting_vertices, ending_vertices


def simplify_all(
    urdf_path: Path,
    voxel_size: float,
) -> None:
    global total_removed, total_starting_vertices, total_ending_vertices  # noqa: PLW0602

    if voxel_size <= 0:
        raise ValueError(f"Voxel size must be non-negative, got {voxel_size}")
    mesh_dir = urdf_path.parent
    if "/meshes" not in str(mesh_dir):
        mesh_dir = Path(str(urdf_path.parent) + "/meshes")
    # Load the URDF file
    logger.info("Getting element tree from mesh filepath.")
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    # Collect all links in urdf and get their meshes
    links = root.findall(".//link")
    logger.info("Found %d links in URDF", len(links))

    # Collect unique filepaths from visual and collision meshes
    filepaths = set()
    for link in links:
        for tag in ["visual", "collision"]:
            element = link.find(tag)
            if element is not None:
                geometry = element.find("geometry/mesh")
                if geometry is not None and "filename" in geometry.attrib:
                    # if ./meshes/ in filename, remove it
                    name = geometry.attrib["filename"]
                    if "./meshes/" in name:
                        name = name.replace("./meshes/", "")
                    filepaths.add(mesh_dir / name)
                else:
                    logger.warning("No geometry found for %s tag in link %s", tag, link.attrib["name"])

    new_filepaths = {}
    new_meshes = {}
    for filepath in filepaths:
        logger.info("Simplifying mesh %s", filepath)
        new_filepath, new_mesh, starting_vertices, ending_vertices = simplify_mesh(str(filepath), voxel_size)
        new_filepaths[str(filepath)] = new_filepath
        new_meshes[new_filepath] = new_mesh
        total_removed += starting_vertices - ending_vertices

    # Update the URDF file with new file paths
    logger.info("Updating URDF with new mesh file paths")
    for link in links:
        for tag in ["visual", "collision"]:
            element = link.find(tag)
            if element is not None:
                geometry = element.find("geometry/mesh")
                if geometry is not None and "filename" in geometry.attrib:
                    old_filepath = geometry.attrib["filename"]
                    new_filepath = new_filepaths.get(str(mesh_dir / old_filepath), "")
                    if new_filepath:
                        geometry.attrib["filename"] = str(Path(new_filepath).relative_to(mesh_dir))
                    else:
                        logger.warning("No new filepath found for %s tag in link %s", tag, link.attrib["name"])
                else:
                    logger.warning("No geometry found for %s tag in link %s", tag, link.attrib["name"])
            else:
                logger.warning("No geometry found for %s tag in link %s", tag, link.attrib["name"])

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
    logger.info("Total starting vertices: %d", total_starting_vertices)
    logger.info("Total ending vertices: %d", total_ending_vertices)
    logger.info("Total vertices removed: %d", total_removed)
    reduction_percentage = (total_removed / total_starting_vertices) * 100 if total_starting_vertices > 0 else 0
    logger.info("Percentage reduction: %.4f%%", reduction_percentage)
