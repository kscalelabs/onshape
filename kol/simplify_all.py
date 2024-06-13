"""Defines functions for simplifying all meshes in a urdf file."""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

import open3d as o3d

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


def simplify_mesh(filepath: str, voxel_size: float) -> str:
    """Simplifies a mesh by clustering vertices."""
    mesh = o3d.io.read_triangle_mesh(filepath)
    simple_mesh = mesh.simplify_vertex_clustering(
        voxel_size=voxel_size,
        contraction=o3d.geometry.SimplificationContraction.Average,
    )
    logger.info("Simplified mesh from %d to %d vertices", len(mesh.vertices), len(simple_mesh.vertices))
    # Remove old mesh and save the simplified one
    ext = Path(filepath).suffix
    if ext.lower() == ".stl":
        simple_mesh.compute_vertex_normals()
    filepath = Path(filepath)
    new_filepath = filepath.parent / f"{filepath.stem}_simple{filepath.suffix}"
    new_filepath = str(new_filepath)
    return new_filepath


def simplify_all(
    urdf_path: Path,
    voxel_size: float,
) -> None:
    if voxel_size <= 0:
        raise ValueError(f"Voxel size must be non-negative, got {voxel_size}")
    mesh_dir = urdf_path.parent
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
                    filepaths.add(mesh_dir / geometry.attrib["filename"])

    new_filepaths = {}
    for filepath in filepaths:
        logger.info("Simplifying mesh %s", filepath)
        new_filepath = simplify_mesh(str(filepath), voxel_size)
        new_filepaths[str(filepath)] = new_filepath

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

    # Save the updated URDF file
    new_urdf_path = urdf_path.with_name(urdf_path.stem + "_simplified" + urdf_path.suffix)
    tree.write(new_urdf_path)
    logger.info("Simplification complete. Updated URDF saved as %s", new_urdf_path)
