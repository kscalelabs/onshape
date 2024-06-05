"""Function for getting the convex hull of an stl."""

import logging
from pathlib import Path

import numpy as np
import tqdm
from numpy.typing import NDArray
from scipy.spatial import ConvexHull

from kol.mesh import Mesh, load_file


def find_convex_hull_points(
    mesh: Mesh,
) -> NDArray:
    """Finds the convex hull points of the mesh.

    Args:
        mesh: The mesh to simplify.

    Returns:
        The convex hull points.
    """
    points = mesh.points

    # Find the convex hull points.
    hull = ConvexHull(points)
    convex_faces = hull.simplices

    # Shape is convex already
    if convex_faces.shape[0] == mesh.faces.shape[0]:
        return mesh

    mesh.faces = convex_faces
    return mesh


def scale_mesh(
    mesh: Mesh,
    scaling: float,
) -> Mesh:
    """Scales the mesh.

    Args:
        mesh: The mesh to simplify.
        scaling: The scaling factor for the mesh.

    Returns:
        The scaled mesh.
    """
    points = mesh.points
    mesh.points = points * scaling
    return mesh


def get_convex_hull(
    mesh: str | Path | Mesh,
    scaling: float,
) -> None:
    """Returns the convex hull of the mesh.

    Args:
        mesh: The mesh to simplify.
        scaling: The scaling factor for the mesh.

    Returns:
        The convex hull.
    """
    if scaling < 0:
        raise ValueError(f"Scaling {scaling} should be (>=0).")

    # Converts the mesh from a file path to a Mesh object, if it is a file path.
    if isinstance(mesh, Path | str):
        mesh = load_file(mesh)
    if not isinstance(mesh, Mesh):
        raise ValueError(f"Invalid mesh type: {type(mesh)}")

    convex_mesh = find_convex_hull_points(mesh)
    if scaling != 1.0:
        convex_mesh = scale_mesh(convex_mesh, scaling)

    # Print some info about the simplification
    logging.info(f"Original mesh faces: {mesh.faces.shape[0]}")
    logging.info(f"Convex hull faces: {convex_mesh.faces.shape[0]}")

    return convex_mesh
