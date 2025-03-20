"""Defines geometric utility functions."""

from dataclasses import dataclass

import numpy as np
import stl.mesh
from scipy.spatial import ConvexHull

from onshape.utils.mesh import Mesh


def apply_matrix_(mesh: stl.mesh.Mesh, matrix: np.ndarray) -> stl.mesh.Mesh:
    rotation = matrix[0:3, 0:3]
    translation = matrix[0:3, 3:4].T.tolist()

    def transform(points: np.ndarray) -> np.ndarray:
        return (rotation * np.matrix(points).T).T + translation * len(points)

    mesh.v0 = transform(mesh.v0)
    mesh.v1 = transform(mesh.v1)
    mesh.v2 = transform(mesh.v2)
    mesh.normals = transform(mesh.normals)
    return mesh


def inv_tf(a_to_b_tf: np.matrix) -> np.matrix:
    return np.matrix(np.linalg.inv(a_to_b_tf))


def transform_inertia_tensor(inertia: np.matrix, rotation: np.matrix) -> np.matrix:
    """Transforms the inertia tensor to a new frame.

    Args:
        inertia: The inertia tensor in the original frame, a (3, 3) matrix.
        rotation: The rotation matrix from the original frame to the new frame.

    Returns:
        The inertia tensor in the new frame.
    """
    return rotation @ inertia @ rotation.T


@dataclass
class Dynamics:
    mass: float
    com: np.ndarray
    inertia: np.matrix


def combine_dynamics(*dynamics: Dynamics) -> Dynamics:
    mass: float = 0.0
    com = np.array([0.0] * 3)
    inertia = np.matrix(np.zeros((3, 3)))
    identity = np.matrix(np.eye(3))

    for dynamic in dynamics:
        mass += dynamic.mass
        com += np.array(dynamic.com) * dynamic.mass

    if mass > 0:
        com /= mass

    for dynamic in dynamics:
        r = dynamic.com - com
        p = np.matrix(r)
        inertia = inertia + dynamic.inertia + (np.dot(r, r) * identity - p.T * p) * dynamic.mass

    return Dynamics(mass, com, inertia)


def matrix_to_moments(matrix: np.matrix) -> dict[str, str]:
    return {
        "ixx": str(matrix[0, 0]),
        "ixy": str(matrix[0, 1]),
        "ixz": str(matrix[0, 2]),
        "iyy": str(matrix[1, 1]),
        "iyz": str(matrix[1, 2]),
        "izz": str(matrix[2, 2]),
    }


def moments_to_matrix(inertia_moments: np.ndarray) -> np.matrix:
    """Convert a 6-element array of inertia moments into a 3x3 inertia matrix.

    Args::
        inertia_moments: A 6-element array ``[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]``.

    Returns:
        A (3, 3) inertia matrix.
    """
    ixx, iyy, izz, ixy, ixz, iyz = (
        inertia_moments[0],
        inertia_moments[1],
        inertia_moments[2],
        inertia_moments[3],
        inertia_moments[4],
        inertia_moments[5],
    )

    inertia_matrix = np.array(
        [
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz],
        ]
    )

    return np.matrix(inertia_matrix)


def get_mesh_convex_hull(mesh: Mesh) -> Mesh:
    hull = ConvexHull(mesh.points)
    return Mesh(points=hull.points, faces=hull.simplices)


def get_center_of_mass(mesh: Mesh) -> tuple[float, float, float]:
    total_volume = 0.0
    center_of_mass = np.zeros(3)
    for triangle in mesh.faces:
        v0, v1, v2 = triangle
        p0, p1, p2 = mesh.points[v0], mesh.points[v1], mesh.points[v2]
        volume = np.dot(p0, np.cross(p1, p2)) / 6.0
        centroid = (p0 + p1 + p2) / 4.0
        total_volume += volume
        center_of_mass += volume * centroid
    center_of_mass /= total_volume
    p0, p1, p2 = center_of_mass
    return (p0, p1, p2)


def scale_mesh(mesh: Mesh, scale: float, about_origin: bool = False) -> Mesh:
    if scale <= 0:
        raise ValueError(f"Scaling {scale} should be greater than 0.")
    com = (0.0, 0.0, 0.0) if about_origin else get_center_of_mass(mesh)
    points = mesh.points - com
    points *= scale
    points += com
    return Mesh(points=points, faces=mesh.faces)


def apply_transform(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    """Apply a transformation matrix to a set of points.

    Args:
        points: A (n, 3) numpy array of points.
        transform: A (4, 4) transformation matrix.

    Returns:
        A (n, 3) numpy array of transformed points.
    """
    points_homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
    transformed_points_homogeneous = points_homogeneous @ transform.T
    transformed_points = transformed_points_homogeneous[:, :3]
    transformed_points = np.asarray(transformed_points)
    return transformed_points


def combine_meshes(parent_mesh: Mesh, child_mesh: Mesh, relative_transform: np.ndarray) -> Mesh:
    """Combine parent and child meshes, applying the relative transform to the child mesh points.

    Args:
        parent_mesh: The parent mesh.
        child_mesh: The child mesh.
        relative_transform: The transformation matrix to apply to the child mesh points.

    Returns:
        A new mesh combining the parent and child meshes.
    """
    if not isinstance(relative_transform, np.ndarray) or relative_transform.shape != (4, 4):
        raise ValueError("relative_transform must be a 4x4 numpy array")
    transformed_child_points = apply_transform(child_mesh.points, relative_transform)
    combined_points = np.concatenate([parent_mesh.points, transformed_child_points])
    offset_child_faces = child_mesh.faces + len(parent_mesh.points)
    combined_faces = np.concatenate([parent_mesh.faces, offset_child_faces])
    return Mesh(points=combined_points, faces=combined_faces)
