"""Defines geometric utility functions."""

import math
from dataclasses import dataclass

import numpy as np
import stl.mesh
from scipy.spatial import ConvexHull

from kol.mesh import Mesh


def rotation_matrix_to_euler_angles(rotation_matrix: np.matrix) -> tuple[float, float, float]:
    sy = math.sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])

    singular = sy < 1e-6

    if singular:
        x = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        y = math.atan2(-rotation_matrix[2, 0], sy)
        z = 0.0

    else:
        x = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        y = math.atan2(-rotation_matrix[2, 0], sy)
        z = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

    return x, y, z


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


def transform_inertia_tensor(inertia: list[float] | np.matrix, rotation: np.ndarray) -> np.ndarray:
    """Transforms the inertia tensor to a new frame.

    Args:
        inertia: The inertia tensor in the original frame.
        rotation: The rotation matrix from the original frame to the new frame.

    Returns:
        The inertia tensor in the new frame.
    """
    inertia_matrix = np.array(inertia).reshape(3, 3)
    return rotation.T @ inertia_matrix @ rotation


@dataclass
class Dynamics:
    mass: float
    com: np.ndarray
    inertia: np.matrix


def combine_dynamics(dynamics: list[Dynamics]) -> Dynamics:
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


def get_mesh_convex_hull(mesh: Mesh) -> Mesh:
    hull = ConvexHull(mesh.points)
    return Mesh(points=mesh.points[hull.vertices], faces=hull.simplices)


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


def combine_meshes(meshes: list[Mesh]) -> Mesh:
    points = np.concatenate([mesh.points for mesh in meshes])
    faces = np.concatenate([mesh.faces for mesh in meshes])
    return Mesh(points=points, faces=faces)
