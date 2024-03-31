"""Defines geometric utility functions."""

import math
from dataclasses import dataclass

import numpy as np
import stl.mesh


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


def apply_matrix_(mesh: stl.mesh.Mesh, matrix: np.matrix) -> stl.mesh.Mesh:
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
        com += dynamic.com * dynamic.mass

    if mass > 0:
        com /= mass

    for dynamic in dynamics:
        r = dynamic.com - com
        p = np.matrix(r)
        inertia = inertia + dynamic.inertia + (np.dot(r, r) * identity - p.T * p) * dynamic.mass

    return Dynamics(mass, com, inertia)
