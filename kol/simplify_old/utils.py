"""Defines mesh simplification utility functions."""

import numpy as np
from numpy.typing import NDArray


def get_edges(faces: NDArray) -> NDArray:
    """Gets the edges of the mesh.

    This function gets the edges of the mesh from the faces. This just means
    converting each face into it's three edges, then removing duplicates.

    Args:
        faces: The faces of the mesh, with shape ``(M, 3)``.

    Returns:
        The edges of the mesh, with shape ``(N, 2)``.
    """
    edge_1 = faces[:, 0:2]
    edge_2 = faces[:, 1:]
    edge_3 = np.concatenate([faces[:, :1], faces[:, -1:]], axis=1)
    edges = np.concatenate([edge_1, edge_2, edge_3], axis=0)
    _, unique_edges_locs = np.unique(edges[:, 0] * (10**10) + edges[:, 1], return_index=True)
    edges = edges[unique_edges_locs, :]
    return edges


def plane_eq_para(points: NDArray, faces: NDArray) -> NDArray:
    """Calculates the plane equations for each face.

    This function calculates the plane equations for each face in the mesh.
    The plane equation is represented as ax + by + cz + d = 0,
    where a^2 + b^2 + c^2 = 1.

    Args:
        points: The points of the mesh, with shape ``(N, 3)``.
        faces: The faces of the mesh, with shape ``(M, 3)``.

    Returns:
        The plane equations for each face, with shape ``(M, 4)``.
    """
    plane_equ_para: list[NDArray] = []
    for i in range(0, faces.shape[0]):
        point_1 = points[faces[i, 0] - 1, :]
        point_2 = points[faces[i, 1] - 1, :]
        point_3 = points[faces[i, 2] - 1, :]
        point_mat = np.array([point_1, point_2, point_3])
        abc = np.matmul(np.linalg.inv(point_mat), np.array([[1], [1], [1]]))
        plane_equ_para.append(np.concatenate([abc.T, np.array(-1).reshape(1, 1)], axis=1) / (np.sum(abc**2) ** 0.5))
    plane_equ_para_arr = np.array(plane_equ_para)
    plane_equ_para_arr = plane_equ_para_arr.reshape(plane_equ_para_arr.shape[0], plane_equ_para_arr.shape[2])
    return plane_equ_para_arr


def q_matrices(points: NDArray, faces: NDArray, plane_eq_para: NDArray) -> list[NDArray]:
    """Calculates the Q matrices for each point.

    This function calculates the Q matrices for each point in the mesh.
    The Q matrix is the sum of the outer product of the plane equation
    parameters for each face that contains the point.

    Args:
        points: The points of the mesh.
        faces: The faces of the mesh.
        plane_eq_para: The plane equations for each face.

    Returns:
        The Q matrices for each point, with shape ``(N, 4, 4)``.
    """
    q_matrices: list[NDArray] = []
    for i in range(0, points.shape[0]):
        point_index = i + 1
        face_set_index = np.where(faces == point_index)[0]
        q_temp = np.zeros((4, 4))
        for j in face_set_index:
            p = plane_eq_para[j, :]
            p = p.reshape(1, len(p))
            q_temp = q_temp + np.matmul(p.T, p)
        q_matrices.append(q_temp)
    breakpoint()
    return q_matrices


def get_nearby_point_pairs(points: NDArray, threshold: float) -> NDArray:
    """Gets pairs of points which are less than some threshold apart.

    Args:
        points: The points of the mesh, with shape ``(N, 3)``.
        threshold: The threshold distance.

    Returns:
        The unique pairs of points, with shape ``(M, 2)``.
    """
    dist_pairs: list[NDArray] = []
    for i in range(0, points.shape[0]):
        current_point_location = i + 1
        current_point = points[i, :]
        current_point_to_others_dist = (np.sum((points - current_point) ** 2, axis=1)) ** 0.5
        valid_pairs_location = np.where(current_point_to_others_dist <= threshold)[0] + 1
        valid_pairs_location = valid_pairs_location.reshape(len(valid_pairs_location), 1)
        current_valid_pairs = np.concatenate(
            [
                current_point_location * np.ones((valid_pairs_location.shape[0], 1)),
                valid_pairs_location,
            ],
            axis=1,
        )
        dist_pairs.append(current_valid_pairs)

    # Gets rid of pairs which are the same point.
    dist_pairs_arr = np.concatenate(dist_pairs, axis=0)  # (N, 2)
    find_same = dist_pairs_arr[:, 1] - dist_pairs_arr[:, 0]
    find_same_loc = np.where(find_same == 0)[0]
    dist_pairs_arr = np.delete(dist_pairs_arr, find_same_loc, axis=0)

    return dist_pairs_arr
