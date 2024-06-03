"""Defines mesh simplification utility functions."""

import logging
from pathlib import Path

import numpy as np
import tqdm
from numpy.typing import NDArray

from kol.mesh import Mesh, load_file

logger = logging.getLogger(__name__)


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


def get_plane_eq_para(points: NDArray, faces: NDArray) -> NDArray:
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
        if np.linalg.det(point_mat) == 0:
            plane_equ_para.append(np.array([[0, 0, 0, 0]]))  # This is a degenerate face.
        else:
            abc = np.matmul(np.linalg.inv(point_mat), np.array([[1], [1], [1]]))
            plane_equ_para.append(np.concatenate([abc.T, np.array(-1).reshape(1, 1)], axis=1) / (np.sum(abc**2) ** 0.5))
    plane_equ_para_arr = np.array(plane_equ_para)
    plane_equ_para_arr = plane_equ_para_arr.reshape(plane_equ_para_arr.shape[0], plane_equ_para_arr.shape[2])
    return plane_equ_para_arr


def get_q_matrices(points: NDArray, faces: NDArray, plane_eq_para: NDArray) -> NDArray:
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
    return np.array(q_matrices)


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
                current_point_location * np.ones((valid_pairs_location.shape[0], 1), dtype=int),
                valid_pairs_location,
            ],
            axis=1,
            dtype=int,
        )
        dist_pairs.append(current_valid_pairs)

    # Gets rid of pairs which are the same point.
    dist_pairs_arr = np.concatenate(dist_pairs, axis=0)  # (N, 2)
    find_same = dist_pairs_arr[:, 1] - dist_pairs_arr[:, 0]
    find_same_loc = np.where(find_same == 0)[0]
    dist_pairs_arr = np.delete(dist_pairs_arr, find_same_loc, axis=0)

    return dist_pairs_arr


def calculate_optimal_contraction_pairs_and_cost(
    points: NDArray,
    candidate_pairs: NDArray,
    q_matrices: NDArray,
) -> tuple[NDArray, NDArray]:
    """Calculates the optimal contraction pairs and cost.

    Compute the optimal contraction target v_opt for each valid pair
    `(v1, v2)`. The error `v_opt.T * (Q1 + Q2) * v_pot` of this target
    vertex becomes the cost of contracting that pair. Place all the pairs
    in a heap keyed on cost with the minimum cost pair at the top.

    Args:
        points: The points of the mesh, with shape ``(N, 3)``.
        candidate_pairs: The candidate pairs of points to contract, with shape
            ``(M, 2)``.
        q_matrices: The Q matrices for each point, with shape ``(N, 4, 4)``.

    Returns:
        The optimal contraction pairs, with shape ``(M, 3)``, and the cost, with
        shape ``(M,)``.
    """
    v_optimal: list[NDArray] = []
    cost: list[float] = []
    number_of_valid_pairs = candidate_pairs.shape[0]
    for i in tqdm.trange(0, number_of_valid_pairs):
        current_valid_pair = candidate_pairs[i, :]
        v_1_location = current_valid_pair[0] - 1
        v_2_location = current_valid_pair[1] - 1
        q_1 = q_matrices[v_1_location]
        q_2 = q_matrices[v_2_location]
        q = q_1 + q_2
        q_new = np.concatenate([q[:3, :], np.array([0, 0, 0, 1]).reshape(1, 4)], axis=0)
        if np.linalg.det(q_new) > 0:
            current_v_opt = np.matmul(np.linalg.inv(q_new), np.array([0, 0, 0, 1]).reshape(4, 1))
            current_cost = np.matmul(np.matmul(current_v_opt.T, q), current_v_opt)
            current_v_opt = current_v_opt.reshape(4)[:3]
        else:
            v_1 = np.append(points[v_1_location, :], 1).reshape(4, 1)
            v_2 = np.append(points[v_2_location, :], 1).reshape(4, 1)
            v_mid = (v_1 + v_2) / 2
            delta_v_1 = np.matmul(np.matmul(v_1.T, q), v_1)
            delta_v_2 = np.matmul(np.matmul(v_2.T, q), v_2)
            delta_v_mid = np.matmul(np.matmul(v_mid.T, q), v_mid)
            current_cost = np.min(np.array([delta_v_1, delta_v_2, delta_v_mid]))
            min_delta_loc = np.argmin(np.array([delta_v_1, delta_v_2, delta_v_mid]))
            current_v_opt = np.concatenate([v_1, v_2, v_mid], axis=1)[:, min_delta_loc].reshape(4)
            current_v_opt = current_v_opt[:3]
        v_optimal.append(current_v_opt)
        cost.append(current_cost.item())

    return np.array(v_optimal), np.array(cost)


def get_simplified_mesh(mesh: str | Path | Mesh, threshold: float, simplify_ratio: float) -> Mesh:
    """Simplifies the mesh.

    Args:
        mesh: The mesh to simplify
        threshold: The threshold for the distance between vertices.
        simplify_ratio: The ratio of vertices to simplify the mesh to.

    Returns:
        The simplified mesh.
    """
    if simplify_ratio > 1 or simplify_ratio <= 0:
        raise ValueError(f"Simplification ratio {simplify_ratio} should be (0<r<=1).")
    if threshold < 0:
        raise ValueError(f"Threshold {threshold} should be (>=0).")

    # Converts the mesh from a file path to a Mesh object, if it is a file path.
    if isinstance(mesh, Path | str):
        mesh = load_file(mesh)
    if not isinstance(mesh, Mesh):
        raise ValueError(f"Invalid mesh type: {type(mesh)}")

    logger.info("Getting point pairs from %d points", len(mesh.points))
    point_pairs = get_nearby_point_pairs(mesh.points, threshold)
    logger.info("Found %d point pairs", len(point_pairs))

    logger.info("Getting Q matrices")
    plane_eq_para = get_plane_eq_para(mesh.points, mesh.faces)
    logger.info("Got %d plane equations", len(plane_eq_para))
    q_mats = get_q_matrices(mesh.points, mesh.faces, plane_eq_para)
    logger.info("Got %d Q matrices", len(q_mats))

    logger.info("Calculating optimal contraction pairs and cost between %d pairs", len(point_pairs))
    v_optimal, cost = calculate_optimal_contraction_pairs_and_cost(mesh.points, point_pairs, q_mats)

    breakpoint()

    return mesh
