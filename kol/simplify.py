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
    converting each face into its three edges, then removing duplicates.

    Args:
        faces: The faces of the mesh, with shape ``(M, 3)``.

    Returns:
        The edges of the mesh, with shape ``(N, 2)``.
    """
    edge_1 = faces[:, 0:2]
    edge_2 = faces[:, 1:]
    edge_3 = np.concatenate([faces[:, :1], faces[:, 2:3]], axis=1)
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
    valid_pairs: NDArray,
    q_matrices: NDArray,
) -> tuple[NDArray, NDArray, NDArray]:
    """Calculates the optimal contraction pairs and cost.

    Compute the optimal contraction target v_opt for each valid pair
    `(v1, v2)`. The error `v_opt.T * (Q1 + Q2) * v_pot` of this target
    vertex becomes the cost of contracting that pair. Place all the pairs
    in a heap keyed on cost with the minimum cost pair at the top.

    Args:
        points: The points of the mesh, with shape ``(N, 3)``.
        valid_pairs: The valid pairs of points to contract, with shape
            ``(M, 2)``.
        q_matrices: The Q matrices for each point, with shape ``(N, 4, 4)``.

    Returns:
        The optimal contraction pairs, with shape ``(M, 3)``, and the cost, with
        shape ``(M,)``.
    """
    v_optimal: list[NDArray] = []
    cost: list[float] = []
    number_of_valid_pairs = valid_pairs.shape[0]
    for i in tqdm.trange(0, number_of_valid_pairs):
        current_valid_pair = valid_pairs[i, :]
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

    cost = np.array(cost)
    cost = cost.reshape(cost.shape[0])
    cost_argsort = np.argsort(cost)
    valid_pairs = valid_pairs[cost_argsort, :]
    v_optimal = np.array(v_optimal)
    v_optimal = v_optimal[cost_argsort, :]

    return np.array(v_optimal), np.array(cost), valid_pairs


def calculate_plane_equation_for_one_face(p1: NDArray, p2: NDArray, p3: NDArray) -> NDArray:
    # input: p1, p2, p3 numpy.array, shape: (3, 1) or (1,3) or (3, )
    # p1 ,p2, p3 (x, y, z) are three points on a face
    # plane equ: ax+by+cz+d=0 a^2+b^2+c^2=1
    # return: numpy.array (a, b, c, d), shape: (1,4)
    p1 = np.array(p1).reshape(3)
    p2 = np.array(p2).reshape(3)
    p3 = np.array(p3).reshape(3)
    point_mat = np.array([p1, p2, p3])
    if np.linalg.matrix_rank(point_mat) < 3:  # Check for singularity
        return np.array([0, 0, 0, 0]).reshape(1, 4)  # This is a degenerate face
    else:
        abc = np.matmul(np.linalg.inv(point_mat), np.array([[1], [1], [1]]))
        output = np.concatenate([abc.T, np.array(-1).reshape(1, 1)], axis=1) / (np.sum(abc**2) ** 0.5)
        output = output.reshape(4)
        return output


def iteratively_remove_least_cost_valid_pairs(
    points: NDArray,
    faces: NDArray,
    valid_pairs: NDArray,
    new_valid_pair: NDArray,
    v_optimal: NDArray,
    new_point: NDArray,
    cost: NDArray,
    plane_equ_para: NDArray,
    q_matrices: NDArray,
    simplify_ratio: float,
) -> tuple[NDArray, NDArray, NDArray, NDArray]:
    new_point_count = 0
    status_points = np.zeros(len(points))
    status_faces = np.zeros(len(faces))
    while (points.shape[0] - new_point_count) >= simplify_ratio * (points.shape[0]):
        # current valid pair
        current_valid_pair = new_valid_pair
        v_1_location = current_valid_pair[0] - 1  # point location in self.points
        v_2_location = current_valid_pair[1] - 1

        # update self.points
        # put the top optimal vertex(point) into the sequence of points
        points[v_1_location, :] = new_point.reshape(1, 3)
        points[v_2_location, :] = new_point.reshape(1, 3)

        # set status of points
        # 0 means no change, -1 means the point is deleted
        # update v1, v2 to v_opt, then delete v2, keep v1
        status_points[v_2_location] = -1

        # set status of faces
        # 0 means no change, -1 means the face will be deleted
        v_1_in_faces_loc = np.where(faces == (v_1_location + 1))
        v_2_in_faces_loc = np.where(faces == (v_2_location + 1))
        v_1_2_in_one_face_loc = []
        for item in v_2_in_faces_loc[0]:
            if np.where(v_1_in_faces_loc[0] == item)[0].size > 0:
                v_1_2_in_one_face_loc.append(item)
        v_1_2_in_one_face_loc = np.array(v_1_2_in_one_face_loc)
        if v_1_2_in_one_face_loc.size >= 1:
            status_faces[v_1_2_in_one_face_loc] = -1

        # update faces
        # points of faces involving v1 and v2 are changed accordingly
        # set v2 to v1
        faces[v_2_in_faces_loc] = v_1_location + 1

        # update plane_equ_para
        v_1_2_in_faces_loc = np.unique(np.append(v_1_in_faces_loc[0], v_2_in_faces_loc[0]))
        for i in v_1_2_in_faces_loc:
            if status_faces[i] == -1:
                plane_equ_para[i, :] = np.array([0, 0, 0, 0]).reshape(1, 4)
            else:
                point_1 = points[faces[i, 0] - 1, :]
                point_2 = points[faces[i, 1] - 1, :]
                point_3 = points[faces[i, 2] - 1, :]
                # calculate plane equation given 3 points
                plane_equ_para[i, :] = calculate_plane_equation_for_one_face(point_1, point_2, point_3)

        # update q matrices
        face_set_index = np.where(faces == v_1_location + 1)[0]
        q_temp = np.zeros((4, 4))

        for j in face_set_index:
            p = plane_equ_para[j, :]
            p = p.reshape(1, len(p))
            q_temp = q_temp + np.matmul(p.T, p)

        for i in current_valid_pair:
            q_matrices[i - 1] = q_temp

        # update valid_pairs, v_optimal, and cost
        # processing valid_pairs
        # replace all the point indexes containing current valid pair with new point index: target_loc+1
        v_1_loc_in_valid_pairs = np.where(valid_pairs == new_valid_pair[0])
        v_2_loc_in_valid_pairs = np.where(valid_pairs == new_valid_pair[1])

        valid_pairs[v_1_loc_in_valid_pairs] = v_1_location + 1
        valid_pairs[v_2_loc_in_valid_pairs] = v_1_location + 1

        delete_locs = []
        for item in v_1_loc_in_valid_pairs[0]:
            if np.where(v_2_loc_in_valid_pairs[0] == item)[0].size > 0:
                delete_locs.append(item)
        delete_locs = np.array(delete_locs)

        find_same = valid_pairs[:, 1] - valid_pairs[:, 0]
        find_same_loc = np.where(find_same == 0)[0]
        if find_same_loc.size >= 1:
            delete_locs = np.append(delete_locs, find_same_loc)

        # delete process for self.valid_pairs, self.v_optimal and self.cost
        valid_pairs = np.delete(valid_pairs, delete_locs, axis=0)
        v_optimal = np.delete(v_optimal, delete_locs, axis=0)
        cost = np.delete(cost, delete_locs, axis=0)

        # unique process for self.valid_pairs, self.v_optimal and self.cost
        unique_valid_pairs_trans, unique_valid_pairs_loc = np.unique(
            valid_pairs[:, 0] * (10**10) + valid_pairs[:, 1],
            return_index=True,
        )
        valid_pairs = valid_pairs[unique_valid_pairs_loc, :]
        v_optimal = v_optimal[unique_valid_pairs_loc, :]
        cost = cost[unique_valid_pairs_loc]

        # re-calculate optimal contraction pairs and cost
        v_target_loc_in_valid_pairs = np.where(valid_pairs == v_1_location + 1)[0]
        for i in v_target_loc_in_valid_pairs:
            current_valid_pair = valid_pairs[i, :]
            v_1_location = current_valid_pair[0] - 1
            v_2_location = current_valid_pair[1] - 1
            # find Q_1
            q_1 = q_matrices[v_1_location]
            # find Q_2
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
            v_optimal[i, :] = current_v_opt
            cost[i] = current_cost

        cost_argsort = np.argsort(cost)
        valid_pairs = valid_pairs[cost_argsort, :]
        v_optimal = v_optimal[cost_argsort, :]
        cost = cost[cost_argsort]

        new_point = v_optimal[0, :]
        new_valid_pair = valid_pairs[0, :]

        if new_point_count % 100 == 0:
            print("Simplification: " + str(100 * (points.shape[0] - new_point_count) / (points.shape[0])) + "%")
            print("Remaining: " + str(points.shape[0] - new_point_count) + " points")
            print("\n")

        new_point_count = new_point_count + 1

    print(
        "Simplification: " + str(100 * (points.shape[0] - new_point_count) / (points.shape[0] + new_point_count)) + "%"
    )
    print("Remaining: " + str(points.shape[0] - new_point_count) + " points")
    print("End\n")

    return points, status_points, faces, status_faces


# Generate the simplified 3d model (points/vertices, faces)
def generate_new_3d_model(
    points: NDArray,
    status_points: NDArray,
    faces: NDArray,
    status_faces: NDArray,
) -> tuple[NDArray, NDArray]:
    point_serial_number = np.arange(points.shape[0]) + 1
    points_to_delete_locs = np.where(status_points == -1)[0]
    points = np.delete(points, points_to_delete_locs, axis=0)
    point_serial_number = np.delete(point_serial_number, points_to_delete_locs)
    point_serial_number_after_del = np.arange(points.shape[0]) + 1

    faces_to_delete_locs = np.where(status_faces == -1)[0]
    faces = np.delete(faces, faces_to_delete_locs, axis=0)

    for i in point_serial_number_after_del:
        point_loc_in_face = np.where(faces == point_serial_number[i - 1])
        faces[point_loc_in_face] = i

    return points, faces


def get_simplified_mesh(
    mesh: str | Path | Mesh,
    threshold: float,
    simplify_ratio: float,
) -> None:
    """Simplifies the mesh.

    Args:
        mesh: The mesh to simplify
        output_filepath: The path to save the simplified mesh.
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
    v_optimal, cost, valid_pairs = calculate_optimal_contraction_pairs_and_cost(mesh.points, point_pairs, q_mats)

    logger.info("Iteratively removing least cost valid pairs")
    points, status_points, faces, status_faces = iteratively_remove_least_cost_valid_pairs(
        mesh.points,
        mesh.faces,
        valid_pairs,
        valid_pairs[0],
        v_optimal,
        v_optimal[0, :],
        cost,
        plane_eq_para,
        q_mats,
        simplify_ratio,
    )

    logger.info("Generating new 3D model")
    mesh.points, mesh.faces = generate_new_3d_model(points, status_points, faces, status_faces)
    return mesh
