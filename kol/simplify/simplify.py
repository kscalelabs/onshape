"""Defines a utility class for simplifying meshes."""

import functools
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from kol.mesh import Mesh, load_file


class MeshSimplify:
    def __init__(self, mesh: str | Path | Mesh, threshold: float, simplify_ratio: float) -> None:
        """Simplifies a mesh using the quadric error metric.

        This class simplifies a mesh using the quadric error metric. The mesh is
        simplified by iteratively removing the pair of vertices with the least
        error until the desired ratio of vertices is reached.

        Args:
            mesh: The mesh to simplify
            threshold: The threshold for the distance between vertices.
            simplify_ratio: The ratio of vertices to simplify the mesh to.
        """
        super().__init__()

        if simplify_ratio > 1 or simplify_ratio <= 0:
            raise ValueError(f"Simplification ratio {simplify_ratio} should be (0<r<=1).")
        if threshold < 0:
            raise ValueError(f"Threshold {threshold} should be (>=0).")

        self._original_mesh = mesh
        self.t = threshold
        self.ratio = simplify_ratio

    @functools.cached_property
    def original_mesh(self) -> Mesh:
        if isinstance(self._original_mesh, Mesh):
            return self._original_mesh
        if isinstance(self._original_mesh, Path | str):
            return load_file(self._original_mesh)
        raise ValueError(f"Invalid mesh type: {type(self._original_mesh)}")

    @property
    def original_number_of_points(self) -> int:
        return self.original_mesh.points.shape[0]

    def calculate_optimal_contraction_pairs_and_cost(self) -> None:
        """Calculates the optimal contraction pairs and cost.

        Compute the optimal contraction target v_opt for each valid pair
        `(v1, v2)`. The error `v_opt.T * (Q1 + Q2) * v_pot` of this target
        vertex becomes the cost of contracting that pair. Place all the pairs
        in a heap keyed on cost with the minimum cost pair at the top.

        Returns:
            The optimal contraction pairs and cost.
        """
        valid_pairs = self.valid_pairs

        v_optimal = []
        cost = []
        number_of_valid_pairs = self.valid_pairs.shape[0]
        for i in range(0, number_of_valid_pairs):
            current_valid_pair = self.valid_pairs[i, :]
            v_1_location = current_valid_pair[0] - 1
            v_2_location = current_valid_pair[1] - 1
            q_1 = self.q_matrices[v_1_location]
            q_2 = self.q_matrices[v_2_location]
            q = q_1 + q_2
            q_new = np.concatenate([q[:3, :], np.array([0, 0, 0, 1]).reshape(1, 4)], axis=0)
            if np.linalg.det(q_new) > 0:
                current_v_opt = np.matmul(np.linalg.inv(q_new), np.array([0, 0, 0, 1]).reshape(4, 1))
                current_cost = np.matmul(np.matmul(current_v_opt.T, q), current_v_opt)
                current_v_opt = current_v_opt.reshape(4)[:3]
            else:
                v_1 = np.append(self.original_points[v_1_location, :], 1).reshape(4, 1)
                v_2 = np.append(self.original_points[v_2_location, :], 1).reshape(4, 1)
                v_mid = (v_1 + v_2) / 2
                delta_v_1 = np.matmul(np.matmul(v_1.T, q), v_1)
                delta_v_2 = np.matmul(np.matmul(v_2.T, q), v_2)
                delta_v_mid = np.matmul(np.matmul(v_mid.T, q), v_mid)
                current_cost = np.min(np.array([delta_v_1, delta_v_2, delta_v_mid]))
                min_delta_loc = np.argmin(np.array([delta_v_1, delta_v_2, delta_v_mid]))
                current_v_opt = np.concatenate([v_1, v_2, v_mid], axis=1)[:, min_delta_loc].reshape(4)
                current_v_opt = current_v_opt[:3]
            v_optimal.append(current_v_opt)
            cost.append(current_cost)

        # self.v_optimal = np.array(self.v_optimal)
        # self.cost = np.array(self.cost)
        # self.cost = self.cost.reshape(self.cost.shape[0])

        # cost_argsort = np.argsort(self.cost)
        # self.valid_pairs = self.valid_pairs[cost_argsort, :]
        # self.v_optimal = self.v_optimal[cost_argsort, :]
        # self.cost = self.cost[cost_argsort]

        # self.new_point = self.v_optimal[0, :]
        # self.new_valid_pair = self.valid_pairs[0, :]

        v_optimal_arr = np.array(v_optimal)
        cost_arr = np.array(cost)

        cost_argsort = np.argsort(cost_arr)
        valid_pairs = valid_pairs[cost_argsort, :]
        v_optimal_arr = v_optimal_arr[cost_argsort, :]
        cost_arr = cost_arr[cost_argsort]

    # Iteratively remove the pair (v1, v2) of least cost from the heap
    # contract this pair, and update the costs of all valid pairs involving (v1, v2).
    # until existing points = ratio * original points
    def iteratively_remove_least_cost_valid_pairs(self) -> None:
        self.new_point_count = 0
        self.status_points = np.zeros(self.original_number_of_points)
        self.status_faces = np.zeros(self.number_of_faces)
        while (self.original_number_of_points - self.new_point_count) >= self.ratio * (self.original_number_of_points):
            # current valid pair
            current_valid_pair = self.new_valid_pair
            v_1_location = current_valid_pair[0] - 1  # point location in self.points
            v_2_location = current_valid_pair[1] - 1

            # update self.points
            # put the top optimal vertex(point) into the sequence of points
            self.original_points[v_1_location, :] = self.new_point.reshape(1, 3)
            self.original_points[v_2_location, :] = self.new_point.reshape(1, 3)

            # set status of points
            # 0 means no change, -1 means the point is deleted
            # update v1, v2 to v_opt, then delete v2, keep v1
            self.status_points[v_2_location] = -1

            # set status of faces
            # 0 means no change, -1 means the face will be deleted
            v_1_in_faces_loc = np.where(self.original_faces == (v_1_location + 1))
            v_2_in_faces_loc = np.where(self.original_faces == (v_2_location + 1))
            v_1_2_in_one_face_loc = []
            for item in v_2_in_faces_loc[0]:
                if np.where(v_1_in_faces_loc[0] == item)[0].size > 0:
                    v_1_2_in_one_face_loc.append(item)
            v_1_2_in_one_face_loc = np.array(v_1_2_in_one_face_loc)
            if v_1_2_in_one_face_loc.size >= 1:
                self.status_faces[v_1_2_in_one_face_loc] = -1

            # update self.faces
            # points of faces involving v1 and v2 are changed accordingly
            # set v2 to v1
            self.original_faces[v_2_in_faces_loc] = v_1_location + 1

            # update edges
            # edge_1=np.delete(self.faces[:,0:2], v_1_2_in_one_face_loc, axis=0)
            # edge_2=np.delete(self.faces[:,1:], v_1_2_in_one_face_loc, axis=0)
            # edge_3=np.delete(np.concatenate([self.faces[:,:1], self.faces[:,-1:]], axis=1),
            # v_1_2_in_one_face_loc, axis=0)
            # self.edges=np.concatenate([edge_1, edge_2, edge_3], axis=0)

            # update self.plane_equ_para
            v_1_2_in_faces_loc = np.unique(np.append(v_1_in_faces_loc[0], v_2_in_faces_loc[0]))
            self.update_plane_equation_parameters(v_1_2_in_faces_loc)

            # update self.Q_matrices
            self.update_q(current_valid_pair - 1, v_1_location)

            # update self.valid_pairs, self.v_optimal, and self.cost
            self.update_valid_pairs_v_optimal_and_cost(v_1_location)
            # re-calculate optimal contraction pairs and cost
            self.update_optimal_contraction_pairs_and_cost(v_1_location)

            if self.new_point_count % 100 == 0:
                print(
                    "Simplification: "
                    + str(
                        100 * (self.original_number_of_points - self.new_point_count) / (self.original_number_of_points)
                    )
                    + "%"
                )
                print("Remaining: " + str(self.original_number_of_points - self.new_point_count) + " points")
                print("\n")

            self.new_point_count = self.new_point_count + 1

        print(
            "Simplification: "
            + str(
                100
                * (self.original_number_of_points - self.new_point_count)
                / (self.original_number_of_points + self.new_point_count)
            )
            + "%"
        )
        print("Remaining: " + str(self.original_number_of_points - self.new_point_count) + " points")
        print("End\n")

    def calculate_plane_equation_for_one_face(self, p1: NDArray, p2: NDArray, p3: NDArray) -> NDArray:
        # input: p1, p2, p3 numpy.array, shape: (3, 1) or (1,3) or (3, )
        # p1 ,p2, p3 (x, y, z) are three points on a face
        # plane equ: ax+by+cz+d=0 a^2+b^2+c^2=1
        # return: numpy.array (a, b, c, d), shape: (1,4)
        p1 = np.array(p1).reshape(3)
        p2 = np.array(p2).reshape(3)
        p3 = np.array(p3).reshape(3)
        point_mat = np.array([p1, p2, p3])
        abc = np.matmul(np.linalg.inv(point_mat), np.array([[1], [1], [1]]))
        output = np.concatenate([abc.T, np.array(-1).reshape(1, 1)], axis=1) / (np.sum(abc**2) ** 0.5)
        output = output.reshape(4)
        return output

    def update_plane_equation_parameters(self, need_updating_loc: NDArray) -> None:
        # input: need_updating_loc, a numpy.array, shape: (n, ), locations of self.plane_equ_para need updating
        for i in need_updating_loc:
            if self.status_faces[i] == -1:
                self.plane_equ_para[i, :] = np.array([0, 0, 0, 0]).reshape(1, 4)
            else:
                point_1 = self.original_points[self.original_faces[i, 0] - 1, :]
                point_2 = self.original_points[self.original_faces[i, 1] - 1, :]
                point_3 = self.original_points[self.original_faces[i, 2] - 1, :]
                self.plane_equ_para[i, :] = self.calculate_plane_equation_for_one_face(point_1, point_2, point_3)

    def update_q(self, replace_locs: NDArray, target_loc: NDArray) -> None:
        # input: replace_locs, a numpy.array, shape: (2, ), locations of self.points need updating
        # input: target_loc, a number, location of self.points need updating
        face_set_index = np.where(self.original_faces == target_loc + 1)[0]
        q_temp = np.zeros((4, 4))

        for j in face_set_index:
            p = self.plane_equ_para[j, :]
            p = p.reshape(1, len(p))
            q_temp = q_temp + np.matmul(p.T, p)

        for i in replace_locs:
            self.Q_matrices[i] = q_temp

    def update_valid_pairs_v_optimal_and_cost(self, target_loc: NDArray) -> None:
        # input: target_loc, a number, location of self.points need updating

        # processing self.valid_pairs
        # replace all the point indexes containing current valid pair with new point index: target_loc+1
        v_1_loc_in_valid_pairs = np.where(self.valid_pairs == self.new_valid_pair[0])
        v_2_loc_in_valid_pairs = np.where(self.valid_pairs == self.new_valid_pair[1])

        self.valid_pairs[v_1_loc_in_valid_pairs] = target_loc + 1
        self.valid_pairs[v_2_loc_in_valid_pairs] = target_loc + 1

        delete_locs = []
        for item in v_1_loc_in_valid_pairs[0]:
            if np.where(v_2_loc_in_valid_pairs[0] == item)[0].size > 0:
                delete_locs.append(item)
        delete_locs = np.array(delete_locs)

        find_same = self.valid_pairs[:, 1] - self.valid_pairs[:, 0]
        find_same_loc = np.where(find_same == 0)[0]
        if find_same_loc.size >= 1:
            delete_locs = np.append(delete_locs, find_same_loc)

        # delete process for self.valid_pairs, self.v_optimal and self.cost
        self.valid_pairs = np.delete(self.valid_pairs, delete_locs, axis=0)
        self.v_optimal = np.delete(self.v_optimal, delete_locs, axis=0)
        self.cost = np.delete(self.cost, delete_locs, axis=0)

        # unique process for self.valid_pairs, self.v_optimal and self.cost
        unique_valid_pairs_trans, unique_valid_pairs_loc = np.unique(
            self.valid_pairs[:, 0] * (10**10) + self.valid_pairs[:, 1],
            return_index=True,
        )
        self.valid_pairs = self.valid_pairs[unique_valid_pairs_loc, :]
        self.v_optimal = self.v_optimal[unique_valid_pairs_loc, :]
        self.cost = self.cost[unique_valid_pairs_loc]

    def update_optimal_contraction_pairs_and_cost(self, target_loc: NDArray) -> None:
        # input: target_loc, a number, location of self.points need updating
        v_target_loc_in_valid_pairs = np.where(self.valid_pairs == target_loc + 1)[0]
        for i in v_target_loc_in_valid_pairs:
            current_valid_pair = self.valid_pairs[i, :]
            v_1_location = current_valid_pair[0] - 1
            v_2_location = current_valid_pair[1] - 1
            # find Q_1
            q_1 = self.Q_matrices[v_1_location]
            # find Q_2
            q_2 = self.Q_matrices[v_2_location]
            q = q_1 + q_2
            q_new = np.concatenate([q[:3, :], np.array([0, 0, 0, 1]).reshape(1, 4)], axis=0)
            if np.linalg.det(q_new) > 0:
                current_v_opt = np.matmul(np.linalg.inv(q_new), np.array([0, 0, 0, 1]).reshape(4, 1))
                current_cost = np.matmul(np.matmul(current_v_opt.T, q), current_v_opt)
                current_v_opt = current_v_opt.reshape(4)[:3]
            else:
                v_1 = np.append(self.original_points[v_1_location, :], 1).reshape(4, 1)
                v_2 = np.append(self.original_points[v_2_location, :], 1).reshape(4, 1)
                v_mid = (v_1 + v_2) / 2
                delta_v_1 = np.matmul(np.matmul(v_1.T, q), v_1)
                delta_v_2 = np.matmul(np.matmul(v_2.T, q), v_2)
                delta_v_mid = np.matmul(np.matmul(v_mid.T, q), v_mid)
                current_cost = np.min(np.array([delta_v_1, delta_v_2, delta_v_mid]))
                min_delta_loc = np.argmin(np.array([delta_v_1, delta_v_2, delta_v_mid]))
                current_v_opt = np.concatenate([v_1, v_2, v_mid], axis=1)[:, min_delta_loc].reshape(4)
                current_v_opt = current_v_opt[:3]
            self.v_optimal[i, :] = current_v_opt
            self.cost[i] = current_cost

        cost_argsort = np.argsort(self.cost)
        self.valid_pairs = self.valid_pairs[cost_argsort, :]
        self.v_optimal = self.v_optimal[cost_argsort, :]
        self.cost = self.cost[cost_argsort]

        self.new_point = self.v_optimal[0, :]
        self.new_valid_pair = self.valid_pairs[0, :]

    def generate_new_3d_model(self) -> None:
        point_serial_number = np.arange(self.original_points.shape[0]) + 1
        points_to_delete_locs = np.where(self.status_points == -1)[0]
        self.original_points = np.delete(self.original_points, points_to_delete_locs, axis=0)
        point_serial_number = np.delete(point_serial_number, points_to_delete_locs)
        point_serial_number_after_del = np.arange(self.original_points.shape[0]) + 1

        faces_to_delete_locs = np.where(self.status_faces == -1)[0]
        self.original_faces = np.delete(self.original_faces, faces_to_delete_locs, axis=0)

        for i in point_serial_number_after_del:
            point_loc_in_face = np.where(self.original_faces == point_serial_number[i - 1])
            self.original_faces[point_loc_in_face] = i

        self.original_number_of_points = self.original_points.shape[0]
        self.number_of_faces = self.original_faces.shape[0]

    def save_simplified_mesh(self, output_filepath: str | Path) -> None:
        with open(output_filepath, "w") as file_obj:
            file_obj.write(
                "# " + str(self.original_number_of_points) + " vertices, " + str(self.number_of_faces) + " faces\n"
            )
            for i in range(self.original_number_of_points):
                file_obj.write(
                    "v "
                    + str(self.original_points[i, 0])
                    + " "
                    + str(self.original_points[i, 1])
                    + " "
                    + str(self.original_points[i, 2])
                    + "\n"
                )
            for i in range(self.number_of_faces):
                file_obj.write(
                    "f "
                    + str(self.original_faces[i, 0])
                    + " "
                    + str(self.original_faces[i, 1])
                    + " "
                    + str(self.original_faces[i, 2])
                    + "\n"
                )
        print("Output simplified model: " + str(output_filepath))
