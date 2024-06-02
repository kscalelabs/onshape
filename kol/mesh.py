"""Defines utility functions for operations on meshes."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, cast, get_args

import numpy as np
import stl.mesh
from numpy.typing import NDArray

MeshType = Literal["stl", "obj"]


@dataclass
class Mesh:
    points: NDArray
    faces: NDArray
    edges: NDArray

    def __eq__(self, other: Any) -> bool:  # noqa: ANN401
        if not isinstance(other, Mesh):
            return False
        return (
            np.allclose(self.points, other.points)
            and np.array_equal(self.faces, other.faces)
            and np.array_equal(self.edges, other.edges)
        )


def get_mesh_type(file_path: str | Path) -> MeshType:
    ext = Path(file_path).suffix[1:]
    if ext not in get_args(MeshType):
        raise ValueError(f"Unsupported mesh format: {ext}")
    return cast(MeshType, ext)


def stl_to_fmt(stl_path: str | Path, output_path: str | Path) -> None:
    if get_mesh_type(output_path) != "stl":
        save_file(load_stl_file(stl_path), output_path)


def fmt_to_stl(input_path: str | Path, stl_path: str | Path) -> None:
    if get_mesh_type(input_path) != "stl":
        save_stl_file(load_file(input_path), stl_path)


def load_file(file_path: str | Path) -> Mesh:
    """Loads a mesh file and returns the points, faces, and edges.

    Args:
        file_path: The path to the mesh file.

    Returns:
        A tuple containing the points, faces, and edges.
    """
    ext = get_mesh_type(file_path)

    match ext:
        case "stl":
            return load_stl_file(file_path)

        case "obj":
            return load_obj_file(file_path)

        case _:
            raise ValueError(f"Unsupported mesh format: {ext}")


def save_file(mesh: Mesh, file_path: str | Path) -> None:
    """Saves the points, faces, and edges to a mesh file.

    Args:
        mesh: The mesh to save.
        file_path: The path to the mesh file.
    """
    ext = get_mesh_type(file_path)

    match ext:
        case "stl":
            save_stl_file(mesh, file_path)

        case "obj":
            save_obj_file(mesh, file_path)

        case _:
            raise ValueError(f"Unsupported mesh format: {ext}")


def load_obj_file(obj_path: str | Path) -> Mesh:
    """Loads an OBJ file and returns the points, faces, and edges.

    Args:
        obj_path: The path to the OBJ file.

    Returns:
        A tuple containing the points, faces, and edges.
    """
    with open(obj_path) as file:
        points_list: list[tuple[float, float, float]] = []
        faces_list: list[tuple[int, int, int]] = []
        while 1:
            line = file.readline()
            if not line:
                break
            strs = line.split(" ")
            if strs[0] == "v":
                points_list.append((float(strs[1]), float(strs[2]), float(strs[3])))
            if strs[0] == "f":
                faces_list.append((int(strs[1]), int(strs[2]), int(strs[3])))
    points = np.array(points_list)
    faces = np.array(faces_list)
    edge_1 = faces[:, 0:2]
    edge_2 = faces[:, 1:]
    edge_3 = np.concatenate([faces[:, :1], faces[:, -1:]], axis=1)
    edges = np.concatenate([edge_1, edge_2, edge_3], axis=0)
    _, unique_edges_locs = np.unique(edges[:, 0] * (10**10) + edges[:, 1], return_index=True)
    edges = edges[unique_edges_locs, :]
    return Mesh(points, faces, edges)


def save_obj_file(mesh: Mesh, obj_path: str | Path) -> None:
    """Saves the points, faces, and edges to an OBJ file.

    Args:
        mesh: The mesh to save.
        obj_path: The path to the OBJ file.
    """
    with open(obj_path, "w") as f:
        for point in mesh.points:
            f.write(f"v {' '.join(map(str, point))}\n")
        for face in mesh.faces:
            f.write(f"f {' '.join(map(str, face))}\n")


def load_stl_file(stl_path: str | Path) -> Mesh:
    """Loads an STL file and returns the points, faces, and edges.

    Args:
        stl_path: The path to the STL file.

    Returns:
        A tuple containing the points, faces, and edges.
    """
    mesh = stl.mesh.Mesh.from_file(stl_path)
    points = mesh.vectors.reshape(-1, 3)
    faces = np.arange(len(points)).reshape(-1, 3)
    edge_1 = faces[:, 0:2]
    edge_2 = faces[:, 1:]
    edge_3 = np.concatenate([faces[:, :1], faces[:, -1:]], axis=1)
    edges = np.concatenate([edge_1, edge_2, edge_3], axis=0)
    _, unique_edges_locs = np.unique(edges[:, 0] * (10**10) + edges[:, 1], return_index=True)
    edges = edges[unique_edges_locs, :]
    return Mesh(points, faces, edges)


def save_stl_file(mesh: Mesh, stl_path: str | Path) -> None:
    """Saves the points, faces, and edges to an STL file.

    Args:
        mesh: The mesh to save.
        stl_path: The path to the STL file.
    """
    vectors = mesh.points.reshape(-1, 3, 3)
    faces = np.arange(len(vectors)).reshape(-1, 3)
    mesh = stl.mesh.Mesh(np.zeros(len(faces), dtype=stl.mesh.Mesh.dtype))
    for i, face in enumerate(faces):
        for j, vertex in enumerate(face):
            mesh.vectors[i][j] = vectors[vertex]
    mesh.save(stl_path)
