"""Defines utility functions for operations on meshes."""

from pathlib import Path
from typing import Literal, cast

import numpy as np
import stl.mesh
from numpy.typing import NDArray

MeshExt = Literal["stl", "obj"]


def stl_to_obj(stl_path: str | Path, obj_path: str | Path) -> None:
    """Converts an STL file to an OBJ file.

    Args:
        stl_path: The path to the STL file.
        obj_path: The path to the OBJ file.
    """
    mesh = stl.mesh.Mesh.from_file(stl_path)

    vertices: dict[tuple[float, float, float], int] = {}
    faces: list[tuple[int, int, int]] = []
    index = 1

    # Process each triangle in the mesh
    for i in range(len(mesh.vectors)):
        face = []
        for point in mesh.vectors[i]:
            vertex = cast(tuple[float, float, float], tuple(point))
            if vertex not in vertices:
                vertices[vertex] = index
                index += 1
            face.append(vertices[vertex])
        face_tuple = cast(tuple[int, int, int], tuple(face))
        faces.append(face_tuple)

    with open(obj_path, "w") as f:
        for vertex, _ in sorted(vertices.items(), key=lambda x: x[1]):
            f.write(f"v {' '.join(map(str, vertex))}\n")
        for face_tuple in faces:
            f.write(f"f {' '.join(map(str, face_tuple))}\n")


def stl_to_fmt(stl_path: str | Path, output_path: str | Path) -> None:
    stl_path = Path(stl_path)
    ext = Path(output_path).suffix[1:]

    match ext:
        case "stl":
            return

        case "obj":
            stl_to_obj(stl_path, output_path)

        case _:
            raise ValueError(f"Unsupported mesh format: {ext}")


def load_obj_file(obj_path: str | Path) -> tuple[NDArray, NDArray, NDArray]:
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
    return points, faces, edges


def load_stl_file(stl_path: str | Path) -> tuple[NDArray, NDArray, NDArray]:
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
    return points, faces, edges
