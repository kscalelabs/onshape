"""Defines utility functions for working with mesh files."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Union, cast, get_args

import numpy as np
from numpy.typing import NDArray
from stl import mesh as stl_mesh

MeshType = Literal["stl", "obj"]


@dataclass
class Mesh:
    """Defines a common representation for a mesh.

    Attributes:
        points: The points of the mesh, where each point is a 3D coordinate.
        faces: The faces of the mesh, where each face is a triangle defined
            by the indices of its vertices in the points array.
    """

    points: NDArray
    faces: NDArray

    def __eq__(self, other: Any) -> bool:  # noqa: ANN401
        if not isinstance(other, Mesh):
            return False
        return np.allclose(self.points, other.points) and np.array_equal(self.faces, other.faces)

    def save(self, file_path: Union[str, Path]) -> None:
        save_file(self, file_path)


def get_mesh_type(file_path: Union[str, Path]) -> MeshType:
    ext = Path(file_path).suffix[1:]
    if ext not in get_args(MeshType):
        raise ValueError(f"Unsupported mesh format: {ext}")
    return cast(MeshType, ext)


def stl_to_fmt(stl_path: Union[str, Path], output_path: Union[str, Path]) -> None:
    if get_mesh_type(output_path) != "stl":
        save_file(load_stl_file(stl_path), output_path)


def fmt_to_stl(input_path: Union[str, Path], stl_path: Union[str, Path]) -> None:
    if get_mesh_type(input_path) != "stl":
        save_stl_file(load_file(input_path), stl_path)


def load_file(file_path: Union[str, Path]) -> Mesh:
    """Loads a mesh file to a common format.

    Args:
        file_path: The path to the mesh file.

    Returns:
        The loaded mesh.
    """
    ext = get_mesh_type(file_path)

    match ext:
        case "stl":
            return load_stl_file(file_path)

        case "obj":
            return load_obj_file(file_path)

        case _:
            raise ValueError(f"Unsupported mesh format: {ext}")


def save_file(mesh: Mesh, file_path: Union[str, Path]) -> None:
    """Saves the mesh to a file.

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


def load_obj_file(obj_path: Union[str, Path]) -> Mesh:
    """Loads an OBJ file and returns the mesh in the common format.

    Args:
        obj_path: The path to the OBJ file.

    Returns:
        The loaded mesh.
    """
    with open(obj_path) as file:
        points_list: list[tuple[float, float, float]] = []
        faces_list: list[tuple[int, int, int]] = []
        while True:
            line = file.readline()
            if not line:
                break
            strs = line.split()
            if strs[0] == "v":
                points_list.append((float(strs[1]), float(strs[2]), float(strs[3])))
            elif strs[0] == "f":
                # Subtract 1 from each index to convert to 0-based indexing
                faces_list.append((int(strs[1]) - 1, int(strs[2]) - 1, int(strs[3]) - 1))
    points = np.array(points_list)
    faces = np.array(faces_list)
    return Mesh(points, faces)


def save_obj_file(mesh: Mesh, obj_path: Union[str, Path]) -> None:
    """Saves the mesh as an OBJ file.

    Args:
        mesh: The mesh to save.
        obj_path: The path to the OBJ file.
    """
    with open(obj_path, "w") as f:
        for point in mesh.points:
            f.write(f"v {' '.join(map(str, point))}\n")
        for face in mesh.faces:
            f.write(f"f {' '.join(map(str, face + 1))}\n")  # Convert to 1-based indexing


def load_stl_file(stl_path: Union[str, Path]) -> Mesh:
    """Loads an STL file and returns the mesh in the common format.

    Args:
        stl_path: The path to the STL file.

    Returns:
        The loaded mesh.
    """
    mesh = stl_mesh.Mesh.from_file(stl_path)
    points = mesh.vectors.reshape(-1, 3)
    faces = np.arange(len(points)).reshape(-1, 3)
    return Mesh(points, faces)


def save_stl_file(mesh: Mesh, stl_path: Union[str, Path]) -> None:
    """Saves the mesh as an STL file.

    Args:
        mesh: The mesh to save.
        stl_path: The path to the STL file.
    """
    # Check the maximum value in mesh.faces
    # max_face_index = np.max(mesh.faces)
    # if max_face_index >= len(mesh.points):
    #     raise ValueError(
    #         f"Index in mesh.faces ({max_face_index}) is out of bounds for mesh.points of size {len(mesh.points)}"
    #     )

    # Create an empty STL mesh with the correct number of faces
    stl_mesh_obj = stl_mesh.Mesh(np.zeros(len(mesh.faces), dtype=stl_mesh.Mesh.dtype))

    # Populate the STL mesh with the vertices for each face
    for i, face in enumerate(mesh.faces):
        for j in range(3):
            stl_mesh_obj.vectors[i][j] = mesh.points[face[j]]

    # Save the populated STL mesh to the specified path
    stl_mesh_obj.save(stl_path)
