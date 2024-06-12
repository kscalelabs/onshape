"""Defines utility functions for working with mesh files."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Union, cast, get_args

import numpy as np
import trimesh
from numpy.typing import NDArray

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

    @classmethod
    def from_trimesh(cls, mesh: trimesh.Trimesh) -> "Mesh":
        return cls(mesh.vertices, mesh.faces)

    def to_trimesh(self) -> trimesh.Trimesh:
        return trimesh.Trimesh(self.points, self.faces)


def get_mesh_type(file_path: Union[str, Path]) -> MeshType:
    ext = Path(file_path).suffix[1:]
    if ext not in get_args(MeshType):
        raise ValueError(f"Unsupported mesh format: {ext}")
    return cast(MeshType, ext)


def stl_to_fmt(stl_path: Union[str, Path], output_path: Union[str, Path]) -> None:
    if get_mesh_type(output_path) != "stl":
        save_file(load_file(stl_path), output_path)


def fmt_to_stl(input_path: Union[str, Path], stl_path: Union[str, Path]) -> None:
    if get_mesh_type(input_path) != "stl":
        save_file(load_file(input_path), stl_path)


def load_file(file_path: Union[str, Path]) -> Mesh:
    mesh = trimesh.load(file_path)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Unsupported mesh type: {type(mesh)}")
    return Mesh.from_trimesh(mesh)


def save_file(mesh: Mesh, file_path: Union[str, Path]) -> None:
    mesh.to_trimesh().export(file_path)
