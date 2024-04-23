"""Defines utility functions for operations on meshes."""

from pathlib import Path
from typing import Literal, cast

import stl.mesh

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
