"""Tests geometry functions."""

from onshape.utils.geometry import get_mesh_convex_hull, scale_mesh
from onshape.utils.mesh import load_file


def test_scale_mesh() -> None:
    mesh = load_file("tests/data/random.stl")

    # Tests getting the mesh convex hull. The particular mesh is convex, so
    # the convex hull should be the same as the original mesh.
    hull_mesh = get_mesh_convex_hull(mesh)
    assert len(hull_mesh.faces) <= len(mesh.faces)

    # Tests scaling the mesh.
    scaled_mesh = scale_mesh(mesh, 2.0)
    assert not (scaled_mesh.points == mesh.points).all()
    assert (scaled_mesh.faces == mesh.faces).all()

    # Tests scaling the mesh about the origin.
    scaled_mesh = scale_mesh(mesh, 2.0, about_origin=True)
    assert not (scaled_mesh.points == mesh.points).all()
    assert (scaled_mesh.faces == mesh.faces).all()
