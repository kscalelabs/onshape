"""Tests mesh simplification."""

from pathlib import Path

from kol.convex_hull import get_convex_hull
from kol.mesh import load_file, save_file


def test_convex_hull(tmpdir: Path) -> None:
    original_mesh = load_file("tests/data/random.stl")
    convex_hull = get_convex_hull(mesh=original_mesh, scaling=1.0)
    save_file(convex_hull, "tests/data/random_convex_hull.stl")
    convex_hull = load_file("tests/data/random_convex_hull.stl")
    assert convex_hull.faces.size <= original_mesh.faces.size
