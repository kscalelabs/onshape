"""Tests mesh simplification."""

from pathlib import Path

from kol.mesh import load_file
from kol.scripts.simplify import MeshSimplify


def test_mesh_simplify(tmpdir: Path) -> None:
    original_mesh = load_file("tests/data/random.stl")
    simplified_path = Path(tmpdir / "simplified.stl")
    simplify = MeshSimplify(mesh=original_mesh, threshold=0.01, simplify_ratio=0.5)
    # TODO: Uncomment when done testing.
    # simplify.save_simplified_mesh(simplified_path)
    # simplified_mesh = load_file(simplified_path)
    # assert simplified_mesh.faces.size < 0.5 * original_mesh.faces.size
    print(simplify.dist_pairs)
    asdf
