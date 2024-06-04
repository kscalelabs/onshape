"""Tests mesh simplification."""

import tempfile
from pathlib import Path

from kol.logging import configure_logging
from kol.mesh import load_file, save_file
from kol.simplify import get_simplified_mesh


def test_mesh_simplify(tmpdir: Path) -> None:
    original_mesh = load_file("tests/data/random.stl")
    simplified_mesh = get_simplified_mesh(mesh=original_mesh, threshold=0.0001, simplify_ratio=0.5)
    save_file(simplified_mesh, "tests/data/random_simplified.stl")
    simplified_mesh = load_file("tests/data/random_simplified.stl")
    assert simplified_mesh.faces.size < 0.5 * original_mesh.faces.size


if __name__ == "__main__":
    # python -m tests.test_simplify
    configure_logging()
    with tempfile.TemporaryDirectory() as tmpdir:
        test_mesh_simplify(Path(tmpdir))
