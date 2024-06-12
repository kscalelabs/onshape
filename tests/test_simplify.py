"""Tests mesh simplification."""

import tempfile
from pathlib import Path

from kol.logging import configure_logging
from kol.mesh import load_file, save_file
from kol.simplify import get_simplified_mesh


def test_mesh_simplify(tmpdir: Path) -> None:
    ratio = 0.95
    original_mesh = load_file("tests/data/random.stl")
    simplified_path = Path(tmpdir / "random_simplified.stl")

    simplified_mesh = get_simplified_mesh(mesh=original_mesh, threshold=0.0001, simplify_ratio=ratio)
    save_file(simplified_mesh, simplified_path)
    save_file(original_mesh, simplified_path)

    simplified_mesh_loaded = load_file(simplified_path)
    assert len(simplified_mesh_loaded.points) < ratio * len(original_mesh.points)


if __name__ == "__main__":
    # python -m tests.test_simplify
    configure_logging()
    with tempfile.TemporaryDirectory() as tmpdir:
        test_mesh_simplify(Path(tmpdir))
