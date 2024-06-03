"""Tests mesh simplification."""

import tempfile
from pathlib import Path

from kol.logging import configure_logging
from kol.mesh import load_file
from kol.simplify import get_simplified_mesh


def test_mesh_simplify(tmpdir: Path) -> None:
    original_mesh = load_file("tests/data/random.stl")
    simplified_mesh = get_simplified_mesh(mesh=original_mesh, threshold=0.0001, simplify_ratio=0.5)
    # TODO: Uncomment when done testing.
    # simplify.save_simplified_mesh(simplified_path)
    # simplified_mesh = load_file(simplified_path)
    # assert simplified_mesh.faces.size < 0.5 * original_mesh.faces.size
    raise NotImplementedError(simplified_mesh)


if __name__ == "__main__":
    # python -m tests.test_simplify
    configure_logging()
    with tempfile.TemporaryDirectory() as tmpdir:
        test_mesh_simplify(Path(tmpdir))
