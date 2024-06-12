"""Tests mesh conversion options."""

from pathlib import Path
from typing import get_args

import pytest

from kol.mesh import MeshType, load_file, save_file, stl_to_fmt


@pytest.mark.parametrize("mesh_type", get_args(MeshType))
def test_stl_to_type(mesh_type: MeshType, tmpdir: Path) -> None:
    if mesh_type == "stl":
        return
    fmt_path = Path(tmpdir / f"random.{mesh_type}")
    stl_to_fmt("tests/data/random.stl", fmt_path)
    assert fmt_path.exists()

    stl_mesh = load_file("tests/data/random.stl")
    fmt_mesh = load_file(fmt_path)

    assert stl_mesh == fmt_mesh

    # Tests saving as the target format.
    fmt_path = Path(tmpdir / f"random.{mesh_type}")
    save_file(stl_mesh, fmt_path)
    assert fmt_path.exists()
    fmt_mesh = load_file(fmt_path)
    assert stl_mesh == fmt_mesh
