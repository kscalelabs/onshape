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


def test_load_save(tmpdir: Path) -> None:
    stl_path = Path(tmpdir / "random.stl")
    obj_path = Path(tmpdir / "random.obj")

    # Test STL loading and saving
    stl_mesh = load_file("tests/data/random.stl")
    save_file(stl_mesh, stl_path)
    assert stl_path.exists()

    stl_mesh2 = load_file(stl_path)
    assert stl_mesh == stl_mesh2

    # Test OBJ loading and saving
    save_file(stl_mesh, obj_path)
    assert obj_path.exists()

    obj_mesh = load_file(obj_path)
    assert stl_mesh == obj_mesh
    save_file(obj_mesh, stl_path)
    assert stl_path.exists()

    stl_mesh3 = load_file(stl_path)
    assert stl_mesh == stl_mesh3

    # Cleanup
    obj_path.unlink()
    stl_path.unlink()
    assert not obj_path.exists()
    assert not stl_path.exists()
