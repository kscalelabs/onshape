"""Tests mesh conversion options."""

from pathlib import Path

from kol.mesh import stl_to_fmt


def test_stl_to_obj(tmpdir: Path) -> None:
    obj_path = Path(tmpdir / "random.obj")
    stl_to_fmt("tests/data/random.stl", obj_path)
    assert obj_path.exists()
