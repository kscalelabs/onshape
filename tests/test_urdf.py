"""Performs some simple checks for writing URDF files."""

from pathlib import Path

from kol.formats import urdf


def test_urdf(tmpdir: Path) -> None:
    links: list[urdf.Link | urdf.BaseJoint] = [
        urdf.Link(
            "first",
            urdf.VisualLink(
                origin=urdf.Origin((0, 0, 0), (0, 0, 0)),
                geometry=urdf.BoxGeometry((1.5, 1.0, 1.0)),
                material=urdf.Material.from_color("red"),
            ),
            urdf.CollisionLink(
                origin=urdf.Origin((0, 0, 0), (0, 0, 0)),
                geometry=urdf.BoxGeometry((1.0, 1.0, 1.0)),
            ),
            urdf.InertialLink(
                mass=1.0,
                inertia=urdf.Inertia(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
                origin=urdf.Origin((0, 0, 0), (0, 0, 0)),
            ),
        ),
        urdf.Link(
            "second",
            urdf.VisualLink(
                origin=urdf.Origin((0, 0, 0), (0, 0, 0)),
                geometry=urdf.MeshGeometry("mesh.stl"),
                material=urdf.Material.from_color("blue"),
            ),
            urdf.CollisionLink(
                origin=urdf.Origin((0, 0, 0), (0, 0, 0)),
                geometry=urdf.SphereGeometry(1.0),
            ),
            urdf.InertialLink(
                mass=1.0,
                inertia=urdf.Inertia(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
                origin=urdf.Origin((0, 0, 0), (0, 0, 0)),
            ),
        ),
        urdf.RevoluteJoint(
            name="joint",
            parent="first",
            child="second",
            origin=urdf.Origin((0, 0, 0), (0, 0, 0)),
            limits=urdf.JointLimits(1.0, 1.0, -1.0, 1.0),
            axis=urdf.Axis((0, 0, 1)),
        ),
    ]

    robot = urdf.Robot("robot", links)
    save_path = Path(tmpdir / "robot.urdf")
    robot.save(save_path)

    # Check that the saved URDF is a valid XML file.
    assert save_path.exists()
    assert save_path.stat().st_size > 0
    assert save_path.suffix == ".urdf"
    assert save_path.read_text()
