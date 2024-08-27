"""Defines a pass to get an MJCF file from the URDF."""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

from kol.formats.common import save_xml
from kol.passes.utils import iter_meshes


def convert_joint_type(urdf_type: str) -> str:
    type_map = {
        "revolute": "hinge",
        "prismatic": "slide",
        "continuous": "hinge",
        "fixed": "fixed",
        "floating": "free",
        "planar": "slide",
    }
    return type_map.get(urdf_type, "hinge")


def parse_xyz(xyz_str: str) -> str:
    return " ".join([str(float(x)) for x in xyz_str.split()])


def convert_urdf_to_mjcf(urdf_file: str | Path, mjcf_file: str | Path) -> None:
    try:
        import mujoco
    except ImportError as e:
        raise ImportError(
            "Please install the package with Mujoco as a dependency, using "
            "`pip install kscale-onshape-library[mujoco]`"
        ) from e

    urdf_file = Path(urdf_file)
    mjcf_file = urdf_file.with_suffix(".mjcf")

    # Because of a quirk in the MuJoCo library, we need to symlink all of the
    # mesh files into the same directory as the MJCF file. We then need to
    # update the paths in the MJCF file to point at the correct locations.
    symlink_to_orig: dict[str, Path] = {}
    for (_, visual_mesh_path), (_, collision_mesh_path) in iter_meshes(urdf_file):
        for mesh_path in list({visual_mesh_path, collision_mesh_path}):
            mesh_name = mesh_path.name
            mesh_symlink = mjcf_file.parent / mesh_name
            if not mesh_symlink.exists():
                mesh_symlink.symlink_to(mesh_path)
            symlink_to_orig[mesh_name] = mesh_path

    model = mujoco.MjModel.from_xml_path(urdf_file.as_posix())
    mujoco.mj_saveLastXML(mjcf_file.as_posix(), model)

    # Clean up the symlinks.
    for mesh_name in symlink_to_orig:
        (mjcf_file.parent / mesh_name).unlink()

    # Read the MJCF file and update the paths to the meshes.
    mjcf_tree = ET.parse(mjcf_file)

    for asset in mjcf_tree.iter("asset"):
        for mesh in asset.iter("mesh"):
            mesh_name = Path(mesh.attrib["file"]).name
            mesh.attrib["file"] = symlink_to_orig[mesh_name].relative_to(mjcf_file.parent).as_posix()

    # Turn off internal collisions
    root = mjcf_tree.getroot()
    for element in root:
        if element.tag == "geom":
            element.attrib["contype"] = str(1)
            element.attrib["conaffinity"] = str(0)

    # Write the updated MJCF file.
    save_xml(mjcf_file, mjcf_tree)


def main() -> None:
    parser = argparse.ArgumentParser(description="Merge fixed joints in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    convert_urdf_to_mjcf(args.urdf_path, args.urdf_path.with_suffix(".mjcf"))


if __name__ == "__main__":
    # python -m kol.passes.simplify_meshes
    main()
