"""Defines a pass to get an MJCF file from the URDF."""

import argparse
from pathlib import Path

from kol.formats.mjcf import ConversionMetadata, convert_to_mjcf_metadata


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


def convert_urdf_to_mjcf(
    urdf_file: str | Path,
    mjcf_file: str | Path | None = None,
    metadata: ConversionMetadata | None = None,
) -> Path:
    """Convert URDF to MJCF format.

    Args:
        urdf_file: Path to input URDF file
        mjcf_file: Optional path for output MJCF file
        default_joint_pd_params: Default joint PD params to use for joints
            without PD params.
        suffix_to_joint_pd_params: Suffix to joint PD params mapping.

    Returns:
        Path to the generated MJCF file
    """
    urdf_file = Path(urdf_file)
    if mjcf_file is None:
        mjcf_file = urdf_file.with_suffix(".mjcf")
    else:
        mjcf_file = Path(mjcf_file)

    try:
        from urdf2mjcf.convert import convert_urdf_to_mjcf
    except ImportError as e:
        raise ImportError(
            "Please install the package with `urdf2mjcf` as a dependency, using "
            "`pip install kscale-onshape-library[mujoco]`"
        ) from e

    convert_urdf_to_mjcf(
        urdf_path=urdf_file,
        mjcf_path=mjcf_file,
        metadata=None if metadata is None else convert_to_mjcf_metadata(metadata),
    )

    return mjcf_file


def main() -> None:
    parser = argparse.ArgumentParser(description="Merge fixed joints in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    convert_urdf_to_mjcf(args.urdf_path, args.urdf_path.with_suffix(".mjcf"))


if __name__ == "__main__":
    # python -m kol.passes.simplify_meshes
    main()
