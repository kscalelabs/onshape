"""Defines a pass to get an MJCF file from the URDF."""

import argparse
from pathlib import Path

from kol.onshape.config import JointPDParams


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
    default_joint_pd_params: JointPDParams | None = None,
    suffix_to_joint_pd_params: dict[str, JointPDParams] = {},
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
        from urdf2mjcf.convert import (
            JointParam,
            JointParamsMetadata,
            convert_urdf_to_mjcf,
        )
    except ImportError as e:
        raise ImportError(
            "Please install the package with `urdf2mjcf` as a dependency, using "
            "`pip install kscale-onshape-library[mujoco]`"
        ) from e

    metadata = JointParamsMetadata(
        suffix_to_pd_params={
            name: JointParam(
                kp=param.kp,
                kd=param.kd,
            )
            for name, param in suffix_to_joint_pd_params.items()
        },
        default=(
            JointParam(
                kp=default_joint_pd_params.kp,
                kd=default_joint_pd_params.kd,
            )
            if default_joint_pd_params is not None
            else None
        ),
    )
    convert_urdf_to_mjcf(urdf_file, mjcf_file, joint_params_metadata=metadata)
    return mjcf_file


def main() -> None:
    parser = argparse.ArgumentParser(description="Merge fixed joints in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    convert_urdf_to_mjcf(args.urdf_path, args.urdf_path.with_suffix(".mjcf"))


if __name__ == "__main__":
    # python -m kol.passes.simplify_meshes
    main()
