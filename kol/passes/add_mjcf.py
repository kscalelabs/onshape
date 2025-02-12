"""Defines a pass to get an MJCF file from the URDF."""

import argparse
from pathlib import Path

from kol.formats.mjcf import ConversionMetadata, convert_to_mjcf_metadata

SCENE_TEMPLATE = """
<mujoco model="{name} scene">
  <include file="{path}"/>

  <statistic center="0.15 0.1 0.38" extent=".8" meansize="0.05"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="220" elevation="-10"/>
    <quality shadowsize="8192"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
"""


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


def write_scene_file(scene_file: Path, mjcf_file: Path) -> None:
    """Writes a scene file to the package."""
    scene_file.write_text(
        SCENE_TEMPLATE.format(
            name=mjcf_file.stem,
            path=mjcf_file.relative_to(scene_file.parent).as_posix(),
        )
    )


def convert_urdf_to_mjcf(
    urdf_file: str | Path,
    mjcf_file: str | Path | None = None,
    metadata: ConversionMetadata | None = None,
) -> list[Path]:
    """Convert URDF to MJCF format.

    Args:
        urdf_file: Path to input URDF file
        mjcf_file: Optional path for output MJCF file
        metadata: Optional metadata for the conversion

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

    # Adds a scene file to the package.
    files = [mjcf_file]
    if metadata is not None and metadata.add_mjcf_scene:
        scene_file = mjcf_file.with_suffix(f".scene{mjcf_file.suffix}")
        write_scene_file(scene_file, mjcf_file)
        files.append(scene_file)

    return files


def main() -> None:
    parser = argparse.ArgumentParser(description="Merge fixed joints in a URDF.")
    parser.add_argument("urdf_path", type=Path, help="The path to the URDF file.")
    args = parser.parse_args()

    convert_urdf_to_mjcf(args.urdf_path)


if __name__ == "__main__":
    # python -m kol.passes.simplify_meshes
    main()
