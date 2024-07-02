"""Function for converting urdf to mjcf."""

import argparse
from pathlib import Path
from typing import Sequence

from kol.formats import mjcf


def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Convert a URDF file to MJCF.")
    parser.add_argument("urdf_path", type=str, help="The path to the URDF file")
    parser.add_argument("robot_name", type=str, help="The name of the robot")
    parser.add_argument("output_dir", type=str, help="The output directory")
    parsed_args = parser.parse_args(args)

    urdf_to_mjcf(Path(parsed_args.urdf_path), parsed_args.robot_name)


def urdf_to_mjcf(urdf_path: Path, robot_name: str) -> None:
    mjcf_robot = mjcf.Robot(robot_name, urdf_path, mjcf.Compiler(angle="radian", meshdir="meshes"))
    mjcf_robot.adapt_world()
    mjcf_robot.save(urdf_path / f"{robot_name}.xml")
