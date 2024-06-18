"""Improve robot kinematics and accuracy by merging fixed joints in a urdf."""

import argparse
from pathlib import Path
from typing import Sequence

from kol.merge_fixed_joints import get_merged_urdf


def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Merge fixed joints in a urdf.")
    parser.add_argument("filepath", type=str, help="The path to the urdf file")
    parser.add_argument("--scaling", type=float, default=1, help="The scaling factor for each fused mesh")
    parsed_args = parser.parse_args(args)

    urdf_path = Path(parsed_args.filepath)
    get_merged_urdf(
        urdf_path=urdf_path,
        scaling=parsed_args.scaling,
    )
