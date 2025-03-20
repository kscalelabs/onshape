"""Simple script to interact with a URDF in Mujoco."""

import argparse
import logging
from pathlib import Path
from typing import Sequence

from onshape.utils.logging import configure_logging

logger = logging.getLogger(__name__)


def mujoco_main(args: Sequence[str] | None = None) -> None:
    configure_logging()

    parser = argparse.ArgumentParser(description="Show a MJCF in Mujoco")
    parser.add_argument("mjcf", nargs="?", help="Path to the MJCF file")
    parser.add_argument("--dt", type=float, default=0.01, help="Time step")
    parser.add_argument("--start-height", type=float, default=1.0, help="Initial height of the robot")
    parser.add_argument("--fixed-base", action="store_true", help="Fix the base linkage in place")
    parser.add_argument("--cycle-duration", type=float, default=10, help="Duration of the joint cycling")
    parsed_args = parser.parse_args(args)

    try:
        from mujoco.viewer import launch_from_path
    except ImportError:
        raise ImportError(
            "mujoco and mujoco_viewer are required to run this script; install them "
            "with `pip install mujoco mujoco-python-viewer`"
        )

    # Load the MJCF file
    mjcf_path = Path("robot" if parsed_args.mjcf is None else parsed_args.mjcf)
    if mjcf_path.is_dir():
        try:
            mjcf_path = next(mjcf_path.glob("*.mjcf"))
        except StopIteration:
            raise FileNotFoundError(f"No MJCF files found in {mjcf_path}")
    elif not mjcf_path.is_file():
        raise FileNotFoundError(mjcf_path)

    # Set up the viewer
    launch_from_path(str(mjcf_path))


if __name__ == "__main__":
    # python -m onshape.scripts.mujoco
    mujoco_main()
