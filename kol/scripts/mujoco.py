# mypy: disable-error-code="import-not-found"
"""Simple script to interact with a URDF in MuJoCo."""

import argparse
import logging
import time
from pathlib import Path
from typing import Sequence

from kol.logging import configure_logging

logger = logging.getLogger(__name__)


def main(args: Sequence[str] | None = None) -> None:
    configure_logging()

    parser = argparse.ArgumentParser(description="Show a URDF in PyBullet")
    parser.add_argument("urdf", nargs="?", help="Path to the URDF file")
    parser.add_argument("--dt", type=float, default=0.01, help="Time step")
    parser.add_argument("-n", "--hide-gui", action="store_true", help="Hide the GUI")
    parsed_args = parser.parse_args(args)

    try:
        import mujoco_py as mjp
    except ImportError:
        raise ImportError("pybullet is required to run this script")

    # Connect to MuJoCo.
    model = mjp.load_model_from_path("robot.xml")
    sim = mjp.MjSim(model)

    # Turn off panels.
    if parsed_args.hide_gui:
        sim.viewer._hide_overlay = True

    # Enable mouse picking.
    sim.viewer._mouse_pick = True

    # Loads the floor plane.
    floor = sim.model.geom_name2id("floor")

    urdf_path = Path("robot" if parsed_args.urdf is None else parsed_args.urdf)
    if urdf_path.is_dir():
        try:
            urdf_path = next(urdf_path.glob("*.urdf"))
        except StopIteration:
            raise FileNotFoundError(f"No URDF files found in {urdf_path}")


if __name__ == "__main__":
    # python -m robot.cad.scripts.show_urdf
    main()
