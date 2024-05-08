# mypy: disable-error-code="import-not-found"
"""Simple script to interact with a MJCF in MuJoCo.

Run with mjpython:
    mjpython kol/scripts/show_mjcf.py robot/test_assembly_4.xml
"""

import argparse
import time
from typing import Sequence

import mujoco
import mujoco.viewer

from kol.logging import configure_logging


def main(args: Sequence[str] | None = None) -> None:
    configure_logging()
    parser = argparse.ArgumentParser(description="Show a MJCF in Mujoco")
    parser.add_argument("path_mjcf", nargs="?", help="Path to the MJCF file")
    parsed_args = parser.parse_args(args)

    model = mujoco.MjModel.from_xml_path(parsed_args.path_mjcf)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    # python -m kol.scripts.show_mjcf
    main()
