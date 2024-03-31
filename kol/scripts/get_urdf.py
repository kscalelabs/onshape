# mypy: disable-error-code="attr-defined"
"""Defines utility functions for importing the robot model from OnShape."""

import argparse
import logging
from typing import Sequence

import numpy as np

from kol import urdf
from kol.logging import configure_logging
from kol.onshape.converter import Converter


def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Import the robot model from OnShape")
    parser.add_argument("document_url", type=str, help="The ID of the document to import")
    parser.add_argument("-o", "--output-dir", type=str, help="The path to save the imported model")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    parser.add_argument("--max-force", type=float, default=80.0, help="The maximum force for a prismatic joint")
    parser.add_argument("--max-velocity", type=float, default=5.0, help="The maximum velocity for a prismatic joint")
    parser.add_argument("--max-length", type=float, default=1.0, help="The maximum length for a prismatic joint")
    parser.add_argument("--max-torque", type=float, default=80.0, help="The maximum force for a revolute joint")
    parser.add_argument("--max-ang-velocity", type=float, default=5.0, help="The maximum velocity for a revolute joint")
    parser.add_argument("--max-angle", type=float, default=np.pi, help="The maximum angle for a revolute joint")
    parsed_args = parser.parse_args(args)

    configure_logging(level=logging.DEBUG if parsed_args.debug else logging.INFO)

    Converter(
        document_url=parsed_args.document_url,
        output_dir=parsed_args.output_dir,
        default_prismatic_joint_limits=urdf.JointLimits(
            parsed_args.max_force,
            parsed_args.max_velocity,
            -parsed_args.max_length,
            parsed_args.max_length,
        ),
        default_revolute_joint_limits=urdf.JointLimits(
            parsed_args.max_torque,
            parsed_args.max_ang_velocity,
            -parsed_args.max_angle,
            parsed_args.max_angle,
        ),
    ).save_urdf()


if __name__ == "__main__":
    # python -m kol.scripts.import_onshape
    main()
