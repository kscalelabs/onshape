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
    parser.add_argument("--max-length", type=float, default=1.0, help="The maximum length, in meters")
    parser.add_argument("--max-torque", type=float, default=80.0, help="The maximum force, in Nm")
    parser.add_argument("--max-ang-velocity", type=float, default=5.0, help="The maximum angular velocity, in rad/s")
    parser.add_argument("--max-angle", type=float, default=np.pi, help="The maximum angle, in radians")
    parser.add_argument("--suffix-to-joint-effort", type=str, nargs="+", help="The suffix to joint effort mapping")
    parser.add_argument("--suffix-to-joint-velocity", type=str, nargs="+", help="The suffix to joint velocity mapping")
    parsed_args = parser.parse_args(args)

    configure_logging(level=logging.DEBUG if parsed_args.debug else logging.INFO)

    suffix_to_joint_effort: list[tuple[str, float]] = []
    if parsed_args.suffix_to_joint_effort:
        for mapping in parsed_args.suffix_to_joint_effort:
            suffix, effort = mapping.split("=")
            suffix_to_joint_effort.append((suffix, float(effort.strip())))

    suffix_to_joint_velocity: list[tuple[str, float]] = []
    if parsed_args.suffix_to_joint_velocity:
        for mapping in parsed_args.suffix_to_joint_velocity:
            suffix, velocity = mapping.split("=")
            suffix_to_joint_velocity.append((suffix, float(velocity.strip())))

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
        suffix_to_joint_effort=suffix_to_joint_effort,
        suffix_to_joint_velocity=suffix_to_joint_velocity,
    ).save_urdf()


if __name__ == "__main__":
    # python -m kol.scripts.import_onshape
    main()
