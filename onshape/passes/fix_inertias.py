"""Defines a pass to ensure that all inertia matrices are positive definite, above epsilon."""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

DEFAULT_EPSILON = 1e-7  # kg m^2


def fix_inertias(urdf_path: Path, epsilon: float = DEFAULT_EPSILON) -> None:
    urdf_tree = ET.parse(urdf_path)

    # Ensure that all inertia matrices are positive definite by thresholding the diagonal.
    for link in urdf_tree.iter("link"):
        if (inertial := link.find("inertial")) is None:
            continue
        if (inertia := inertial.find("inertia")) is None:
            continue
        try:
            ixx, iyy, izz, ixy, ixz, iyz = [
                float(inertia.attrib[i]) for i in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
            ]
        except KeyError:
            continue
        imat = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        eigvals = np.linalg.eigvals(imat)
        if np.any(eigvals < epsilon):
            raise ValueError(f"Inertia matrix is not positive definite: {eigvals}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Fix inertias in an URDF file.")
    parser.add_argument(
        "urdf_path",
        type=Path,
        help="The path to the URDF file.",
    )
    parser.add_argument(
        "--epsilon",
        type=float,
        default=DEFAULT_EPSILON,
        help="The epsilon value to use for thresholding.",
    )
    args = parser.parse_args()

    fix_inertias(args.urdf_path, epsilon=args.epsilon)


if __name__ == "__main__":
    # python -m onshape.passes.fix_inertias
    main()
