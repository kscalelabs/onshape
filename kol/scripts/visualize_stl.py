# mypy: disable-error-code="import-not-found, import-untyped, attr-defined, misc"
"""Visualizes a single STL file."""

import argparse
import logging
from typing import Sequence

from stl import mesh

from kol.logging import configure_logging

logger = logging.getLogger(__name__)


def main(args: Sequence[str] | None = None) -> None:
    configure_logging()

    parser = argparse.ArgumentParser(description="Show an STL file using Matplotlib")
    parser.add_argument("stl", help="Path to the STL file")
    parsed_args = parser.parse_args(args)

    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits import mplot3d
    except ImportError:
        raise ImportError("matplotlib is required to run this script")

    stl_mesh = mesh.Mesh.from_file(parsed_args.stl)

    # Create a new plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Add the loaded STL mesh
    ax.add_collection3d(mplot3d.art3d.Poly3DCollection(stl_mesh.vectors, alpha=0.5))

    # Auto scale to the mesh size
    scale = stl_mesh.points.flatten()
    ax.auto_scale_xyz(scale, scale, scale)

    # Show the origin
    ax.scatter([0], [0], [0], color="red", s=100, label="Origin")

    # Show the plot to the screen
    plt.legend()
    plt.show()


if __name__ == "__main__":
    # python -m kol.scripts.visualize_stl
    main()
