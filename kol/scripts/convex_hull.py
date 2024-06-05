"""Gets convex hull from mesh."""

import argparse
from typing import Sequence

from kol.convex_hull import get_convex_hull
from kol.mesh import save_file


def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Simplify a mesh")
    parser.add_argument("filepath", type=str, help="The path to the mesh file")
    parser.add_argument("output", type=str, help="The path to save the simplified mesh")
    parser.add_argument("scaling", type=float, help="The scaling factor for the mesh")
    parsed_args = parser.parse_args(args)

    output_mesh = get_convex_hull(
        mesh=parsed_args.filepath,
        scaling=parsed_args.scaling,
    )
    save_file(output_mesh, parsed_args.output)
