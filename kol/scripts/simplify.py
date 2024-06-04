"""Runs mesh simpification."""

import argparse
from typing import Sequence

from kol.mesh import save_file
from kol.simplify import get_simplified_mesh


def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Simplify a mesh")
    parser.add_argument("filepath", type=str, help="The path to the mesh file")
    parser.add_argument("output", type=str, help="The path to save the simplified mesh")
    parser.add_argument("threshold", type=float, help="The threshold to simplify the mesh")
    parser.add_argument("simplify_ratio", type=float, help="The ratio to simplify the mesh")
    parsed_args = parser.parse_args(args)

    output_mesh = get_simplified_mesh(
        mesh=parsed_args.filepath,
        threshold=parsed_args.threshold,
        simplify_ratio=parsed_args.simplify_ratio,
    )
    save_file(output_mesh, parsed_args.output)
