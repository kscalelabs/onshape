"""Runs mesh simpification."""

import argparse
from typing import Sequence

from kol.simplify import MeshSimplify


def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Simplify a mesh")
    parser.add_argument("filepath", type=str, help="The path to the mesh file")
    parser.add_argument("output", type=str, help="The path to save the simplified mesh")
    parser.add_argument("threshold", type=float, help="The threshold to simplify the mesh")
    parser.add_argument("simplify-ratio", type=float, help="The ratio to simplify the mesh")
    parsed_args = parser.parse_args(args)

    MeshSimplify(
        filepath=parsed_args.filepath,
        threshold=parsed_args.threshold,
        simplify_ratio=parsed_args.simplify_ratio,
    ).save_simplified_mesh(parsed_args.output)
