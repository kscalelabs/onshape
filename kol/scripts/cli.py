"""Defines the top-level KOL CLI."""

import argparse
from typing import Sequence

from kol.scripts import import_onshape, show_urdf


def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="K-Scale OnShape Library", add_help=False)
    parser.add_argument("subcommand", choices=["urdf", "show-urdf"], help="The subcommand to run")
    parsed_args, remaining_args = parser.parse_known_args(args)

    match parsed_args.subcommand:
        case "urdf":
            import_onshape.main(remaining_args)
        case "show-urdf":
            show_urdf.main(remaining_args)
        case _:
            raise ValueError(f"Unknown subcommand: {parsed_args.subcommand}")


if __name__ == "__main__":
    # python -m kol.scripts.cli
    main()
