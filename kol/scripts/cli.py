"""Defines the top-level KOL CLI."""

import argparse
import asyncio
from typing import Literal, Sequence, get_args

from kol.onshape.download import main as download_main
from kol.onshape.postprocess import (
    download_and_postprocess as download_and_postprocess_main,
    main as postprocess_main,
)
from kol.scripts import pybullet, visualize_stl

Subcommand = Literal[
    "run",
    "download",
    "postprocess",
    "pybullet",
    "stl",
]


async def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="K-Scale OnShape Library", add_help=False)
    parser.add_argument("subcommand", choices=get_args(Subcommand), help="The subcommand to run")
    parsed_args, remaining_args = parser.parse_known_args(args)
    subcommand: Subcommand = parsed_args.subcommand

    match subcommand:
        case "run":
            await download_and_postprocess_main(remaining_args)
        case "download":
            await download_main(remaining_args)
        case "postprocess":
            await postprocess_main(remaining_args)
        case "pybullet":
            pybullet.main(remaining_args)
        case "stl":
            visualize_stl.main(remaining_args)
        case _:
            raise ValueError(f"Unknown subcommand: {parsed_args.subcommand}")


def sync_main() -> None:
    asyncio.run(main())


if __name__ == "__main__":
    # python -m kol.scripts.cli
    sync_main()
