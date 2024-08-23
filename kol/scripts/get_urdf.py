# mypy: disable-error-code="attr-defined"
"""Defines utility functions for converting an OnShape model to a URDF file."""

import asyncio
import logging
import sys
from typing import Sequence

from kol.logging import configure_logging
from kol.onshape.converter import Converter, ConverterConfig


async def main(args: Sequence[str] | None = None) -> None:
    if args is None:
        args = sys.argv[1:]
    config = ConverterConfig.from_cli_args(args)
    configure_logging(level=logging.DEBUG if config.debug else logging.INFO)
    await Converter(config).save_urdf()


def sync_main(args: Sequence[str] | None = None) -> None:
    asyncio.run(main(args))


if __name__ == "__main__":
    sync_main()
