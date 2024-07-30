# mypy: disable-error-code="attr-defined"
"""Defines utility functions for converting an OnShape model to a URDF file."""

import logging
import sys
from typing import Sequence

from kol.logging import configure_logging
from kol.onshape.converter import Converter, ConverterConfig


def main(args: Sequence[str] | None = None) -> None:
    if args is None:
        args = sys.argv[1:]
    config = ConverterConfig.from_cli_args(args)
    configure_logging(level=logging.DEBUG if config.debug else logging.INFO)
    Converter(config).save_urdf()


if __name__ == "__main__":
    main()
