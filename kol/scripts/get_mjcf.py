# mypy: disable-error-code="attr-defined"
"""Defines utility functions for converting an OnShape model to a MJCF file."""

import argparse
import logging
from typing import Sequence

from kol.logging import configure_logging
from kol.onshape.converter import Converter


def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Import the robot model from OnShape")
    parser.add_argument("document_url", type=str, help="The ID of the document to import")
    parser.add_argument("-o", "--output-dir", type=str, help="The path to save the imported model")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    parsed_args = parser.parse_args(args)

    configure_logging(level=logging.DEBUG if parsed_args.debug else logging.INFO)

    Converter(
        document_url=parsed_args.document_url,
        output_dir=parsed_args.output_dir,
    ).save_mjcf()


if __name__ == "__main__":
    # python -m kol.scripts.get_mjcf
    main()
