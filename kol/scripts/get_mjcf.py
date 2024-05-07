# mypy: disable-error-code="attr-defined"
"""Defines utility functions for converting an OnShape model to a MJCF file.
python kol/scripts/get_mjcf.py https://cad.onshape.com/documents/655c1b96d8d37f817d03ec70/w/2b9c1d4805ca57134ac438be/e/9d91baa008a881d1993de549

python kol/scripts/show_mjcf.py


 1. Collision rules are defined automatically.
 2. Geom without pos takes 0 0 0 and 1 0 0 0 as default
 3. he quat attribute has a default value corresponding to the null rotation
 4.

"""
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
