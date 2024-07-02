"""Removes all meshes not mentioned in a urdf from the ./meshes directory."""

import argparse
import logging
from pathlib import Path
from typing import Sequence

from kol.cleanup import cleanup_mesh_dir

logger = logging.getLogger(__name__)


def main(args: Sequence[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser(description="Cleanup meshes directory")
    parser.add_argument("filepath", type=str, help="The path to the urdf file")
    parsed_args = parser.parse_args(args)

    filepath = Path(parsed_args.filepath)
    cleanup_mesh_dir(filepath)
