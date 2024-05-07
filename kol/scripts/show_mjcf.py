# mypy: disable-error-code="import-not-found"
"""Simple script to interact with a URDF in MuJoCo."""
import argparse
import logging
from typing import Sequence

from kol.logging import configure_logging

logger = logging.getLogger(__name__)


def main(args: Sequence[str] | None = None) -> None:
    configure_logging()
    parser = argparse.ArgumentParser(description="Show a MJCF in Mujoco")
    parser.add_argument("mjcf", nargs="?", help="Path to the MJCF file")
    raise NotImplementedError


if __name__ == "__main__":
    # python -m kol.scripts.show_mjcf
    main()
