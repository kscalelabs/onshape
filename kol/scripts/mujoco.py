# mypy: disable-error-code="import-not-found"
"""Simple script to interact with a URDF in MuJoCo."""

import logging
from typing import Sequence

from kol.logging import configure_logging

logger = logging.getLogger(__name__)


def main(args: Sequence[str] | None = None) -> None:
    configure_logging()

    raise NotImplementedError


if __name__ == "__main__":
    # python -m robot.cad.scripts.mujoco
    main()
