"""Defines functions for post-processing the downloaded URDF."""

import asyncio
import logging
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from kol.onshape.config import ConverterConfig, PostprocessConfig
from kol.onshape.download import download
from kol.utils.logging import configure_logging
from kol.utils.merge_fixed_joints import get_merged_urdf


@dataclass
class PostprocessedDocument:
    urdf_path: Path


async def postprocess(
    urdf_path: str | Path,
    *,
    config: PostprocessConfig | None = None,
) -> PostprocessedDocument:
    """Combines multiple post-processing steps into a single function.

    Args:
        urdf_path: The path to the downloaded URDF to post-process.
        config: The post-processing configuration.

    Returns:
        The post-processed document.
    """
    urdf_path = Path(urdf_path)
    if config is None:
        config = PostprocessConfig()

    # Merges all fixed joints in the URDF.
    if config.merge_fixed_joints:
        get_merged_urdf(urdf_path)

    return PostprocessedDocument(
        urdf_path=urdf_path,
    )


async def download_and_postprocess(args: Sequence[str] | None = None) -> PostprocessedDocument:
    if args is None:
        args = sys.argv[1:]
    config = ConverterConfig.from_cli_args(args)
    configure_logging(level=logging.DEBUG if config.debug else logging.INFO)
    document_info = await download(
        document_url=config.document_url,
        output_dir=config.output_dir,
        config=config,
    )
    config.urdf_path = document_info.urdf_info.urdf_path.as_posix()
    return await postprocess(
        urdf_path=document_info.urdf_info.urdf_path,
        config=config,
    )


async def main(args: Sequence[str] | None = None) -> PostprocessedDocument:
    if args is None:
        args = sys.argv[1:]
    config = PostprocessConfig.from_cli_args(args)
    configure_logging(level=logging.DEBUG if config.debug else logging.INFO)
    return await postprocess(
        urdf_path=config.urdf_path,
        config=config,
    )


def sync_main(args: Sequence[str] | None = None) -> None:
    asyncio.run(main(args))


if __name__ == "__main__":
    # python -m kol.onshape.postprocess
    sync_main()
