"""Defines functions for post-processing the downloaded URDF."""

import asyncio
import logging
import sys
import tarfile
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from kol.onshape.config import ConverterConfig, PostprocessConfig
from kol.onshape.download import download
from kol.passes.add_mjcf import convert_urdf_to_mjcf
from kol.passes.fix_inertias import fix_inertias
from kol.passes.make_convex_collision_mesh import get_convex_collision_meshes
from kol.passes.merge_fixed_joints import get_merged_urdf
from kol.passes.prepend_root_link import prepend_root_link
from kol.passes.remove_collision_meshes import remove_collision_meshes
from kol.passes.simplify_meshes import get_simplified_urdf
from kol.passes.update_names import update_urdf_names
from kol.passes.utils import iter_meshes
from kol.utils.logging import configure_logging


@dataclass
class PostprocessedDocument:
    urdf_path: Path
    tar_path: Path


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

    # Simplifies the meshes in the URDF.
    if config.simplify_meshes:
        get_simplified_urdf(urdf_path, voxel_size=config.voxel_size)

    # Prepends the root link to the URDF.
    if config.prepend_root_link:
        prepend_root_link(urdf_path)

    # Updates the names in the URDF.
    if config.update_names:
        update_urdf_names(urdf_path, joint_name_map=config.joint_name_map, link_name_map=config.link_name_map)

    # Creates separate convex hulls for collision geomtries.
    if config.convex_collision_meshes:
        get_convex_collision_meshes(urdf_path)

    # Removes collision meshes.
    if config.remove_collision_meshes:
        remove_collision_meshes(urdf_path)

    # Fixes the inertias in the URDF.
    if config.fix_inertias:
        fix_inertias(urdf_path, epsilon=config.min_inertia_eigval)

    # Adds the MJCF XML to the package.
    paths = [urdf_path]
    if config.add_mjcf:
        mjcf_path = urdf_path.with_suffix(".mjcf")
        convert_urdf_to_mjcf(urdf_path, mjcf_path)
        paths.append(mjcf_path)

    # Combines everything to a single TAR file.
    for (_, visual_mesh_path), (_, collision_mesh_path) in iter_meshes(urdf_path, config.remove_collision_meshes):
        for path in list({visual_mesh_path, collision_mesh_path}):
            if path is not None:
                paths.append(path)

    tar_path = urdf_path.with_suffix(".tgz")
    with tarfile.open(tar_path, "w:gz") as tar:
        for path in paths:
            arcname = path.relative_to(urdf_path.parent).as_posix()
            tar.add(path, arcname=arcname)

    return PostprocessedDocument(
        urdf_path=urdf_path,
        tar_path=tar_path,
    )


async def download_and_postprocess_main(args: Sequence[str] | None = None) -> PostprocessedDocument:
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


async def postprocess_main(args: Sequence[str] | None = None) -> PostprocessedDocument:
    if args is None:
        args = sys.argv[1:]
    config = PostprocessConfig.from_cli_args(args)
    configure_logging(level=logging.DEBUG if config.debug else logging.INFO)
    return await postprocess(
        urdf_path=config.urdf_path,
        config=config,
    )


def sync_main(args: Sequence[str] | None = None) -> None:
    asyncio.run(postprocess_main(args))


if __name__ == "__main__":
    # python -m kol.onshape.postprocess
    sync_main()
