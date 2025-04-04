"""Defines functions for post-processing the downloaded URDF."""

import asyncio
import logging
import sys
import tarfile
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from onshape.onshape.config import ConverterConfig, PostprocessConfig
from onshape.onshape.download import download
from onshape.passes.add_base_linkage import add_base_linkage
from onshape.passes.add_joint_separation import add_joint_separation
from onshape.passes.add_mjcf import convert_urdf_to_mjcf
from onshape.passes.convert_floats_to_consistent_types import convert_floats_to_consistent_types
from onshape.passes.exclude_collision_meshes import exclude_collision_meshes
from onshape.passes.fix_inertias import fix_inertias
from onshape.passes.flip_joints import flip_joints
from onshape.passes.make_convex_collision_mesh import get_convex_collision_meshes
from onshape.passes.merge_fixed_joints import get_merged_urdf
from onshape.passes.move_collision_meshes import move_collision_meshes
from onshape.passes.remove_extra_meshes import remove_extra_meshes
from onshape.passes.remove_internal_geometries import remove_internal_geometries_from_urdf
from onshape.passes.rotate_joints import rotate_joints
from onshape.passes.separate_collision_meshes import separate_collision_meshes_in_urdf
from onshape.passes.shrink_collision_meshes import shrink_collision_meshes
from onshape.passes.simplify_meshes import get_simplified_urdf
from onshape.passes.sort_sections import sort_sections
from onshape.passes.update_names import update_urdf_names
from onshape.passes.use_collision_meshes_as_visual_meshes import use_collision_meshes_as_visual_meshes
from onshape.passes.utils import iter_meshes
from onshape.utils.logging import configure_logging


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

    # Updates the names in the URDF.
    if config.update_names:
        update_urdf_names(urdf_path, joint_name_map=config.joint_name_map, link_name_map=config.link_name_map)

    # Merges all fixed joints in the URDF.
    if config.merge_fixed_joints:
        get_merged_urdf(urdf_path, ignore_merging_fixed_joints=config.ignore_merging_fixed_joints)

    # Applies rotations to some joints.
    if config.rotate_joints is not None:
        rotate_joints(urdf_path, config.rotate_joints)

    # Creates separate collision meshes for each link in the URDF.
    if config.separate_collision_meshes:
        separate_collision_meshes_in_urdf(urdf_path)

    # Shrinks some of the collision meshes.
    if config.shrink_collision_meshes is not None:
        shrink_collision_meshes(urdf_path, config.shrink_collision_meshes)

    # Moves some of the collision meshes.
    if config.move_collision_meshes is not None:
        move_collision_meshes(urdf_path, config.move_collision_meshes)

    # Creates separate convex hulls for collision geomtries.
    if config.convex_collision_meshes:
        get_convex_collision_meshes(
            urdf_path,
            max_triangles=config.max_convex_collision_mesh_triangles,
        )

    # Simplifies the meshes in the URDF.
    if config.simplify_meshes:
        get_simplified_urdf(
            urdf_path,
            voxel_size=config.voxel_size,
            max_triangles=config.max_mesh_triangles,
        )

    # Add a small separation between adjacent joints.
    if config.joint_separation_distance is not None:
        add_joint_separation(urdf_path, config.joint_separation_distance)

    # Flips the orientation of specific joints.
    if config.flip_joints is not None:
        flip_joints(urdf_path, config.flip_joints)

    # Fixes the inertias in the URDF.
    if config.fix_inertias:
        fix_inertias(urdf_path, epsilon=config.min_inertia_eigval)

    # Converts floating point numbers to consistent types.
    if config.convert_floats_to_consistent_types:
        convert_floats_to_consistent_types(urdf_path)

    # Sorts the sections in the URDF, putting the joints at the end.
    if config.sort_sections:
        sort_sections(urdf_path)

    # Adds a base linkage.
    if config.add_base_linkage:
        add_base_linkage(urdf_path, base_rpy=config.base_rpy)

    # Remove internal triangles from the meshes in the URDF.
    if config.remove_internal_geometries:
        remove_internal_geometries_from_urdf(urdf_path)

    # Excludes collision meshes from the URDF.
    if config.exclude_collision_meshes:
        exclude_collision_meshes(urdf_path, config.exclude_collision_meshes)

    # Use collision meshes as visual meshes.
    if config.use_collision_meshes_as_visual_meshes:
        use_collision_meshes_as_visual_meshes(urdf_path)

    # Adds the MJCF XML to the package.
    paths = [urdf_path]
    if config.add_mjcf:
        for metadata in config.mjcf_metadata or []:
            if metadata.suffix is not None:
                paths.extend(
                    convert_urdf_to_mjcf(
                        urdf_file=urdf_path,
                        mjcf_file=urdf_path.with_suffix(f".{metadata.suffix}.mjcf"),
                        metadata=metadata,
                    )
                )
            else:
                paths.extend(convert_urdf_to_mjcf(urdf_path, metadata=metadata))

    if config.remove_extra_meshes:
        remove_extra_meshes(urdf_path)

    # Combines everything to a single TAR file.
    for _, (_, visual_mesh_path), (_, collision_mesh_path) in iter_meshes(urdf_path):
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
    # python -m onshape.onshape.postprocess
    sync_main()
