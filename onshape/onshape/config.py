"""Defines the config class."""

import argparse
from dataclasses import dataclass, field
from pathlib import Path
from typing import Self, Sequence, cast

from omegaconf import MISSING, OmegaConf
from omegaconf.errors import ConfigKeyError

from onshape.formats.mjcf import ConversionMetadata
from onshape.onshape.schema.assembly import Key, MatedEntity, MateType


@dataclass
class MimicRelation:
    parent: Key
    multiplier: float


@dataclass
class Joint:
    parent: Key
    child: Key
    parent_entity: MatedEntity
    child_entity: MatedEntity
    mate_type: MateType
    joint_key: Key
    lhs_is_first: bool


@dataclass
class JointLimits:
    z_min_expression: str | None = field(default=None)
    z_max_expression: str | None = field(default=None)
    axial_z_min_expression: str | None = field(default=None)
    axial_z_max_expression: str | None = field(default=None)


@dataclass
class DownloadConfig:
    document_url: str = field(
        default=MISSING,
        metadata={"help": "The URL of the OnShape document."},
    )
    use_assembly_name: bool = field(
        default=False,
        metadata={"help": "Uses the assembly name as the file name, instead of the default name."},
    )
    output_dir: str = field(
        default=MISSING,
        metadata={"help": "The output directory."},
    )
    default_prismatic_joint_effort: float = field(
        default=80.0,
        metadata={"help": "The default effort limit for prismatic joints."},
    )
    default_prismatic_joint_velocity: float = field(
        default=5.0,
        metadata={"help": "The default velocity limit for prismatic joints."},
    )
    default_revolute_joint_effort: float = field(
        default=80.0,
        metadata={"help": "The default effort limit for revolute joints, in Nm."},
    )
    default_revolute_joint_velocity: float = field(
        default=5.0,
        metadata={"help": "The default velocity limit for revolute joints, in rad/s."},
    )
    default_revolute_joint_limits: tuple[float, float] | None = field(
        default=None,
        metadata={"help": "The default joint limits to use for joints without limits."},
    )
    default_prismatic_joint_limits: tuple[float, float] | None = field(
        default=None,
        metadata={"help": "The default joint limits to use for joints without limits."},
    )
    suffix_to_joint_effort: dict[str, float] = field(
        default_factory=lambda: {},
        metadata={"help": "The suffix to joint effort mapping."},
    )
    suffix_to_joint_velocity: dict[str, float] = field(
        default_factory=lambda: {},
        metadata={"help": "The suffix to joint velocity mapping."},
    )
    invalidate_cache_after_n_minutes: float | None = field(
        default=None,
        metadata={"help": "Invalidates the cache after n minutes."},
    )
    disable_mimics: bool = field(
        default=False,
        metadata={"help": "Disables the mimic joints."},
    )
    override_central_node: str | None = field(
        default=None,
        metadata={"help": "The name of the central node to use."},
    )
    min_facet_width: float | None = field(
        default=0.002,
        metadata={"help": "The minimum facet width to use for simplifying meshes."},
    )
    mesh_dir: str = field(
        default="meshes",
        metadata={"help": "The directory to store the meshes."},
    )
    max_concurrent_requests: int = field(
        default=5,
        metadata={"help": "The maximum number of concurrent requests to make."},
    )
    api_post_wait: float = field(
        default=0.0,
        metadata={"help": "The number of seconds to wait after each API request."},
    )
    default_part_mass: float | None = field(
        default=None,
        metadata={"help": "The default mass to use for parts, if mass is missing."},
    )
    min_inertia_value: float = field(
        default=1e-6,
        metadata={"help": "The minimum value to use for the inertia matrix."},
    )
    debug: bool = field(
        default=False,
        metadata={"help": "Enables debug mode."},
    )

    @classmethod
    def from_cli_args(cls, args: Sequence[str]) -> Self:
        # First, parse the document URL and output directory.
        parser = argparse.ArgumentParser()
        parser.add_argument("document_url", help="The URL of the OnShape document.")
        parser.add_argument("-o", "--output-dir", type=str, default="robot", help="The output directory.")
        parser.add_argument("-f", "--config-path", type=Path, default=None, help="The path to the config file.")
        parser.add_argument("-n", "--no-cache", action="store_true", help="Disables caching.")
        parsed_args, remaining_args = parser.parse_known_args(args)
        document_url: str = parsed_args.document_url
        output_dir: str = parsed_args.output_dir
        config_path: Path | None = parsed_args.config_path

        # Next, parses additional config arguments.
        cfg = cast(Self, OmegaConf.structured(cls))
        cfg.document_url = document_url
        cfg.output_dir = output_dir
        if config_path is not None:
            with config_path.open("r") as f:
                file_cfg = OmegaConf.load(f)
                cfg = cast(Self, OmegaConf.merge(cfg, file_cfg))
        try:
            cli_cfg = OmegaConf.from_cli(remaining_args)
            cfg = cast(Self, OmegaConf.merge(cfg, cli_cfg))
        except ConfigKeyError as e:
            raise ValueError(
                "You tried to supply config override arguments that are not valid for this config. If you meant to "
                "specify the output directory, use `-o <output_dir>` or `--output-dir <output_dir>` instead."
            ) from e
        if cfg.invalidate_cache_after_n_minutes is None and parsed_args.no_cache:
            cfg.invalidate_cache_after_n_minutes = 0.0
        return cfg


@dataclass
class PostprocessConfig:
    update_names: bool = field(
        default=True,
        metadata={"help": "Updates the names in the URDF."},
    )
    joint_name_map: dict[str, str] = field(
        default_factory=lambda: {},
        metadata={"help": "The mapping from old joint names to new joint names."},
    )
    exclude_collision_meshes: list[str] = field(
        default_factory=lambda: [],
        metadata={"help": "The names of the collision meshes to exclude."},
    )
    link_name_map: dict[str, str] = field(
        default_factory=lambda: {},
        metadata={"help": "The mapping from old link names to new link names."},
    )
    urdf_path: str = field(
        default=MISSING,
        metadata={"help": "The path to the downloaded URDF."},
    )
    merge_fixed_joints: bool = field(
        default=True,
        metadata={"help": "Merges fixed joints in the URDF."},
    )
    rotate_joints: dict[str, list[float]] | None = field(
        default=None,
        metadata={"help": "The joints to rotate, and the RPY values to rotate them by."},
    )
    ignore_merging_fixed_joints: list[str] = field(
        default_factory=lambda: [],
        metadata={"help": "The names of the fixed joints to ignore when merging (for example, IMU joints)."},
    )
    simplify_meshes: bool = field(
        default=True,
        metadata={"help": "Simplifies the meshes when converting the URDF."},
    )
    remove_internal_geometries: bool = field(
        default=True,
        metadata={"help": "Remove the internal geometries from meshes as a clean-up step."},
    )
    remove_extra_meshes: bool = field(
        default=True,
        metadata={"help": "Removes any extra mesh files in the URDF directory."},
    )
    joint_separation_distance: float | None = field(
        default=None,
        metadata={"help": "If set, separate adjacent joints by this distance, to prevent collisions."},
    )
    add_base_linkage: bool = field(
        default=True,
        metadata={"help": "Adds a base linkage to the URDF."},
    )
    base_rpy: tuple[float, float, float] = field(
        default=(0.0, 0.0, 0.0),
        metadata={"help": "The RPY to apply to the base linkage to orient the robot."},
    )
    voxel_size: float = field(
        default=0.00002,
        metadata={"help": "The voxel size to use for simplifying meshes."},
    )
    convex_collision_meshes: bool = field(
        default=False,
        metadata={"help": "Creates separate convex hulls for collision geometries."},
    )
    max_mesh_triangles: int | None = field(
        default=None,
        metadata={"help": "The maximum number of triangles to use for meshes."},
    )
    max_convex_collision_mesh_triangles: int | None = field(
        default=None,
        metadata={"help": "The maximum number of triangles to use for convex collision meshes."},
    )
    separate_collision_meshes: bool = field(
        default=False,
        metadata={"help": "Creates separate collision meshes for each link in the URDF."},
    )
    shrink_collision_meshes: dict[str, float] | None = field(
        default=None,
        metadata={"help": "Shrinks the named collision meshes by this factor."},
    )
    move_collision_meshes: dict[str, list[float]] | None = field(
        default=None,
        metadata={"help": "Moves the named collision meshes by this offset."},
    )
    use_collision_meshes_as_visual_meshes: bool = field(
        default=False,
        metadata={"help": "Uses collision meshes as visual meshes - mainly used for debugging."},
    )
    fix_inertias: bool = field(
        default=True,
        metadata={"help": "Ensures that the inertias are positive definite."},
    )
    convert_floats_to_consistent_types: bool = field(
        default=True,
        metadata={"help": "Converts floating point numbers to consistent types."},
    )
    min_inertia_eigval: float = field(
        default=1e-7,
        metadata={"help": "The minimum value to use for the inertia matrix."},
    )
    flip_joints: list[str] | None = field(
        default=None,
        metadata={"help": "The names of the joints to flip."},
    )
    sort_sections: bool = field(
        default=True,
        metadata={"help": "Sorts the sections in the URDF, putting the joints at the end."},
    )
    add_mjcf: bool = field(
        default=True,
        metadata={"help": "Adds the MJCF XML to the package."},
    )
    mjcf_metadata: list[ConversionMetadata] | None = field(
        default=None,
        metadata={"help": "The MJCF metadata(s) to use for the URDF."},
    )
    package_tgz: bool = field(
        default=True,
        metadata={"help": "Packages the URDF into a .tgz file."},
    )
    debug: bool = field(
        default=False,
        metadata={"help": "Enables debug mode."},
    )

    @classmethod
    def from_cli_args(cls, args: Sequence[str]) -> Self:
        # First, parse the URDF path.
        parser = argparse.ArgumentParser()
        parser.add_argument("urdf_path", help="The path to the downloaded URDF.")
        parser.add_argument("-f", "--config-path", type=Path, default=None, help="The path to the config file.")
        parsed_args, remaining_args = parser.parse_known_args(args)
        urdf_path: str = parsed_args.urdf_path
        config_path: Path | None = parsed_args.config_path

        # Next, parses additional config arguments.
        cfg = cast(Self, OmegaConf.structured(cls))
        cfg.urdf_path = urdf_path
        if config_path is not None:
            with config_path.open("r") as f:
                file_cfg = OmegaConf.load(f)
                cfg = cast(Self, OmegaConf.merge(cfg, file_cfg))
        try:
            cli_cfg = OmegaConf.from_cli(remaining_args)
            cfg = cast(Self, OmegaConf.merge(cfg, cli_cfg))
        except ConfigKeyError as e:
            raise ValueError(
                "You tried to supply config override arguments that are not valid for this config. If you meant to "
                "specify the output directory, use `-o <output_dir>` or `--output-dir <output_dir>` instead."
            ) from e
        return cfg


@dataclass
class ConverterConfig(DownloadConfig, PostprocessConfig):
    @classmethod
    def from_cli_args(cls, args: Sequence[str]) -> Self:
        # First, parse the document URL and output directory.
        parser = argparse.ArgumentParser()
        parser.add_argument("document_url", help="The URL of the OnShape document.")
        parser.add_argument("-o", "--output-dir", type=str, default="robot", help="The output directory.")
        parser.add_argument("-f", "--config-path", type=Path, default=None, help="The path to the config file.")
        parser.add_argument("-n", "--no-cache", action="store_true", help="Disables caching.")
        parsed_args, remaining_args = parser.parse_known_args(args)
        document_url: str = parsed_args.document_url
        output_dir: str = parsed_args.output_dir
        config_path: Path | None = parsed_args.config_path

        # Next, parses additional config arguments.
        cfg = cast(Self, OmegaConf.structured(cls))
        cfg.document_url = document_url
        cfg.output_dir = output_dir
        if config_path is not None:
            with config_path.open("r") as f:
                file_cfg = OmegaConf.load(f)
                cfg = cast(Self, OmegaConf.merge(cfg, file_cfg))
        try:
            cli_cfg = OmegaConf.from_cli(remaining_args)
            cfg = cast(Self, OmegaConf.merge(cfg, cli_cfg))
        except ConfigKeyError as e:
            raise ValueError(
                "You tried to supply config override arguments that are not valid for this config. If you meant to "
                "specify the output directory, use `-o <output_dir>` or `--output-dir <output_dir>` instead."
            ) from e
        if cfg.invalidate_cache_after_n_minutes is None and parsed_args.no_cache:
            cfg.invalidate_cache_after_n_minutes = 0.0
        return cfg
