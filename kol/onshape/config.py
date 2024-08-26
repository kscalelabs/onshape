"""Defines the config class."""

import argparse
from dataclasses import dataclass, field
from typing import Self, Sequence, cast

import numpy as np
from omegaconf import MISSING, OmegaConf

from kol.onshape.schema.assembly import Key, MatedEntity, MateType


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
    z_min_expression: str | None = None
    z_max_expression: str | None = None
    axial_z_min_expression: str | None = None
    axial_z_max_expression: str | None = None


@dataclass
class DownloadConfig:
    document_url: str = field(
        default=MISSING,
        metadata={"help": "The URL of the OnShape document."},
    )
    output_dir: str = field(
        default=MISSING,
        metadata={"help": "The output directory."},
    )
    default_prismatic_joint_limits: tuple[float, float, float, float] = field(
        default=(80.0, 5.0, -1.0, 1.0),
        metadata={"help": "The default prismatic joint limits, as (effort, velocity, min, max)."},
    )
    default_revolute_joint_limits: tuple[float, float, float, float] = field(
        default=(80.0, 5.0, -np.pi, np.pi),
        metadata={"help": "The default revolute joint limits, as (effort, velocity, min, max)."},
    )
    suffix_to_joint_effort: dict[str, float] = field(
        default_factory=lambda: {},
        metadata={"help": "The suffix to joint effort mapping."},
    )
    suffix_to_joint_velocity: dict[str, float] = field(
        default_factory=lambda: {},
        metadata={"help": "The suffix to joint velocity mapping."},
    )
    invalidate_cache_after_n_minutes: int | None = field(
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
    debug: bool = field(
        default=False,
        metadata={"help": "Enables debug mode."},
    )

    @classmethod
    def from_cli_args(cls, args: Sequence[str]) -> Self:
        # First, parse the document URL and output directory.
        parser = argparse.ArgumentParser()
        parser.add_argument("document_url", help="The URL of the OnShape document.")
        parser.add_argument("output_dir", help="The output directory.")
        parsed_args, remaining_args = parser.parse_known_args(args)
        document_url: str = parsed_args.document_url
        output_dir: str = parsed_args.output_dir

        # Next, parses additional config arguments.
        cfg = cast(Self, OmegaConf.structured(cls))
        cfg.document_url = document_url
        cfg.output_dir = output_dir
        cli_cfg = OmegaConf.from_cli(remaining_args)
        cfg = cast(Self, OmegaConf.merge(cfg, cli_cfg))
        return cfg


@dataclass
class PostprocessConfig:
    urdf_path: str = field(
        default=MISSING,
        metadata={"help": "The path to the downloaded URDF."},
    )
    merge_fixed_joints: bool = field(
        default=True,
        metadata={"help": "Merges fixed joints in the URDF."},
    )
    simplify_meshes: bool = field(
        default=True,
        metadata={"help": "Simplifies the meshes when converting the URDF."},
    )
    voxel_size: float = field(
        default=0.002,
        metadata={"help": "The voxel size to use for simplifying meshes."},
    )
    convex_collision_meshes: bool = field(
        default=True,
        metadata={"help": "Creates separate convex hulls for collision geometries."},
    )
    add_mjcf: bool = field(
        default=True,
        metadata={"help": "Adds the MJCF XML to the package."},
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
        parsed_args, remaining_args = parser.parse_known_args(args)
        urdf_path: str = parsed_args.urdf_path

        # Next, parses additional config arguments.
        cfg = cast(Self, OmegaConf.structured(cls))
        cfg.urdf_path = urdf_path
        cli_cfg = OmegaConf.from_cli(remaining_args)
        cfg = cast(Self, OmegaConf.merge(cfg, cli_cfg))
        return cfg


@dataclass
class ConverterConfig(DownloadConfig, PostprocessConfig):
    @classmethod
    def from_cli_args(cls, args: Sequence[str]) -> Self:
        # First, parse the document URL and output directory.
        parser = argparse.ArgumentParser()
        parser.add_argument("document_url", help="The URL of the OnShape document.")
        parser.add_argument("output_dir", help="The output directory.")
        parsed_args, remaining_args = parser.parse_known_args(args)
        document_url: str = parsed_args.document_url
        output_dir: str = parsed_args.output_dir

        # Next, parses additional config arguments.
        cfg = cast(Self, OmegaConf.structured(cls))
        cfg.document_url = document_url
        cfg.output_dir = output_dir
        cli_cfg = OmegaConf.from_cli(remaining_args)
        cfg = cast(Self, OmegaConf.merge(cfg, cli_cfg))
        return cfg
