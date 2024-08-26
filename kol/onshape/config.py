"""Defines the config class."""

import argparse
from dataclasses import dataclass, field
from pathlib import Path
from typing import Self, Sequence

import numpy as np
from omegaconf import OmegaConf

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
class ConverterConfig:
    """Defines a configuration class for the converter."""

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
    disable_mimics: bool = field(
        default=False,
        metadata={"help": "Disables the mimic joints."},
    )
    mesh_ext: str = field(
        default="stl",
        metadata={"help": "The mesh extension to use for the URDF files, e.g., 'stl' or 'obj'."},
    )
    override_central_node: str | None = field(
        default=None,
        metadata={"help": "The name of the central node to use."},
    )
    remove_inertia: bool = field(
        default=False,
        metadata={"help": "Removes inertia from the URDF."},
    )
    merge_fixed_joints: bool = field(
        default=False,
        metadata={"help": "Merges fixed joints in the URDF."},
    )
    simplify_meshes: bool = field(
        default=True,
        metadata={"help": "Simplifies the meshes when converting the URDF."},
    )
    override_joint_names: dict[str, str] | None = field(
        default=None,
        metadata={"help": "A mapping from an OnShape joint name to some canonical name."},
    )
    override_nonfixed: list[str] | None = field(
        default=None,
        metadata={"help": "The override non-fixed joints."},
    )
    override_limits: dict[str, str] | None = field(
        default=None,
        metadata={"help": "The override joint limits."},
    )
    override_torques: dict[str, int] | None = field(
        default=None,
        metadata={"help": "The override joint torques."},
    )
    sim_ignore_key: str = field(
        default="sim_ignore",
        metadata={"help": "If this key is in the joint name, ignore it."},
    )
    voxel_size: float = field(
        default=0.002,
        metadata={"help": "The voxel size to use for simplifying meshes."},
    )
    mesh_dir: str = field(
        default="meshes",
        metadata={"help": "The directory to store the meshes."},
    )
    invalidate_cache_after_n_minutes: int | None = field(
        default=None,
        metadata={"help": "Invalidates the cache after n minutes."},
    )
    debug: bool = field(
        default=False,
        metadata={"help": "Enables debug mode."},
    )

    @classmethod
    def from_cli_args(cls, args: Sequence[str]) -> tuple[str, str, Self]:
        # First, parse the document URL and output directory.
        parser = argparse.ArgumentParser()
        parser.add_argument("document_url", help="The URL of the OnShape document.")
        parser.add_argument("output_dir", help="The output directory.")
        parsed_args, remaining_args = parser.parse_known_args(args)
        document_url: str = parsed_args.document_url
        output_dir: str = parsed_args.output_dir

        # Next, parses additional config arguments.
        cfg = OmegaConf.structured(cls)
        cli_args: list[str] = []
        for arg in remaining_args:
            if Path(arg).is_file():
                cfg = OmegaConf.merge(cfg, OmegaConf.load(arg))
            else:
                cli_args.append(arg)
        if cli_args:
            # This is a patch to support some syntax sugar - we automatically
            # treat the first argument as the document URL.
            if "=" not in cli_args[0]:
                cli_args[0] = f"document_url={cli_args[0]}"
            cfg = OmegaConf.merge(cfg, OmegaConf.from_cli(cli_args))
        return document_url, output_dir, cfg
