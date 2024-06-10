"""Defines functions for merging urdf parts at fixed joints."""

import datetime
import functools
import hashlib
import io
import itertools
import logging
import re
import uuid
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Deque, Iterator, Literal, TypeVar

import networkx as nx
import numpy as np
import stl
from scipy.spatial.transform import Rotation as R

from kol.formats import mjcf, urdf
from kol.geometry import (
    Dynamics,
    apply_matrix_,
    combine_dynamics,
    combine_meshes,
    get_mesh_convex_hull,
    inv_tf,
    matrix_to_moments,
    process_key_name,
    scale_mesh,
    transform_inertia_tensor,
)
from kol.mesh import MeshType, load_file, stl_to_fmt
from kol.onshape.api import OnshapeApi
from kol.onshape.client import OnshapeClient
from kol.onshape.schema.assembly import (
    Assembly,
    AssemblyInstance,
    AssemblyMetadata,
    Instance,
    Key,
    MatedEntity,
    MateFeature,
    MateRelationFeature,
    MateType,
    Occurrence,
    Part,
    PartInstance,
    RelationType,
    RootAssembly,
    SubAssembly,
)
from kol.onshape.schema.common import ElementUid
from kol.onshape.schema.features import Feature, Features
from kol.onshape.schema.part import PartDynamics, PartMetadata
from kol.resolvers import ExpressionResolver


def process_fixed_joints(self, urdf_parts: list[urdf.Link | urdf.BaseJoint]) -> list[urdf.Link | urdf.BaseJoint]:
    """Processes the fixed joints in the assembly."""
    merged_central = False
    # While there still exists fixed joints, fuse the parts they connect.
    while any(j.mate_type == MateType.FASTENED for j in self.ordered_joint_list):
        # Get first fixed joint from joint list.
        joint = next(j for j in self.ordered_joint_list if j.mate_type == MateType.FASTENED)
        logger.info("Found fixed joint.")
        # Get parent and child of fixed joint
        parent = joint.parent
        child = joint.child
        collected_parts = [parent, child]
        # Get relative transform between the two joints
        relative_transform = inv_tf(joint.child_entity.matedCS.part_to_mate_tf)
        logger.info("Fusing parts: %s.", ", ".join(process_key_name(self.key_name(p, None)) for p in collected_parts))
        new_part_name = "fused_" + "_".join(process_key_name(self.key_name(p, None)) for p in collected_parts)
        if len(new_part_name) > 50:
            new_part_name = new_part_name[:46] + "..."
        # Calculate new stl.
        combined_stl_file_name = f"{new_part_name}.stl"
        combined_stl_file_path = self.mesh_dir / combined_stl_file_name
        combined_stl_file_path.unlink(missing_ok=True)
        parent_meshpath = f"{process_key_name(self.key_name(parent, None))}.stl"
        child_meshpath = f"{process_key_name(self.key_name(child, None))}.stl"
        parent_mesh = load_file(self.mesh_dir / parent_meshpath)
        child_mesh = load_file(self.mesh_dir / child_meshpath)

        # Combine the meshes using the relative origins
        combined_mesh = combine_meshes(parent_mesh, child_mesh, relative_transform)
        for point in combined_mesh.points:
            if len(point) != 3:
                raise ValueError("Invalid point in combined mesh.", point)
        for face in combined_mesh.faces:
            if len(face) != 3:
                raise ValueError("Invalid face in combined mesh.", face)
        combined_mesh.save(combined_stl_file_path)
        # Get convex hull of combined mesh, and scale to good size.
        combined_collision = get_mesh_convex_hull(combined_mesh)
        combined_collision = scale_mesh(combined_mesh, 0.6)

        # Save collision mesh to filepath
        combined_collision_stl_name = f"{new_part_name}_collision.stl"
        collision_stl_file_path = self.mesh_dir / combined_collision_stl_name
        combined_collision.save(collision_stl_file_path)

        # Get combined dynamic mesh
        part_instances = [self.key_to_part_instance[p] for p in collected_parts]
        parts = [self.euid_to_part[p.euid] for p in part_instances]
        part_dynamics = [self.part_dynamics(p) for p in parts]
        part_bodies = [d.bodies[p.partId] for d, p in zip(part_dynamics, part_instances)]
        true_dynamics = [Dynamics(mass=p.mass[0], com=p.center_of_mass, inertia=p.inertia_matrix) for p in part_bodies]
        combined_dynamics = combine_dynamics(true_dynamics)

        # Get combined visual mesh
        combined_visual = urdf.VisualLink(
            origin=urdf.Origin.zero_origin(),
            geometry=urdf.MeshGeometry(filename=f"./meshes/{combined_stl_file_name}"),
            material=urdf.Material(
                name=f"{new_part_name}_material",
                color=[0, 0, 0],
            ),
        )

        # Get combined collision mesh
        combined_collision = urdf.CollisionLink(
            origin=urdf.Origin.zero_origin(),
            geometry=urdf.MeshGeometry(filename=f"./meshes/{combined_collision_stl_name}"),
        )
        logger.info("Got combined meshes and dynamics.")

        # Add the fused part to the URDF.
        fused_part = urdf.Link(
            name=new_part_name,
            visual=combined_visual,
            inertial=urdf.InertialLink(
                origin=urdf.Origin.zero_origin(),
                mass=combined_dynamics.mass,
                inertia=matrix_to_moments(combined_dynamics.inertia),
            ),
            collision=combined_collision,
        )
        new_euid = uuid.uuid4().hex
        print(new_euid)
        urdf_parts.append(fused_part)
        fused_part_instance = PartInstance(
            name=new_part_name,
            suppressed=False,
            id=new_euid,
            fullConfiguration="default",
            configuration="default",
            documentMicroversion="",
            documentId="",
            elementId="",
            type="Part",
            isStandardContent=False,
            partId="JHD",
        )
        # Logging for debugging
        logger.debug(f"Registering fused_part_instance: {fused_part_instance}")
        logger.debug(f"Registering fused_part: {fused_part}")

        # Register the new part
        self.key_to_instance[new_part_name] = fused_part_instance
        # Get part version of the new part
        fused_part_part = Part(
            isStandardContent=False,
            partId=new_part_name,
            bodyType="solid",
            fullConfiguration="default",
            configuration="default",
            documentMicroversion="microversion_001",
            documentId="doc_001",
            elementId=new_euid,
        )
        self.euid_to_part[fused_part_instance.euid] = fused_part_part

        # Origin of new part is origin of parent link
        self.stl_origin_to_part_tf[new_part_name] = joint.parent_entity.matedCS.part_to_mate_tf
        self.ordered_joint_list.remove(joint)
        # Rename joints involving any of the connected parts to the new part.
        for j in self.ordered_joint_list:
            if j.parent in collected_parts:
                j.parent = new_part_name
            if j.child in collected_parts:
                j.child = new_part_name
        # If we merged away central node, we add a flag.
        if self.central_node in collected_parts:
            merged_central = True
        logger.info("Successfully fused joint.")
    return urdf_parts, merged_central


def get_merged_urdf(
    urdf: Path,
    scaling: float,
) -> Path:
    """Merges meshes at each fixed joints to avoid collision issues.

    Args:
        path: The path to the mesh file.
        scaling: The scaling factor to apply to the meshes.

    Returns:
        The path to the merged urdf file.
    """
    if scaling < 0:
        raise ValueError(f"Scaling {scaling} should be greater than 0.")

    return urdf
