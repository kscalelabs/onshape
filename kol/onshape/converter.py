# mypy: disable-error-code="attr-defined"
"""Defines shared functions."""

import datetime
import functools
import hashlib
import io
import itertools
import logging
import re
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Deque, Iterator, Literal, TypeVar

import networkx as nx
import numpy as np
import stl

from kol import urdf
from kol.geometry import apply_matrix_, inv_tf, rotation_matrix_to_euler_angles
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

logger = logging.getLogger(__name__)

T = TypeVar("T")

Color = tuple[int, int, int, int]

DEFAULT_COLOR: Color = (0, 0, 255, 255)  # Blue


def clean_name(name: str) -> str:
    return re.sub(r"\s+", "_", re.sub(r"[<>]", "", name)).lower()


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


@dataclass
class JointLimits:
    z_min_expression: str | None = None
    z_max_expression: str | None = None
    axial_z_min_expression: str | None = None
    axial_z_max_expression: str | None = None


class Converter:
    """Defines a utility class for getting document components efficiently.

    Parameters:
        document_url: The OnShape URL of the document to import.
        output_dir: The directory to save the imported model.
        api: The OnShape API to use for importing the model.
        default_prismatic_joint_limits: The default limits for prismatic joints.
        default_revolute_joint_limits: The default limits for revolute joints.
        suffix_to_joint_effort: The mapping from joint suffix to effort. This
            is used to override the default joint effort limits by matching
            the suffix of the joint name.
        suffix_to_joint_velocity: The mapping from joint suffix to velocity.
            This is used to override the default joint velocity limits by
            matching the suffix of the joint name.
    """

    def __init__(
        self,
        document_url: str,
        output_dir: str | Path | None = None,
        api: OnshapeApi | None = None,
        default_prismatic_joint_limits: urdf.JointLimits = urdf.JointLimits(80.0, 5.0, -1.0, 1.0),
        default_revolute_joint_limits: urdf.JointLimits = urdf.JointLimits(80.0, 5.0, -np.pi, np.pi),
        suffix_to_joint_effort: list[tuple[str, float]] = [],
        suffix_to_joint_velocity: list[tuple[str, float]] = [],
    ) -> None:
        super().__init__()

        # Gets a default output directory.
        self.output_dir = Path.cwd() / "robot" if output_dir is None else Path(output_dir)

        # Creates a new directory for cached artifacts.
        self.cache_dir = self.output_dir / ".cache"
        self.cache_dir.mkdir(parents=True, exist_ok=True)

        # Creates a new directory for meshes.
        self.mesh_dir = self.output_dir / "meshes"
        self.mesh_dir.mkdir(parents=True, exist_ok=True)

        self.api = OnshapeApi(OnshapeClient()) if api is None else api
        self.document = self.api.parse_url(document_url)
        self.default_prismatic_joint_limits = default_prismatic_joint_limits
        self.default_revolute_joint_limits = default_revolute_joint_limits
        self.suffix_to_joint_effort = [(k.lower().strip(), v) for k, v in suffix_to_joint_effort]
        self.suffix_to_joint_velocity = [(k.lower().strip(), v) for k, v in suffix_to_joint_velocity]

        # Map containing all cached items.
        self.cache_map: dict[str, Any] = {}

        # Map containing the transformations from the STL origin to the part frame.
        self.stl_origin_to_part_tf: dict[Key, np.matrix] = {}

    def get_or_use_cached(
        self,
        cache_key: str,
        get_fn: Callable[[], T],
        from_json_fn: Callable[[str], T],
        to_json_fn: Callable[[T], str],
        invalidate_after_n_minutes: int | None = None,
    ) -> T:
        """Gets an item by calling an API or by retrieving it from the cache.

        Args:
            cache_dir: The directory to store the cache.
            cache_key: The key to use for the cache.
            get_fn: The function to call to get the item.
            from_json_fn: The function to call to parse the item from JSON.
            to_json_fn: The function to call to serialize the item to JSON.
            invalidate_after_n_minutes: The number of minutes after which the cache should be invalidated.

        Returns:
            The item.
        """
        if cache_key in self.cache_map:
            return self.cache_map[cache_key]
        cache_path = self.cache_dir / f"{cache_key}.json"
        if cache_path.exists():
            if invalidate_after_n_minutes is not None:
                modified_time = datetime.datetime.fromtimestamp(cache_path.stat().st_mtime)
                if (datetime.datetime.now() - modified_time).seconds > invalidate_after_n_minutes * 60:
                    logger.warning("Cache invalidated after %d minutes", invalidate_after_n_minutes)
                    cache_path.unlink()
            else:
                with open(cache_path) as f:
                    return from_json_fn(f.read())
        item = get_fn()
        with open(cache_path, "w") as f:
            f.write(to_json_fn(item))
        self.cache_map[cache_key] = item
        return item

    @property
    def assembly(self) -> Assembly:
        return self.get_or_use_cached(
            "assembly",
            lambda: self.api.get_assembly(self.document),
            lambda asm_str: Assembly.model_validate_json(asm_str),
            lambda asm: asm.model_dump_json(indent=2),
            invalidate_after_n_minutes=1,
        )

    @property
    def assembly_metadata(self) -> AssemblyMetadata:
        return self.get_or_use_cached(
            "assembly_metadata",
            lambda: self.api.get_assembly_metadata(self.assembly.rootAssembly),
            lambda asm_str: AssemblyMetadata.model_validate_json(asm_str),
            lambda asm: asm.model_dump_json(indent=2),
        )

    def part_metadata(self, part: Part) -> PartMetadata:
        return self.get_or_use_cached(
            f"part_{part.key.unique_id}_metadata",
            lambda: self.api.get_part_metadata(part),
            lambda part_str: PartMetadata.model_validate_json(part_str),
            lambda part: part.model_dump_json(indent=2),
        )

    def part_color(self, part: Part) -> Color:
        part_color = self.part_metadata(part).property_map.get("Appearance")
        if part_color is None:
            return DEFAULT_COLOR
        return (
            part_color["color"]["red"],
            part_color["color"]["green"],
            part_color["color"]["blue"],
            # part_color["opacity"],
            # TODO: Revert back later.
            128,
        )

    def part_dynamics(self, part: Part) -> PartDynamics:
        return self.get_or_use_cached(
            f"part_{part.key.unique_id}_mass_properties",
            lambda: self.api.get_part_mass_properties(part),
            lambda props_str: PartDynamics.model_validate_json(props_str),
            lambda props: props.model_dump_json(indent=2),
        )

    def part_name(self, part: Part) -> str:
        part_prop_name = self.part_metadata(part).property_map.get("Name")
        return clean_name(part_prop_name if isinstance(part_prop_name, str) else part.key.unique_id)

    @functools.cached_property
    def euid_to_assembly(self) -> dict[ElementUid, RootAssembly | SubAssembly]:
        assemblies: list[RootAssembly | SubAssembly] = [self.assembly.rootAssembly, *self.assembly.subAssemblies]
        return {assembly.key: assembly for assembly in assemblies}

    @functools.cached_property
    def euid_to_subassembly(self) -> dict[ElementUid, SubAssembly]:
        return {sub.key: sub for sub in self.assembly.subAssemblies}

    @functools.cached_property
    def euid_to_part(self) -> dict[ElementUid, Part]:
        return {part.key: part for part in self.assembly.parts}

    def traverse_assemblies(self) -> Iterator[tuple[Key, AssemblyInstance, SubAssembly]]:
        subassembly_deque: Deque[tuple[Key, SubAssembly]] = deque()
        visited: set[Key] = set()

        # Adds the root assembly to the traversal.
        for instance in self.assembly.rootAssembly.instances:
            if isinstance(instance, AssemblyInstance):
                instance_path: Key = (instance.id,)
                if instance_path in visited:
                    continue
                visited.add(instance_path)
                subassembly = self.euid_to_subassembly[instance.euid]
                yield instance_path, instance, subassembly
                subassembly_deque.append((instance_path, subassembly))

        # Adds all the subassemblies to the traversal, recursively.
        while subassembly_deque:
            path, sub_assembly = subassembly_deque.popleft()
            for instance in sub_assembly.instances:
                if isinstance(instance, AssemblyInstance):
                    instance_path = path + (instance.id,)
                    if instance_path in visited:
                        continue
                    visited.add(instance_path)
                    subassembly = self.euid_to_subassembly[instance.euid]
                    yield instance_path, instance, subassembly
                    subassembly_deque.append((instance_path, subassembly))

    @functools.cached_property
    def assembly_key_to_id(self) -> dict[Key, ElementUid]:
        key_map = {key: assembly.key for key, _, assembly in self.traverse_assemblies()}
        key_map[()] = self.assembly.rootAssembly.key
        return key_map

    @functools.cached_property
    def key_to_occurrence(self) -> dict[Key, Occurrence]:
        return {occurrence.key: occurrence for occurrence in self.assembly.rootAssembly.occurrences}

    @functools.cached_property
    def key_to_instance(self) -> dict[Key, Instance]:
        instance_mapping: dict[Key, Instance] = {}
        for instance in self.assembly.rootAssembly.instances:
            instance_mapping[(instance.id,)] = instance
        for path, assembly_instance, sub_assembly in self.traverse_assemblies():
            instance_mapping[path] = assembly_instance
            for instance in sub_assembly.instances:
                instance_mapping[path + (instance.id,)] = instance
        return instance_mapping

    @property
    def key_to_part_instance(self) -> dict[Key, PartInstance]:
        return {p: i for p, i in self.key_to_instance.items() if isinstance(i, PartInstance)}

    @property
    def key_to_assembly_instance(self) -> dict[Key, AssemblyInstance]:
        return {p: i for p, i in self.key_to_instance.items() if isinstance(i, AssemblyInstance)}

    @functools.cached_property
    def key_to_feature(self) -> dict[Key, MateRelationFeature | MateFeature]:
        feature_mapping: dict[Key, MateRelationFeature | MateFeature] = {}
        for feature in self.assembly.rootAssembly.features:
            feature_mapping[(feature.id,)] = feature
        for key, _, sub_assembly in self.traverse_assemblies():
            for feature in sub_assembly.features:
                feature_mapping[key + (feature.id,)] = feature
        return feature_mapping

    @functools.cached_property
    def key_to_mate_feature(self) -> dict[Key, MateFeature]:
        return {p: f for p, f in self.key_to_feature.items() if isinstance(f, MateFeature)}

    @functools.cached_property
    def key_to_mate_relation_feature(self) -> dict[Key, MateRelationFeature]:
        return {p: f for p, f in self.key_to_feature.items() if isinstance(f, MateRelationFeature)}

    @functools.cached_property
    def key_to_name(self) -> dict[Key, list[str]]:
        key_name_mapping: dict[Key, list[str]] = {}
        key_name_mapping[()] = []
        for instance in self.assembly.rootAssembly.instances:
            key_name_mapping[(instance.id,)] = [instance.name]
        for feature in self.assembly.rootAssembly.features:
            key_name_mapping[(feature.id,)] = [feature.featureData.name]
        for key, assembly_instance, sub_assembly in self.traverse_assemblies():
            key_name_mapping[key] = key_name_mapping[key[:-1]] + [assembly_instance.name]
            for instance in sub_assembly.instances:
                key_name_mapping[key + (instance.id,)] = key_name_mapping[key] + [instance.name]
            for feature in sub_assembly.features:
                key_name_mapping[key + (feature.id,)] = key_name_mapping[key] + [feature.featureData.name]
        return key_name_mapping

    def key_name(self, key: Key, prefix: Literal["link", "joint", None]) -> str:
        return ("" if prefix is None else f"{prefix}_") + clean_name("_".join(self.key_to_name.get(key, key)))

    @functools.cached_property
    def graph(self) -> nx.Graph:
        """Converts the assembly to an undirected graph of joints and parts.

        This checks that the assembly is connected as a single piece (meaning
        that there are no unconnected parts or joints) and that it is a tree
        (meaning that there are no parallel connections). Parallel connections
        arise when a part has two parents.

        Returns:
            The graph of the assembly.
        """
        graph = nx.Graph()
        for key, _ in self.key_to_part_instance.items():
            graph.add_node(key)

        def add_edge_safe(node_a: Key, node_b: Key, name: str) -> None:
            if node_a not in graph:
                raise ValueError(f"Node {self.key_name(node_a, 'link')} not found in graph")
            if node_b not in graph:
                raise ValueError(f"Node {self.key_name(node_b, 'link')} not found in graph")
            graph.add_edge(node_a, node_b, name=name)

        # Add edges between nodes that have a feature connecting them.
        for key, mate_feature in self.key_to_mate_feature.items():
            if mate_feature.suppressed:
                continue
            for mate_pair in itertools.combinations(mate_feature.featureData.matedEntities, 2):
                name = self.key_name(key, "joint")
                add_edge_safe(key[:-1] + mate_pair[0].key, key[:-1] + mate_pair[1].key, name)

        # Logs all of the nodes and edges in the graph.
        for edge in graph.edges:
            logger.debug(
                'Edge: Part "%s" -> Path "%s" (Feature "%s")',
                self.key_name(edge[0], "link"),
                self.key_name(edge[1], "link"),
                graph.edges[edge]["name"],
            )

        # If there are any unconnected nodes in the graph, raise an error.
        if not nx.is_connected(graph):
            num_components = nx.number_connected_components(graph)

            components: list[str] = []
            for component in nx.connected_components(graph):
                component_list = sorted([self.key_name(c, "link") for c in component])
                components.append("\n      ".join(component_list))
            components_string = "\n\n".join(f"  {i + 1: <3} {component}" for i, component in enumerate(components))

            raise ValueError(
                "The assembly is not fully connected! URDF export requires a single fully-connected robot, "
                f"but the graph has {num_components} components. Components:\n{components_string}"
            )

        return graph

    @functools.cached_property
    def central_node(self) -> Key:
        """Identifies the most central node of the assembly.

        Since the assembly is undirected by default, we need to choose a node
        to act as the central node. We use the node with the highest closeness
        centrality as the root.

        Returns:
            The key of the central node.
        """
        closeness_centrality = nx.closeness_centrality(self.graph)
        central_node: Key = max(closeness_centrality, key=closeness_centrality.get)
        return central_node

    @functools.cached_property
    def digraph(self) -> tuple[Key, nx.DiGraph]:
        """Converts the undirected graph to a directed graph.

        In order to convert the undirected graph to a directed graph, we need
        to choose a node to act as the root of the tree. We choose the node
        with the highest closeness centrality as the root.

        We also verify that the graph is actually a tree, and doesn't have any
        parallel connections, meaning there are no parts with two parents.

        Returns:
            The central node key and the directed graph.
        """
        if not nx.is_tree(self.graph):
            # Detects the parallel connections in the graph.
            for cycle in nx.cycle_basis(self.graph):
                for i, _ in enumerate(cycle):
                    if i == len(cycle) - 1:
                        break
                    logger.error(
                        "Parallel connection: %s -> %s",
                        self.key_name(cycle[i], "link"),
                        self.key_name(cycle[i + 1], "link"),
                    )
            raise ValueError("The assembly has parallel connections! URDF export requires no parallel connections.")

        central_node = self.central_node
        logger.debug("Central node: %s", self.key_name(central_node, "link"))
        return nx.bfs_tree(self.graph, central_node)

    @functools.cached_property
    def relations(self) -> dict[Key, MimicRelation]:
        relations: dict[Key, MimicRelation] = {}
        for path, mate_relation_feature in self.key_to_mate_relation_feature.items():
            if mate_relation_feature.suppressed:
                continue

            relation_type = mate_relation_feature.featureData.relationType
            match relation_type:
                case RelationType.GEAR:
                    parent_key, child_key = mate_relation_feature.keys(path[:-1])
                    ratio = mate_relation_feature.featureData.relationRatio
                    reverse = mate_relation_feature.featureData.reverseDirection
                    relations[child_key] = MimicRelation(
                        parent=parent_key,
                        multiplier=-ratio if reverse else ratio,
                    )

                case RelationType.LINEAR:
                    parent_key, child_key = mate_relation_feature.keys(path[:-1])
                    ratio = mate_relation_feature.featureData.relationRatio
                    reverse = mate_relation_feature.featureData.reverseDirection
                    relations[child_key] = MimicRelation(
                        parent=parent_key,
                        multiplier=-ratio if reverse else ratio,
                    )

                case _:
                    raise ValueError(f"Unsupported relation type: {relation_type}")

        return relations

    @functools.cached_property
    def ordered_joint_list(self) -> list[Joint]:
        digraph = self.digraph
        bfs_node_ordering = list(nx.topological_sort(digraph))
        node_level = {node: i for i, node in enumerate(bfs_node_ordering)}

        # Creates a topologically-sorted list of joints.
        joint_list: list[Joint] = []
        for joint_key, mate_feature in self.key_to_mate_feature.items():
            if mate_feature.suppressed:
                continue
            lhs_entity, rhs_entity = mate_feature.featureData.matedEntities
            lhs_key, rhs_key = joint_key[:-1] + lhs_entity.key, joint_key[:-1] + rhs_entity.key
            lhs_is_first = node_level[lhs_key] < node_level[rhs_key]
            parent_key, child_key = (lhs_key, rhs_key) if lhs_is_first else (rhs_key, lhs_key)
            parent_entity, child_entity = (lhs_entity, rhs_entity) if lhs_is_first else (rhs_entity, lhs_entity)
            mate_type = mate_feature.featureData.mateType
            joint_list.append(Joint(parent_key, child_key, parent_entity, child_entity, mate_type, joint_key))
        joint_list.sort(key=lambda x: (node_level[x.parent], node_level[x.child]))

        return joint_list

    @functools.cached_property
    def joint_limits(self) -> dict[ElementUid, JointLimits]:
        """Gets the feature information for each assembly.

        Features are the properties of each part in an assembly. In our case,
        we use features to get things like the joint limits for the parts,
        which are not given in the default assembly or assembly metadata APIs.

        Returns:
            A dictionary mapping each assembly key to its features.
        """

        def download_or_get_cached_features(assembly: RootAssembly | SubAssembly) -> Features:
            return self.get_or_use_cached(
                f"assembly_{assembly.key.unique_id}",
                lambda: self.api.get_features(assembly),
                lambda feats_str: Features.model_validate_json(feats_str),
                lambda feats: feats.model_dump_json(indent=2),
            )

        # Gets the features for the assembly and all subassemblies.
        assembly_features = {
            self.assembly.rootAssembly.key: download_or_get_cached_features(self.assembly.rootAssembly),
            **{
                sub_assembly.key: download_or_get_cached_features(sub_assembly)
                for sub_assembly in self.assembly.subAssemblies
            },
        }

        def get_feature_value(key: str, feature: Feature) -> str | None:
            if (attrib := feature.message.parameter_dict.get(key)) is None:
                return None
            match attrib["typeName"]:
                case "BTMParameterNullableQuantity":
                    return None if attrib["message"]["isNull"] else attrib["message"]["expression"]
                case _:
                    return None

        # Gets the joint information for each feature from the assembly features.
        joint_limits: dict[ElementUid, JointLimits] = {}
        for assembly_key, assembly_feature in assembly_features.items():
            for feature in assembly_feature.features:
                key = ElementUid(
                    document_id=assembly_key.document_id,
                    document_microversion=assembly_key.document_microversion,
                    element_id=assembly_key.element_id,
                    part_id=feature.message.featureId,
                )

                if (
                    "limitsEnabled" not in feature.message.parameter_dict
                    or not feature.message.parameter_dict["limitsEnabled"]["message"]["value"]
                ):
                    joint_limits[key] = JointLimits()
                    continue

                joint_limits[key] = JointLimits(
                    z_min_expression=get_feature_value("limitZMin", feature),
                    z_max_expression=get_feature_value("limitZMax", feature),
                    axial_z_min_expression=get_feature_value("limitAxialZMin", feature),
                    axial_z_max_expression=get_feature_value("limitAxialZMax", feature),
                )

        return joint_limits

    def get_urdf_part(self, key: Key, joint: Joint | None = None) -> urdf.Link:
        part_name = self.key_name(key, None)
        part_instance = self.key_to_part_instance[key]
        part = self.euid_to_part[part_instance.euid]

        # Gets the configuration string suffix.
        if part.configuration == "default":
            configuration_str = ""
        elif len(part.configuration) > 40:
            configuration_str = hashlib.md5(part.configuration.encode()).hexdigest()[:16]
        else:
            configuration_str = part.configuration

        part_color = self.part_color(part)
        part_dynamic = self.part_dynamics(part).bodies[part_instance.partId]

        # If the part is the root part, move the STL to be relative to the
        # center of mass and principle inertia axes, otherwise move it to
        # the origin of the part frame.
        com_to_part_tf = np.matrix(np.eye(4))
        com_to_part_tf[:3, 3] = -np.array(part_dynamic.center_of_mass).reshape(3, 1)
        if joint is None:
            stl_origin_to_part_tf = np.matrix(com_to_part_tf)
        else:
            stl_origin_to_part_tf = inv_tf(joint.child_entity.matedCS.part_to_mate_tf)
        self.stl_origin_to_part_tf[key] = stl_origin_to_part_tf

        part_file_name = f"{part_name}{configuration_str}.stl"
        part_file_path = self.mesh_dir / part_file_name
        if part_file_path.exists():
            logger.info("Using cached STL file %s", part_file_path)
        else:
            logger.info("Downloading STL file %s", part_file_path)
            buffer = io.BytesIO()
            self.api.download_stl(part, buffer)
            buffer.seek(0)
            mesh = stl.mesh.Mesh.from_file(None, fh=buffer)
            mesh = apply_matrix_(mesh, stl_origin_to_part_tf)
            mesh.save(part_file_path)

        # Move the mesh origin and dynamics from the part frame to the parent
        # joint frame (since URDF expects this by convention).
        mesh_origin = urdf.Origin.zero_origin()
        center_of_mass = part_dynamic.center_of_mass_in_frame(stl_origin_to_part_tf)
        # inertia = part_dynamic.inertia_in_frame(stl_origin_to_part_tf)
        inertia = part_dynamic.inertia_matrix

        # Aligns the inertia matrix with the principal axes.
        principal_axes = part_dynamic.principal_axes_in_frame(stl_origin_to_part_tf)
        principal_axes_rpy = rotation_matrix_to_euler_angles(principal_axes)
        # principal_axes_rpy = (0.0, 0.0, 0.0)

        urdf_file_path = f"./meshes/{part_file_name}"
        urdf_link_name = self.key_name(key, "link")

        urdf_part_link = urdf.Link(
            name=urdf_link_name,
            visual=urdf.VisualLink(
                origin=mesh_origin,
                geometry=urdf.MeshGeometry(filename=urdf_file_path),
                material=urdf.Material(
                    name=f"{urdf_link_name}_material",
                    color=[c / 255.0 for c in part_color],
                ),
            ),
            inertial=urdf.InertialLink(
                origin=urdf.Origin(
                    xyz=center_of_mass,
                    rpy=principal_axes_rpy,
                ),
                mass=part_dynamic.mass[0],
                inertia=urdf.Inertia(
                    ixx=float(inertia[0, 0]),
                    ixy=float(inertia[0, 1]),
                    ixz=float(inertia[0, 2]),
                    iyy=float(inertia[1, 1]),
                    iyz=float(inertia[1, 2]),
                    izz=float(inertia[2, 2]),
                ),
            ),
            collision=urdf.CollisionLink(
                origin=mesh_origin,
                geometry=urdf.MeshGeometry(filename=urdf_file_path),
            ),
        )

        return urdf_part_link

    def get_effort_and_velocity(self, name: str, default_effort: float, default_velocity: float) -> tuple[float, float]:
        effort = default_effort
        for suffix, value in self.suffix_to_joint_effort:
            if name.lower().endswith(suffix):
                effort = value
                break
        velocity = default_velocity
        for suffix, value in self.suffix_to_joint_velocity:
            if name.lower().endswith(suffix):
                velocity = value
                break
        return effort, velocity

    def get_urdf_joint(self, joint: Joint) -> urdf.BaseJoint:
        """Returns the URDF joint.

        Args:
            joint: The joint to convert.

        Returns:
            The URDF link and joint.
        """
        parent_stl_origin_to_part_tf = self.stl_origin_to_part_tf[joint.parent]
        parent_part_to_mate_tf = joint.parent_entity.matedCS.part_to_mate_tf
        parent_stl_origin_to_mate_tf = parent_stl_origin_to_part_tf @ parent_part_to_mate_tf

        # Gets the joint limits.
        joint_assembly_id, feature_id = self.assembly_key_to_id[joint.joint_key[:-1]], joint.joint_key[-1]
        joint_info_key = ElementUid(
            document_id=joint_assembly_id.document_id,
            document_microversion=joint_assembly_id.document_microversion,
            element_id=joint_assembly_id.element_id,
            part_id=feature_id,
        )
        joint_limits = self.joint_limits[joint_info_key]
        expression_resolver = ExpressionResolver(joint_assembly_id.configuration)

        def resolve(expression: str | None) -> float | None:
            return None if expression is None else expression_resolver.read_expression(expression)

        name = self.key_name(joint.joint_key, "joint")
        origin = urdf.Origin.from_matrix(parent_stl_origin_to_mate_tf)
        mate_type = joint.mate_type

        match mate_type:
            case MateType.FASTENED:
                parent, child = self.key_name(joint.parent, "link"), self.key_name(joint.child, "link")
                return urdf.FixedJoint(
                    name=name,
                    parent=parent,
                    child=child,
                    origin=origin,
                )

            case MateType.REVOLUTE:
                parent, child = self.key_name(joint.parent, "link"), self.key_name(joint.child, "link")
                mimic_joint = self.relations.get(joint.joint_key)

                min_value = resolve(joint_limits.axial_z_min_expression)
                max_value = resolve(joint_limits.axial_z_max_expression)

                if min_value is None or max_value is None:
                    raise ValueError(f"Revolute joint {name} ({parent} -> {child}) does not have limits defined.")

                effort, velocity = self.get_effort_and_velocity(
                    name,
                    self.default_revolute_joint_limits.effort,
                    self.default_revolute_joint_limits.velocity,
                )

                return urdf.RevoluteJoint(
                    name=name,
                    parent=parent,
                    child=child,
                    origin=origin,
                    axis=urdf.Axis((0.0, 0.0, 1.0)),
                    limits=urdf.JointLimits(
                        effort=effort,
                        velocity=velocity,
                        lower=min_value,
                        upper=max_value,
                    ),
                    mimic=(
                        None
                        if mimic_joint is None
                        else urdf.JointMimic(
                            joint=self.key_name(mimic_joint.parent, "joint"),
                            multiplier=mimic_joint.multiplier,
                            offset=0.0,
                        )
                    ),
                )

            case MateType.SLIDER:
                parent, child = self.key_name(joint.parent, "link"), self.key_name(joint.child, "link")
                mimic_joint = self.relations.get(joint.joint_key)

                min_value = resolve(joint_limits.z_min_expression)
                max_value = resolve(joint_limits.z_max_expression)

                if min_value is None or max_value is None:
                    raise ValueError(f"Slider joint {name} ({parent} -> {child}) does not have limits defined.")

                effort, velocity = self.get_effort_and_velocity(
                    name,
                    self.default_prismatic_joint_limits.effort,
                    self.default_prismatic_joint_limits.velocity,
                )

                return urdf.PrismaticJoint(
                    name=name,
                    parent=parent,
                    child=child,
                    origin=origin,
                    axis=urdf.Axis((0.0, 0.0, 1.0)),
                    limits=urdf.JointLimits(
                        effort=effort,
                        velocity=velocity,
                        lower=min_value,
                        upper=max_value,
                    ),
                    mimic=(
                        None
                        if mimic_joint is None
                        else urdf.JointMimic(
                            joint=self.key_name(mimic_joint.parent, "joint"),
                            multiplier=mimic_joint.multiplier,
                            offset=0.0,
                        )
                    ),
                )

            case MateType.PLANAR:
                raise NotImplementedError

            case MateType.CYLINDRICAL:
                raise NotImplementedError

            case MateType.PIN_SLOT:
                raise NotImplementedError

            case MateType.BALL:
                raise NotImplementedError

            case MateType.PARALLEL:
                raise NotImplementedError

            case _:
                raise ValueError(f"Unsupported mate type: {mate_type}")

    def save_urdf(self) -> None:
        """Saves a URDF file for the assembly to the output directory."""
        urdf_parts: list[urdf.Link | urdf.BaseJoint] = []

        # Add the first link, since it has no incoming joint.
        part_link = self.get_urdf_part(self.central_node)
        urdf_parts.append(part_link)

        # For debugging.
        # names = {k: chr(ord('a') + i) for i, k in enumerate(self.key_to_part_instance.keys())}
        # for k, v in self.key_to_occurrence.items():
        #     print(f"T_w_{names[k]} = np.", end="")
        #     print(repr(v.world_to_part_tf))
        # for j in self.ordered_joint_list:
        #     ab = f"{names[j.parent]}{names[j.child]}"
        #     print(f"T_{ab}_{names[j.parent]} = np.", end="")
        #     print(repr(j.parent_entity.matedCS.part_to_mate_tf))
        #     print(f"T_{ab}_{names[j.child]} = np.", end="")
        #     print(repr(j.child_entity.matedCS.part_to_mate_tf))

        # Creates a URDF joint for each feature connecting two parts.
        for joint in self.ordered_joint_list:
            urdf_joint = self.get_urdf_joint(joint)
            urdf_link = self.get_urdf_part(joint.child, joint)
            urdf_parts.append(urdf_link)
            urdf_parts.append(urdf_joint)

        # Saves the final URDF.
        robot_name = clean_name(str(self.assembly_metadata.property_map.get("Name", "robot")))
        urdf_robot = urdf.Robot(name=robot_name, parts=urdf_parts)
        urdf_robot.save(self.output_dir / f"{robot_name}.urdf")
