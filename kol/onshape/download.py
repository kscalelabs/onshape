# mypy: disable-error-code="attr-defined"
"""Defines functions for download Onshape links as URDFs.

Note that this module is intended to be used as a stand-alone script which
outputs a URDF file. We can additionally post-process the URDF files using
various downstream scripts, but we should avoid adding any unnecessary behavior
here.
"""

import asyncio
import hashlib
import io
import itertools
import logging
import re
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Coroutine, Deque, Iterator, Literal, Sequence, TypeVar

import networkx as nx
import numpy as np
import stl
from scipy.spatial.transform import Rotation as R

from kol.formats import urdf
from kol.onshape.api import OnshapeApi
from kol.onshape.cached_api import CachedOnshapeApi
from kol.onshape.cacher import Cacher
from kol.onshape.client import DocumentInfo, OnshapeClient
from kol.onshape.config import DownloadConfig, Joint, JointLimits, MimicRelation
from kol.onshape.schema.assembly import (
    Assembly,
    AssemblyInstance,
    AssemblyMetadata,
    Instance,
    Key,
    MateFeature,
    MateGroupFeature,
    MateRelationFeature,
    MateType,
    Part,
    PartInstance,
    RelationType,
    SubAssembly,
)
from kol.onshape.schema.common import ElementUid
from kol.onshape.schema.document import Document
from kol.onshape.schema.features import Feature, FeatureStatus
from kol.onshape.schema.part import PartDynamics, PartMetadata
from kol.utils.errors import catch_error
from kol.utils.geometry import apply_matrix_, inv_tf, transform_inertia_tensor
from kol.utils.logging import configure_logging
from kol.utils.resolvers import ExpressionResolver

logger = logging.getLogger(__name__)

T = TypeVar("T")
Tk = TypeVar("Tk")
Tv = TypeVar("Tv")

Color = tuple[int, int, int, int]


def clean_name(name: str) -> str:
    name = name.replace("<", "(").replace(">", ")")
    # name = re.sub(r"[<>]", "", name)
    name = re.sub(r"\s+", "_", name)
    return name


async def gather_dict(d: dict[Tk, Coroutine[Any, Any, Tv]]) -> dict[Tk, Tv]:
    dict_items = [(k, v) for k, v in d.items()]
    values = await asyncio.gather(*(v for _, v in dict_items))
    return {k: v for (k, _), v in zip(dict_items, values)}


class FailedCheckError(ValueError):
    def __init__(self, msg: str, *suggestions: str) -> None:
        full_msg = f"{msg}" + "".join(f"\n * {s}" for s in suggestions)
        super().__init__(full_msg)


def traverse_assemblies(assembly: Assembly) -> Iterator[tuple[Key, AssemblyInstance, SubAssembly]]:
    subassembly_deque: Deque[tuple[Key, SubAssembly]] = deque()
    visited: set[Key] = set()

    # Mapping from EUID to subassembly.
    euid_to_subassembly = {sub.key: sub for sub in assembly.subAssemblies}

    # Adds the root assembly to the traversal.
    for instance in assembly.rootAssembly.instances:
        if isinstance(instance, AssemblyInstance):
            instance_path: Key = (instance.id,)
            if instance_path in visited:
                continue
            visited.add(instance_path)
            subassembly = euid_to_subassembly[instance.euid]
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
                subassembly = euid_to_subassembly[instance.euid]
                yield instance_path, instance, subassembly
                subassembly_deque.append((instance_path, subassembly))


def get_key_to_instance_mapping(assembly: Assembly) -> dict[Key, Instance]:
    key_to_part_instance: dict[Key, Instance] = {}
    for instance in assembly.rootAssembly.instances:
        key_to_part_instance[(instance.id,)] = instance
    for path, assembly_instance, sub_assembly in traverse_assemblies(assembly):
        key_to_part_instance[path] = assembly_instance
        for instance in sub_assembly.instances:
            key_to_part_instance[path + (instance.id,)] = instance
    return key_to_part_instance


def get_key_to_part_instance_mapping(key_to_instance_mapping: dict[Key, Instance]) -> dict[Key, PartInstance]:
    return {k: v for k, v in key_to_instance_mapping.items() if isinstance(v, PartInstance)}


def get_key_to_assembly_instance_mapping(key_to_instance_mapping: dict[Key, Instance]) -> dict[Key, AssemblyInstance]:
    return {k: v for k, v in key_to_instance_mapping.items() if isinstance(v, AssemblyInstance)}


def get_key_to_feature(assembly: Assembly) -> dict[Key, MateRelationFeature | MateFeature | MateGroupFeature]:
    feature_mapping: dict[Key, MateRelationFeature | MateFeature | MateGroupFeature] = {}
    for feature in assembly.rootAssembly.features:
        feature_mapping[(feature.id,)] = feature
    for key, _, sub_assembly in traverse_assemblies(assembly):
        for feature in sub_assembly.features:
            feature_mapping[key + (feature.id,)] = feature
    return feature_mapping


def get_key_to_mate_feature(
    key_to_feature: dict[Key, MateRelationFeature | MateFeature | MateGroupFeature],
) -> dict[Key, MateFeature]:
    return {p: f for p, f in key_to_feature.items() if isinstance(f, MateFeature)}


def get_key_to_mate_relation_feature(
    key_to_feature: dict[Key, MateRelationFeature | MateFeature | MateGroupFeature],
) -> dict[Key, MateRelationFeature]:
    return {p: f for p, f in key_to_feature.items() if isinstance(f, MateRelationFeature)}


def get_key_to_name(assembly: Assembly) -> dict[Key, list[str]]:
    key_name_mapping: dict[Key, list[str]] = {}
    key_name_mapping[()] = []
    for instance in assembly.rootAssembly.instances:
        key_name_mapping[(instance.id,)] = [instance.name]
    for feature in assembly.rootAssembly.features:
        key_name_mapping[(feature.id,)] = [feature.featureData.name]
    for key, assembly_instance, sub_assembly in traverse_assemblies(assembly):
        key_name_mapping[key] = key_name_mapping[key[:-1]] + [assembly_instance.name]
        for instance in sub_assembly.instances:
            key_name_mapping[key + (instance.id,)] = key_name_mapping[key] + [instance.name]
        for feature in sub_assembly.features:
            key_name_mapping[key + (feature.id,)] = key_name_mapping[key] + [feature.featureData.name]
    return key_name_mapping


class KeyNamer:
    def __init__(self, assembly: Assembly, joiner: str = ":", clean: bool = True) -> None:
        self.key_to_name = get_key_to_name(assembly)
        self.joiner = joiner
        self.clean = clean

    def lookup_key(
        self,
        name: str,
        prefix: Literal["link", "joint", None],
        joiner: str | None = None,
        clean: bool | None = None,
    ) -> Key:
        for key in self.key_to_name:
            if self(key, prefix, joiner, clean) == name:
                return key
        raise ValueError(f'Name "{name}" not found in key namer.')

    def __call__(
        self,
        key: Key,
        prefix: Literal["link", "joint", None],
        joiner: str | None = None,
        clean: bool | None = None,
    ) -> str:
        if clean is None:
            clean = self.clean
        if joiner is None:
            joiner = self.joiner
        name_parts = self.key_to_name.get(key, key)
        name = joiner.join(name_parts)
        if clean:
            name = clean_name(name)
        return ("" if prefix is None else f"{prefix}{joiner}") + name


def log_graph(graph: nx.Graph, key_namer: KeyNamer) -> None:
    for edge in graph.edges:
        logger.info(
            'Edge: Part "%s" -> Path "%s" (Feature "%s")',
            key_namer(edge[0], "link"),
            key_namer(edge[1], "link"),
            graph.edges[edge]["name"],
        )


def get_graph(
    assembly: Assembly,
    key_namer: KeyNamer | None = None,
    key_to_part_instance: dict[Key, PartInstance] | None = None,
    key_to_mate_feature: dict[Key, MateFeature] | None = None,
) -> tuple[nx.Graph, set[Key]]:
    """Converts the assembly to an undirected graph of joints and parts.

    This checks that the assembly is connected as a single piece (meaning
    that there are no unconnected parts or joints) and that it is a tree
    (meaning that there are no parallel connections). Parallel connections
    arise when a part has two parents.

    Args:
        assembly: The assembly to convert.
        key_namer: The key namer to use.
        key_to_part_instance: The key to part instance mapping.
        key_to_mate_feature: The key to mate feature mapping

    Returns:
        The graph of the assembly, along with a set of the ignored joints.
    """
    graph = nx.Graph()

    if key_namer is None:
        key_namer = KeyNamer(assembly)
    if key_to_part_instance is None:
        key_to_part_instance = get_key_to_part_instance_mapping(get_key_to_instance_mapping(assembly))
    if key_to_mate_feature is None:
        key_to_feature = get_key_to_feature(assembly)
        key_to_mate_feature = get_key_to_mate_feature(key_to_feature)

    # Adds all the graph nodes.
    for key, _ in key_to_part_instance.items():
        graph.add_node(key)

    def add_edge_safe(node_a: Key, node_b: Key, joint: Key) -> None:
        # Here, we are considering a graph with no ignored joints
        # to make sure the original graph is connected.
        for node_lhs, node_rhs in ((node_a, node_b), (node_b, node_a)):
            if node_lhs not in graph:
                lhs_name = key_namer(node_lhs, None, " : ", False)
                rhs_name = key_namer(node_rhs, None, " : ", False)
                joint_name = key_namer(joint, None, " : ", False)
                raise ValueError(
                    f'Node "{lhs_name}" (which is connected to "{rhs_name}" via "{joint_name}") '
                    "not found in graph. Do you have a mate between a part and the origin?"
                )

        name = key_namer(joint, "joint")
        graph.add_edge(node_a, node_b, name=name)

    # Add edges between nodes that have a feature connecting them.
    for key, mate_feature in key_to_mate_feature.items():
        if mate_feature.suppressed:
            continue
        for mate_pair in itertools.combinations(mate_feature.featureData.matedEntities, 2):
            add_edge_safe(key[:-1] + mate_pair[0].key, key[:-1] + mate_pair[1].key, key)

    # If there are any unconnected nodes in the graph, raise an error.
    if not nx.is_connected(graph):
        num_components = nx.number_connected_components(graph)
        components: list[str] = []
        for component in nx.connected_components(graph):
            component_list = sorted([key_namer(c, None, " : ", False) for c in component])
            components.append("\n      ".join(component_list))
        components_string = "\n\n".join(f"  {i + 1: <3} {component}" for i, component in enumerate(components))
        raise FailedCheckError(
            "The assembly is not fully connected! URDF export requires a single fully-connected robot, "
            f"but the graph has {num_components} components. Ensure that all components have a mate connecting them. "
            f"Components:\n{components_string}"
        )

    if not nx.is_tree(graph):
        # Detects the parallel connections in the graph.
        first_n_bad: list[tuple[str, str]] = []
        for cycle in nx.cycle_basis(graph):
            for i, _ in enumerate(cycle):
                if i == len(cycle) - 1:
                    break
                first_n_bad.append(
                    (
                        key_namer(cycle[i], None, " : ", False),
                        key_namer(cycle[i + 1], None, " : ", False),
                    )
                )
        first_n_bad_str = "\n".join(f" * {a} -> {b}" for a, b in first_n_bad[:5])
        prefix = "first " if len(first_n_bad) > 5 else ""
        raise FailedCheckError(
            f"The assembly has parallel connections! URDF export requires no parallel connections. The {prefix}"
            f"{min(5, len(first_n_bad))} joints in the parallel connection are:\n{first_n_bad_str}"
        )

    return graph


def get_digraph(graph: nx.Graph, override_central_node: Key | None = None) -> tuple[nx.DiGraph, Key]:
    """Converts the undirected graph to a directed graph using a root node.

    Args:
        graph: The undirected graph to convert.
        override_central_node: The central node to use.

    Returns:
        The central node of the graph and the directed graph
    """
    # Gets the most central node as the "root" node.
    central_node: Key
    if override_central_node is not None:
        central_node = override_central_node
    else:
        closeness_centrality = nx.closeness_centrality(graph)
        central_node = max(closeness_centrality, key=closeness_centrality.get)
    return nx.bfs_tree(graph, central_node), central_node


def get_joint_list(
    digraph: nx.DiGraph,
    key_to_mate_feature: dict[Key, MateFeature],
) -> list[Joint]:
    # Creates a BFS ordering of the graph.
    bfs_node_ordering = list(nx.topological_sort(digraph))
    node_level = {node: i for i, node in enumerate(bfs_node_ordering)}

    # Creates a topologically-sorted list of joints.
    joint_list: list[Joint] = []
    for joint_key, mate_feature in key_to_mate_feature.items():
        if mate_feature.suppressed:
            continue

        lhs_entity, rhs_entity = mate_feature.featureData.matedEntities
        lhs_key, rhs_key = joint_key[:-1] + lhs_entity.key, joint_key[:-1] + rhs_entity.key

        lhs_is_first = node_level[lhs_key] < node_level[rhs_key]
        parent_key, child_key = (lhs_key, rhs_key) if lhs_is_first else (rhs_key, lhs_key)
        parent_entity, child_entity = (lhs_entity, rhs_entity) if lhs_is_first else (rhs_entity, lhs_entity)
        mate_type = mate_feature.featureData.mateType
        joint = Joint(
            parent_key,
            child_key,
            parent_entity,
            child_entity,
            mate_type,
            joint_key,
            lhs_is_first,
        )
        joint_list.append(joint)
    joint_list.sort(key=lambda x: (node_level[x.parent], node_level[x.child]))

    return joint_list


async def get_joint_limits(assembly: Assembly, api: OnshapeApi) -> dict[ElementUid, JointLimits]:
    # Gets the features for the assembly and all subassemblies.
    assembly_features = await gather_dict(
        {
            assembly.rootAssembly.key: api.get_features(assembly.rootAssembly),
            **{sub_assembly.key: api.get_features(sub_assembly) for sub_assembly in assembly.subAssemblies},
        }
    )

    joint_limits: dict[ElementUid, JointLimits] = {}

    def get_feature_value(key: str, feature: Feature) -> str | None:
        if (attrib := feature.message.parameter_dict.get(key)) is None:
            return None
        match attrib["typeName"]:
            case "BTMParameterNullableQuantity":
                return None if attrib["message"]["isNull"] else attrib["message"]["expression"]
            case _:
                return None

    for assembly_key, assembly_feature in assembly_features.items():
        for feature_state in assembly_feature.featureStates:
            if feature_state.value.message.featureStatus != FeatureStatus.OK:
                logging.warn(
                    "Feature %s has status %s",
                    feature_state.key,
                    feature_state.value.message.featureStatus,
                )

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


def get_part_color(part_metadata: PartMetadata) -> Color | None:
    part_color = part_metadata.property_map.get("Appearance")
    if part_color is None:
        return None
    return (
        part_color["color"]["red"],
        part_color["color"]["green"],
        part_color["color"]["blue"],
        part_color["opacity"],
    )


def get_part_name(part: Part, part_metadata: PartMetadata) -> str:
    return str(part_metadata.property_map.get("Name", part.key.unique_id))


async def check_part(part: Part, api: OnshapeApi) -> tuple[PartMetadata, PartDynamics]:
    """Checks the metadata and mass properties of a single part.

    Args:
        part: The part to check.
        api: The Onshape API to use.

    Returns:
        The part metadata and mass properties.

    Raises:
        FailedCheckError: If the part is invalid.
    """
    part_metadata, part_mass_properties = await asyncio.gather(
        api.get_part_metadata(part),
        api.get_part_mass_properties(part),
    )
    part_name = get_part_name(part, part_metadata)

    if get_part_color(part_metadata) is None:
        raise FailedCheckError(f'Part "{part_name}" has no color. Check that the part has a color.')

    part_dynamic = part_mass_properties.bodies[part.partId]
    mass = part_dynamic.mass[0]
    if mass <= 0:
        raise FailedCheckError(f'Part "{part_name}" has a mass of {mass}. All parts should have a positive mass.')

    return part_metadata, part_mass_properties


async def get_mate_relations(
    key_to_mate_relation_feature: dict[Key, MateRelationFeature],
    *,
    config: DownloadConfig | None = None,
) -> dict[Key, MimicRelation]:
    """Gets the mimic relations from the mate relations.

    Args:
        key_to_mate_relation_feature: The key to mate relation feature mapping.
        config: The download configuration.

    Returns:
        The mimic relations.
    """
    if config is None:
        config = DownloadConfig()
    relations: dict[Key, MimicRelation] = {}
    if config.disable_mimics:
        return relations

    for path, mate_relation_feature in key_to_mate_relation_feature.items():
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


@dataclass
class CheckedDocument:
    document: Document
    assembly: Assembly
    assembly_metadata: AssemblyMetadata
    key_to_part_instance: dict[Key, PartInstance]
    key_to_assembly_instance: dict[Key, AssemblyInstance]
    key_to_feature: dict[Key, MateRelationFeature | MateFeature | MateGroupFeature]
    key_to_mate_feature: dict[Key, MateFeature]
    key_to_euid: dict[Key, ElementUid]
    euid_to_part: dict[ElementUid, Part]
    key_namer: KeyNamer
    part_metadata: dict[ElementUid, PartMetadata]
    part_dynamics: dict[ElementUid, PartDynamics]
    central_node: Key
    digraph: nx.DiGraph
    joints: list[Joint]
    joint_limits: dict[ElementUid, JointLimits]
    mate_relations: dict[Key, MimicRelation]


async def check_document(
    document_info: DocumentInfo,
    api: OnshapeApi,
    *,
    config: DownloadConfig | None = None,
) -> CheckedDocument:
    """Checks that a document is valid.

    This function is basically a "pre-processing" step that runs light-weight
    checks on the document to make sure that it can be converted to a URDF
    successfully. We don't download any meshes or do any heavy processing here,
    just some basic checks to make sure that the document is in a good state.

    Args:
        document_info: The document to check.
        api: The Onshape API to use
        config: The converter configuration.

    Raises:
        FailedCheckError: If the document is invalid.
    """
    if config is None:
        config = DownloadConfig()

    # Checks that the document can be downloaded. We do this first to just make
    # sure that the user isn't trying to retrieve a private document.
    try:
        document = await api.get_document(document_info.document_id)
    except Exception as e:
        raise FailedCheckError(
            f"Failed to get document {document_info.get_url()}",
            "Check that the document ID is correct.",
            "Check that the document is not private.",
        ) from e

    # Checks that the assembly is valid.
    try:
        assembly = await api.get_assembly(document_info)
    except Exception as e:
        raise FailedCheckError(
            f"Failed to get assembly for document {document_info.get_url()}",
            "Check that the document is an assembly, not a part studio.",
        ) from e

    # Checks that the assembly metadata is valid.
    try:
        assembly_metadata = await api.get_assembly_metadata(assembly.rootAssembly)
    except Exception as e:
        raise FailedCheckError(
            f"Failed to get assembly metadata for document {document_info.get_url()}",
            "Check that the assembly is not empty.",
        ) from e

    # Gets some mappings from the assembly to make subsequent lookups easier.
    key_to_instance = get_key_to_instance_mapping(assembly)
    key_to_part_instance = get_key_to_part_instance_mapping(key_to_instance)
    key_to_assembly_instance = get_key_to_assembly_instance_mapping(key_to_instance)
    key_to_feature = get_key_to_feature(assembly)
    key_to_mate_feature = get_key_to_mate_feature(key_to_feature)
    key_to_mate_relation_feature = get_key_to_mate_relation_feature(key_to_feature)
    key_to_euid = {key: assembly.key for key, _, assembly in traverse_assemblies(assembly)}
    key_to_euid[()] = assembly.rootAssembly.key
    euid_to_part = {part.key: part for part in assembly.parts}
    key_namer = KeyNamer(assembly)

    try:
        graph = get_graph(assembly, key_namer, key_to_part_instance, key_to_mate_feature)

    except ValueError as e:
        raise FailedCheckError(
            f"Failed to get graph for document {document_info.get_url()}",
            "Check that the assembly is fully connected.",
            "Check that there are no parallel connections.",
            "Check that no parts are connected to the origin.",
        ) from e

    try:
        override_central_node: Key | None
        if config.override_central_node is not None:
            override_central_node = key_namer.lookup_key(config.override_central_node, None, None, False)
        else:
            override_central_node = None
        digraph, central_node = get_digraph(graph, override_central_node)

    except ValueError as e:
        raise FailedCheckError(
            f"Failed to get digraph for document {document_info.get_url()}",
            "Check that the provided central node is in the graph.",
        ) from e

    # Checks all the parts in the assembly.
    check_part_results = await asyncio.gather(*(catch_error(check_part(part, api)) for part in assembly.parts))
    checked_part_properties, errs = zip(*check_part_results)

    if any(err is not None for err in errs):
        raise FailedCheckError(
            f"Invalid parts for document {document_info.get_url()}",
            *(str(err) for err in errs if err is not None),
        )

    part_metadata = {part.key: md for part, (md, _) in zip(assembly.parts, checked_part_properties)}
    part_dynamics = {part.key: dyn for part, (_, dyn) in zip(assembly.parts, checked_part_properties)}

    # Checks all the joints in the assembly.
    joints = get_joint_list(digraph, key_to_mate_feature)
    joint_limits = await get_joint_limits(assembly, api)

    # Checks all the mate relations in the assembly.
    try:
        mate_relations = await get_mate_relations(key_to_mate_relation_feature, config=config)

    except Exception as e:
        raise FailedCheckError(
            f"Failed to get mate relations for document {document_info.get_url()}",
            "Check that you are only using supported mimic relations.",
        ) from e

    return CheckedDocument(
        document=document,
        assembly=assembly,
        assembly_metadata=assembly_metadata,
        key_to_part_instance=key_to_part_instance,
        key_to_assembly_instance=key_to_assembly_instance,
        key_to_feature=key_to_feature,
        key_to_mate_feature=key_to_mate_feature,
        key_to_euid=key_to_euid,
        euid_to_part=euid_to_part,
        part_metadata=part_metadata,
        part_dynamics=part_dynamics,
        key_namer=key_namer,
        central_node=central_node,
        digraph=digraph,
        joints=joints,
        joint_limits=joint_limits,
        mate_relations=mate_relations,
    )


async def download_stl(
    doc: CheckedDocument,
    key: Key,
    part_file_path: Path,
    api: OnshapeApi,
    stl_origin_to_part_tf: np.ndarray,
    min_facet_width: float | None = None,
) -> Path:
    part_instance = doc.key_to_part_instance[key]
    part = doc.euid_to_part[part_instance.euid]

    buffer = io.BytesIO()
    await api.download_stl(part, buffer, min_facet_width=min_facet_width)
    buffer.seek(0)
    mesh_obj = stl.mesh.Mesh.from_file(None, fh=buffer)
    mesh_obj = apply_matrix_(mesh_obj, stl_origin_to_part_tf)
    mesh_obj.save(part_file_path)

    return part_file_path


def get_urdf_part(
    doc: CheckedDocument,
    key: Key,
    mesh_dir_name: str,
    joint: Joint | None = None,
) -> tuple[urdf.Link, np.ndarray]:
    """Returns the URDF link for a part.

    Args:
        doc: The checked document.
        key: The part key.
        mesh_dir: The directory to save the mesh files.
        mesh_dir_name: The name of the mesh directory.
        joint: The joint to use.

    Returns:
        The URDF link and the transformation matrix from the STL origin to
        the part frame.
    """
    part_name = doc.key_namer(key, None)
    part_instance = doc.key_to_part_instance[key]
    part = doc.euid_to_part[part_instance.euid]

    part_metadata = doc.part_metadata[part_instance.euid]
    if (part_color := get_part_color(part_metadata)) is None:
        raise ValueError(f"Part {part_name} has no color.")
    part_dynamic = doc.part_dynamics[part_instance.euid].bodies[part.partId]

    # If the part is the root part, move the STL to be relative to the
    # center of mass and principle inertia axes, otherwise move it to
    # the origin of the part frame.
    com_to_part_tf = np.eye(4)
    com_to_part_tf[:3, 3] = -np.array(part_dynamic.center_of_mass).reshape(3)
    if joint is None:
        stl_origin_to_part_tf = com_to_part_tf
    else:
        stl_origin_to_part_tf = inv_tf(joint.child_entity.matedCS.part_to_mate_tf)

    # Gets the part mass.
    mass = part_dynamic.mass[0]
    if mass <= 0.0:
        raise ValueError(f"Part {part_name} has a mass of {mass}.")

    # Move the mesh origin and dynamics from the part frame to the parent
    # joint frame (since URDF expects this by convention).
    mesh_origin = urdf.Origin.zero_origin()
    center_of_mass = part_dynamic.center_of_mass_in_frame(stl_origin_to_part_tf)

    inertia = part_dynamic.inertia_matrix
    inertia_transformed = transform_inertia_tensor(inertia, np.matrix(stl_origin_to_part_tf[:3, :3]))

    principal_axes = part_dynamic.principal_axes_in_frame(stl_origin_to_part_tf)
    principal_axes_rpy = R.from_matrix(principal_axes).as_euler("xyz", degrees=False)

    # Gets the configuration string suffix.
    if part.configuration == "default":
        configuration_str = ""
    elif len(part.configuration) > 40:
        configuration_str = hashlib.md5(part.configuration.encode()).hexdigest()[:16]
    else:
        configuration_str = part.configuration

    # Downloads the STL file.
    part_file_name = f"{part_name}{configuration_str}.stl"
    urdf_file_path = f"{mesh_dir_name}/{part_file_name}"
    urdf_link_name = doc.key_namer(key, "link")

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
            mass=mass,
            inertia=urdf.Inertia(
                ixx=float(inertia_transformed[0, 0]),
                ixy=float(inertia_transformed[0, 1]),
                ixz=float(inertia_transformed[0, 2]),
                iyy=float(inertia_transformed[1, 1]),
                iyz=float(inertia_transformed[1, 2]),
                izz=float(inertia_transformed[2, 2]),
            ),
        ),
        collision=urdf.CollisionLink(
            origin=mesh_origin,
            geometry=urdf.MeshGeometry(filename=urdf_file_path),
        ),
    )

    return urdf_part_link, stl_origin_to_part_tf


def get_urdf_joint(
    doc: CheckedDocument,
    joint: Joint,
    parent_stl_origin_to_part_tf: np.ndarray,
    *,
    config: DownloadConfig | None = None,
) -> urdf.BaseJoint:
    """Returns the URDF joint.

    Args:
        doc: The checked document.
        joint: The joint to convert.
        parent_stl_origin_to_part_tf: The transformation matrix from the parent
            STL origin to the parent part frame.
        config: The converter configuration.

    Returns:
        The URDF link and joint.
    """
    if config is None:
        config = DownloadConfig()

    default_prismatic_joint_limits = urdf.JointLimits(*config.default_prismatic_joint_limits)
    default_revolute_joint_limits = urdf.JointLimits(*config.default_revolute_joint_limits)

    suffix_to_joint_effort = [(k.lower().strip(), v) for k, v in config.suffix_to_joint_effort.items()]
    suffix_to_joint_velocity = [(k.lower().strip(), v) for k, v in config.suffix_to_joint_velocity.items()]

    parent_part_to_mate_tf = joint.parent_entity.matedCS.part_to_mate_tf
    parent_stl_origin_to_mate_tf = parent_stl_origin_to_part_tf @ parent_part_to_mate_tf

    # Gets the joint limits.
    joint_assembly_id, feature_id = doc.key_to_euid[joint.joint_key[:-1]], joint.joint_key[-1]
    joint_info_key = ElementUid(
        document_id=joint_assembly_id.document_id,
        document_microversion=joint_assembly_id.document_microversion,
        element_id=joint_assembly_id.element_id,
        part_id=feature_id,
    )
    joint_limits = doc.joint_limits[joint_info_key]
    expression_resolver = ExpressionResolver(joint_assembly_id.configuration)

    def resolve(expression: str | None) -> float | None:
        return None if expression is None else expression_resolver.read_expression(expression)

    def get_effort_and_velocity(name: str, default_effort: float, default_velocity: float) -> tuple[float, float]:
        effort = default_effort
        for suffix, value in suffix_to_joint_effort:
            if name.lower().endswith(suffix):
                effort = value
                break
        velocity = default_velocity
        for suffix, value in suffix_to_joint_velocity:
            if name.lower().endswith(suffix):
                velocity = value
                break
        return effort, velocity

    name = doc.key_namer(joint.joint_key, "joint")
    origin = urdf.Origin.from_matrix(parent_stl_origin_to_mate_tf)
    mate_type = joint.mate_type

    match mate_type:
        case MateType.FASTENED:
            parent, child = doc.key_namer(joint.parent, "link"), doc.key_namer(joint.child, "link")
            return urdf.FixedJoint(
                name=name,
                parent=parent,
                child=child,
                origin=origin,
            )

        case MateType.REVOLUTE:
            parent, child = doc.key_namer(joint.parent, "link"), doc.key_namer(joint.child, "link")
            mimic_joint = doc.mate_relations.get(joint.joint_key)

            min_value = resolve(joint_limits.axial_z_min_expression)
            max_value = resolve(joint_limits.axial_z_max_expression)

            if min_value is None or max_value is None:
                raise ValueError(f"Revolute joint {name} ({parent} -> {child}) does not have limits defined.")

            effort, velocity = get_effort_and_velocity(
                name,
                default_revolute_joint_limits.effort,
                default_revolute_joint_limits.velocity,
            )

            return urdf.RevoluteJoint(
                name=name,
                parent=parent,
                child=child,
                origin=origin,
                axis=urdf.Axis((0.0, 0.0, -1.0)),
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
                        joint=doc.key_namer(mimic_joint.parent, "joint"),
                        multiplier=mimic_joint.multiplier,
                        offset=0.0,
                    )
                ),
            )

        case MateType.SLIDER:
            parent, child = doc.key_namer(joint.parent, "link"), doc.key_namer(joint.child, "link")
            mimic_joint = doc.mate_relations.get(joint.joint_key)

            min_value = resolve(joint_limits.z_min_expression)
            max_value = resolve(joint_limits.z_max_expression)

            if min_value is None or max_value is None:
                raise ValueError(f"Slider joint {name} ({parent} -> {child}) does not have limits defined.")

            effort, velocity = get_effort_and_velocity(
                name,
                default_prismatic_joint_limits.effort,
                default_prismatic_joint_limits.velocity,
            )

            return urdf.PrismaticJoint(
                name=name,
                parent=parent,
                child=child,
                origin=origin,
                axis=urdf.Axis((0.0, 0.0, -1.0)),
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
                        joint=doc.key_namer(mimic_joint.parent, "joint"),
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


@dataclass
class SavedUrdf:
    urdf_path: Path
    mesh_paths: list[Path]


async def save_urdf(
    doc: CheckedDocument,
    output_dir: Path,
    api: OnshapeApi,
    *,
    config: DownloadConfig | None = None,
) -> SavedUrdf:
    """Saves the URDF files for the document.

    Args:
        doc: The checked document to save.
        output_dir: The output directory.
        api: The Onshape API to use.
        config: The converter configuration.

    Returns:
        The path to the saved URDF file and mesh files.
    """
    if config is None:
        config = DownloadConfig()
    mesh_dir_name = config.mesh_dir

    # Gets the root link in the URDF.
    urdf_parts: list[urdf.Link | urdf.BaseJoint] = []
    part_link, root_stl_origin_to_part_tf = get_urdf_part(doc, doc.central_node, mesh_dir_name)
    urdf_parts.append(part_link)

    # Keeps track of the parent STL origin to part transformation matrices.
    stl_origin_to_part_tfs: dict[Key, np.ndarray] = {doc.central_node: root_stl_origin_to_part_tf}

    for joint in doc.joints:
        urdf_joint = get_urdf_joint(doc, joint, stl_origin_to_part_tfs[joint.parent], config=config)
        urdf_link, stl_origin_to_part_tf = get_urdf_part(doc, joint.child, mesh_dir_name, joint)
        stl_origin_to_part_tfs[joint.child] = stl_origin_to_part_tf
        urdf_parts.extend([urdf_joint, urdf_link])

    robot_name = clean_name(str(doc.assembly_metadata.property_map.get("Name", "robot"))).lower()
    urdf_path = output_dir / f"{robot_name}.urdf"
    urdf_robot = urdf.Robot(name=robot_name, parts=urdf_parts)
    urdf_robot.save(urdf_path)

    # Finally, downloads the STL files.
    (mesh_dir := output_dir / mesh_dir_name).mkdir(parents=True, exist_ok=True)
    mesh_paths = await asyncio.gather(
        *(
            download_stl(
                doc,
                key,
                mesh_dir / f"{doc.key_namer(key, None)}.stl",
                api,
                stl_origin_to_part_tf,
                min_facet_width=config.min_facet_width,
            )
            for key, stl_origin_to_part_tf in stl_origin_to_part_tfs.items()
        )
    )

    return SavedUrdf(
        urdf_path=urdf_path,
        mesh_paths=mesh_paths,
    )


@dataclass
class DownloadedDocument:
    check_document: CheckedDocument
    urdf_info: SavedUrdf


async def download(
    document_url: str,
    output_dir: str | Path,
    *,
    config: DownloadConfig | None = None,
    api: OnshapeApi | None = None,
) -> DownloadedDocument:
    """Converts an Onshape document to a URDF.

    Args:
        document_url: The URL of the Onshape document.
        output_dir: The output directory.
        config: The converter configuration.
        api: The Onshape API to use.

    Returns:
        The downloaded document information.
    """
    output_dir = Path(output_dir).expanduser().resolve()
    if config is None:
        config = DownloadConfig()

    # Creates directories for storing cached artifacts and meshes.
    (cache_dir := output_dir / ".cache").mkdir(parents=True, exist_ok=True)

    cacher = Cacher(
        cache_dir,
        invalidate_after_n_minutes=config.invalidate_cache_after_n_minutes,
    )

    if api is None:
        api = CachedOnshapeApi(OnshapeClient(), cacher)

    document = api.parse_url(document_url)

    # Runs document pre-checks.
    checked_document = await check_document(document, api, config=config)

    # Converts the checked document to a URDF.
    urdf_info = await save_urdf(checked_document, output_dir, api, config=config)

    return DownloadedDocument(
        check_document=checked_document,
        urdf_info=urdf_info,
    )


async def main(args: Sequence[str] | None = None) -> DownloadedDocument:
    if args is None:
        args = sys.argv[1:]
    config = DownloadConfig.from_cli_args(args)
    configure_logging(level=logging.DEBUG if config.debug else logging.INFO)
    return await download(
        document_url=config.document_url,
        output_dir=config.output_dir,
        config=config,
    )


def sync_main(args: Sequence[str] | None = None) -> None:
    asyncio.run(main(args))


if __name__ == "__main__":
    # python -m kol.onshape.convert
    sync_main()
