# mypy: disable-error-code="attr-defined"
"""Defines utility functions for importing the robot model from OnShape."""

import argparse
import datetime
import hashlib
import itertools
import logging
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence, TypeVar, cast

import networkx as nx
import numpy as np

from kol import urdf
from kol.logging import configure_logging
from kol.onshape.api import OnshapeApi, WorkspaceType
from kol.onshape.client import OnshapeClient
from kol.onshape.schema.assembly import (
    Assembly,
    AssemblyMetadata,
    Key,
    MatedEntity,
    MateType,
    RelationType,
    RootAssembly,
    SubAssembly,
    clean_name,
)
from kol.onshape.schema.common import ElementUid
from kol.onshape.schema.features import Feature, Features
from kol.onshape.schema.part import PartDynamics, PartMetadata
from kol.resolvers import ExpressionResolver

logger = logging.getLogger(__name__)

T = TypeVar("T")

Color = tuple[int, int, int, int]

DEFAULT_COLOR: Color = (0, 0, 255, 255)  # Blue


def b_to_a_tf(a_to_b_tf: np.matrix) -> np.matrix:
    return np.matrix(np.linalg.inv(a_to_b_tf))


def get_link_for_part(
    assembly: Assembly,
    key: Key,
    part_colors: dict[ElementUid, Color],
    part_file_names: dict[ElementUid, str],
    part_dynamics: dict[ElementUid, PartDynamics],
    mate_to_part_tf: np.matrix,
) -> urdf.Link:
    part_instance = assembly.key_to_part_instance[key]
    part_file_name = f"package:///meshes/{part_file_names[part_instance.key]}"
    part_color = part_colors[part_instance.key]
    part_dynamic = part_dynamics[part_instance.key].bodies[part_instance.partId]

    # Move the mesh origin and dynamics from the part frame to the parent
    # joint frame (since URDF expects this by convention).
    mesh_origin = urdf.Origin.from_matrix(mate_to_part_tf)
    center_of_mass = part_dynamic.center_of_mass_in_frame(mate_to_part_tf)
    inertia = part_dynamic.inertia_in_frame(mate_to_part_tf)

    name = assembly.key_name(key, "link")
    part_link = urdf.Link(
        name=name,
        visual=urdf.VisualLink(
            origin=mesh_origin,
            geometry=urdf.MeshGeometry(filename=part_file_name),
            material=urdf.Material(
                name=f"{name}_material",
                color=[c / 255.0 for c in part_color],
            ),
        ),
        inertial=urdf.InertialLink(
            origin=urdf.Origin(
                xyz=center_of_mass,
                rpy=(0.0, 0.0, 0.0),
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
            geometry=urdf.MeshGeometry(filename=part_file_name),
        ),
    )
    return part_link


def get_or_use_cached(
    cache_dir: Path | None,
    cache_name: str,
    get_fn: Callable[[], T],
    from_json_fn: Callable[[str], T],
    to_json_fn: Callable[[T], str],
    invalidate_after_n_minutes: int | None = None,
) -> T:
    cache_path: Path | None = None
    if cache_dir is not None:
        cache_path = cache_dir / f"{cache_name}.json"
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
    if cache_path is not None:
        with open(cache_path, "w") as f:
            f.write(to_json_fn(item))
    return item


def get_assembly_id(
    assembly_id: str | None,
    document_id: str,
    item_id: str,
    item_type: WorkspaceType,
    api: OnshapeApi,
) -> str:
    if assembly_id is not None:
        return assembly_id
    return api.get_first_assembly_id(document_id, item_id, item_type)


def download_or_get_cached_features(
    cache_dir: Path | None,
    assembly: RootAssembly | SubAssembly,
    api: OnshapeApi,
) -> Features:
    return get_or_use_cached(
        cache_dir,
        f"assembly_{assembly.key.unique_id}",
        lambda: api.get_features(assembly),
        lambda feats_str: Features.model_validate_json(feats_str),
        lambda feats: feats.model_dump_json(indent=2),
    )


def get_directories(
    output_dir: str | Path | None,
    ignore_cache: bool,
    clear_cache: bool,
    clear_meshes: bool,
) -> tuple[Path, Path | None, Path]:
    output_dir = Path.cwd() / "robot" if output_dir is None else Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Create a cache directory.
    cache_dir: Path | None = None
    if not ignore_cache:
        (cache_dir := output_dir / ".cache").mkdir(exist_ok=True)
        if clear_cache:
            for cache_file in cache_dir.glob("*"):
                cache_file.unlink()

    # Clear the meshes directory if requested.
    meshes_dir = output_dir / "meshes"
    if clear_meshes:
        if meshes_dir.exists():
            for mesh_file in meshes_dir.glob("*"):
                mesh_file.unlink()

    return output_dir, cache_dir, meshes_dir


def resolve_lookup_id(
    document_id_or_url: str,
    workspace_id: str | None,
    version_id: str | None,
    assembly_id: str | None,
    api: OnshapeApi,
) -> tuple[str, WorkspaceType, str, str]:
    # See if the input is a URL.
    url_matches = re.match(r".+?/documents/([\w\d]+)/(w|v)/([\w\d]+)/e/([\w\d]+)", document_id_or_url)
    if url_matches is not None:
        document_id = url_matches.group(1)
        item_kind = cast(WorkspaceType, url_matches.group(2))
        item_id = url_matches.group(3)
        element_id = url_matches.group(4)
        return document_id, item_kind, item_id, element_id

    # Treat the input as a document ID.
    if workspace_id is None and version_id is None:
        workspace_id = api.get_document(document_id_or_url).defaultWorkspace.id
        logger.info("Using default workspace %s", workspace_id)
    if workspace_id is not None:
        if assembly_id is None:
            assembly_id = api.get_first_assembly_id(document_id, workspace_id, "w")
        return document_id_or_url, "w", workspace_id, assembly_id
    if version_id is not None:
        if assembly_id is None:
            assembly_id = api.get_first_assembly_id(document_id, version_id, "v")
        return document_id_or_url, "v", version_id, assembly_id
    raise ValueError("Either `workspace_id` or `version_id` must be provided")


def get_assembly_features(
    cache_dir: Path | None,
    assembly: Assembly,
    api: OnshapeApi,
) -> dict[ElementUid, Features]:
    assembly_features: dict[ElementUid, Features] = {
        assembly.rootAssembly.key: download_or_get_cached_features(cache_dir, assembly.rootAssembly, api)
    }
    for sub_assembly in assembly.subAssemblies:
        if sub_assembly.key in assembly_features:
            continue
        assembly_features[sub_assembly.key] = download_or_get_cached_features(cache_dir, sub_assembly, api)
    return assembly_features


@dataclass
class JointInformation:
    z_min_expression: str | None = None
    z_max_expression: str | None = None
    axial_z_min_expression: str | None = None
    axial_z_max_expression: str | None = None


def get_joint_information(
    cache_dir: Path | None,
    assembly: Assembly,
    api: OnshapeApi,
) -> dict[tuple[str, str, str, str], JointInformation]:
    # Retrieves the features for the root assembly and each subassembly.
    assembly_features = get_assembly_features(cache_dir, assembly, api)

    def get_feature_value(key: str, feature: Feature) -> str | None:
        if (attrib := feature.message.parameter_dict.get(key)) is None:
            return None
        match attrib["typeName"]:
            case "BTMParameterNullableQuantity":
                return None if attrib["message"]["isNull"] else attrib["message"]["expression"]
            case _:
                return None

    # Gets the joint information for each feature from the assembly features.
    joint_information: dict[tuple[str, str, str, str], JointInformation] = {}
    for assembly_key, assembly_feature in assembly_features.items():
        for feature in assembly_feature.features:
            key = (
                assembly_key.document_id,
                assembly_key.document_microversion,
                assembly_key.element_id,
                feature.message.featureId,
            )

            if (
                "limitsEnabled" not in feature.message.parameter_dict
                or not feature.message.parameter_dict["limitsEnabled"]["message"]["value"]
            ):
                joint_information[key] = JointInformation()
                continue

            joint_information[key] = JointInformation(
                z_min_expression=get_feature_value("limitZMin", feature),
                z_max_expression=get_feature_value("limitZMax", feature),
                axial_z_min_expression=get_feature_value("limitAxialZMin", feature),
                axial_z_max_expression=get_feature_value("limitAxialZMax", feature),
            )

    return joint_information


def download_parts(
    cache_dir: Path | None,
    meshes_dir: Path,
    assembly: Assembly,
    api: OnshapeApi,
) -> tuple[dict[ElementUid, str], dict[ElementUid, Color], dict[ElementUid, PartDynamics]]:
    part_file_names: dict[ElementUid, str] = {}
    part_colors: dict[ElementUid, tuple[int, int, int, int]] = {}
    part_dynamics: dict[ElementUid, PartDynamics] = {}

    for part in assembly.parts:
        # Gets the part color.
        part_metadata = get_or_use_cached(
            cache_dir,
            f"part_{part.key.unique_id}_metadata",
            lambda: api.get_part_metadata(part),
            lambda part_str: PartMetadata.model_validate_json(part_str),
            lambda part: part.model_dump_json(indent=2),
        )
        part_color = part_metadata.property_map.get("Appearance")
        if part_color is None:
            part_colors[part.key] = DEFAULT_COLOR
        else:
            part_colors[part.key] = (
                part_color["color"]["red"],
                part_color["color"]["green"],
                part_color["color"]["blue"],
                part_color["opacity"],
            )

        # Gets the part's dynamics.
        part_mass_properties = get_or_use_cached(
            cache_dir,
            f"part_{part.key.unique_id}_mass_properties",
            lambda: api.get_part_mass_properties(part),
            lambda props_str: PartDynamics.model_validate_json(props_str),
            lambda props: props.model_dump_json(indent=2),
        )
        part_dynamics[part.key] = part_mass_properties

        # Gets the official part name from the part metadata.
        part_prop_name = part_metadata.property_map.get("Name")
        part_name = clean_name(part_prop_name if isinstance(part_prop_name, str) else part.key.unique_id)

        # Gets the configuration string suffix.
        if part.configuration == "default":
            configuration_str = ""
        elif len(part.configuration) > 40:
            configuration_str = hashlib.md5(part.configuration.encode()).hexdigest()[:16]
        else:
            configuration_str = part.configuration

        part_hash = hashlib.md5(part.key.unique_id.encode()).hexdigest()[:8]

        part_file_name = f"{part_name}{configuration_str}.{part_hash}.stl"
        part_file_names[part.key] = part_file_name

        # Saves the STL file for the part.
        output_path = meshes_dir / part_file_name
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not output_path.exists():
            logger.info("Downloading STL file to %s", output_path)
            api.download_stl(part, output_path)
        else:
            logger.info("Using cached STL file %s", output_path)

    # Ensure that none of the features use assembly instances.
    for path, mate_feature in assembly.key_to_mate_feature.items():
        for mate_pair in itertools.combinations(mate_feature.featureData.matedEntities, 2):
            for mate_i in mate_pair:
                mate_path = path[:-1] + mate_i.key
                if mate_path not in assembly.key_to_part_instance:
                    raise ValueError(
                        f'Feature "{" / ".join(assembly.key_to_name.get(path, path))}" connects to an assembly '
                        f'"{" / ".join(assembly.key_to_name.get(mate_path, mate_path))}", which is not supported'
                    )

    return part_file_names, part_colors, part_dynamics


def get_assembly_graph(assembly: Assembly) -> nx.Graph:
    graph = nx.Graph()
    for path, _ in assembly.key_to_part_instance.items():
        graph.add_node(path)

    def add_edge_safe(node_a: Key, node_b: Key, name: str) -> None:
        if node_a not in graph:
            raise ValueError(f"Node {assembly.key_to_name.get(node_a, node_a)} not found in graph")
        if node_b not in graph:
            raise ValueError(f"Node {assembly.key_to_name.get(node_b, node_b)} not found in graph")
        graph.add_edge(node_a, node_b, name=name)

    # Add edges between nodes that have a feature connecting them.
    for path, mate_feature in assembly.key_to_mate_feature.items():
        for mate_pair in itertools.combinations(mate_feature.featureData.matedEntities, 2):
            name = " / ".join(assembly.key_to_name.get(path, path))
            add_edge_safe(path[:-1] + mate_pair[0].key, path[:-1] + mate_pair[1].key, name)

    # Logs all of the nodes and edges in the graph.
    for edge in graph.edges:
        logger.debug(
            'Edge: Part "%s" -> Path "%s" (Feature "%s")',
            " / ".join(assembly.key_to_name.get(edge[0], edge[0])),
            " / ".join(assembly.key_to_name.get(edge[1], edge[1])),
            graph.edges[edge]["name"],
        )

    # If there are any unconnected nodes in the graph, raise an error.
    if not nx.is_connected(graph):
        num_components = nx.number_connected_components(graph)

        components: list[str] = []
        for component in nx.connected_components(graph):
            component_list = sorted([" / ".join(assembly.key_to_name.get(c, c)) for c in component])
            components.append("\n      ".join(component_list))
        components_string = "\n\n".join(f"  {i + 1: <3} {component}" for i, component in enumerate(components))

        raise ValueError(
            "The assembly is not fully connected! URDF export requires a single fully-connected robot, "
            f"but the graph has {num_components} components. Components:\n{components_string}"
        )

    if not nx.is_tree(graph):
        # Detects the parallel connections in the graph.
        for cycle in nx.cycle_basis(graph):
            for i, _ in enumerate(cycle):
                if i == len(cycle) - 1:
                    break
                logger.error(
                    "Parallel connection: %s -> %s",
                    " / ".join(assembly.key_to_name.get(cycle[i], cycle[i])),
                    " / ".join(assembly.key_to_name.get(cycle[i + 1], cycle[i + 1])),
                )
        raise ValueError("The assembly has parallel connections! URDF export requires no parallel connections.")

    return graph


Joint = tuple[Key, Key, MatedEntity, MatedEntity, MateType, Key]


def get_central_node_and_ordered_joint_list(assembly: Assembly) -> tuple[Key, list[Joint]]:
    graph = get_assembly_graph(assembly)

    # Gets the most central node in the graph.
    closeness_centrality = nx.closeness_centrality(graph)
    central_node: Key = max(closeness_centrality, key=closeness_centrality.get)
    logger.debug("Central node: %s", " / ".join(assembly.key_to_name.get(central_node, central_node)))

    # Get BFS ordering from the central node to establish parent-child.
    digraph = nx.bfs_tree(graph, central_node)
    bfs_node_ordering = list(nx.topological_sort(digraph))
    node_level = {node: i for i, node in enumerate(bfs_node_ordering)}

    # Creates a topologically-sorted list of joints.
    joint_list: list[Joint] = []
    for joint_key, mate_feature in assembly.key_to_mate_feature.items():
        lhs_entity, rhs_entity = mate_feature.featureData.matedEntities
        lhs_key, rhs_key = joint_key[:-1] + lhs_entity.key, joint_key[:-1] + rhs_entity.key
        lhs_is_first = node_level[lhs_key] < node_level[rhs_key]
        parent_key, child_key = (lhs_key, rhs_key) if lhs_is_first else (rhs_key, lhs_key)
        parent_entity, child_entity = (lhs_entity, rhs_entity) if lhs_is_first else (rhs_entity, lhs_entity)
        mate_type = mate_feature.featureData.mateType
        joint_list.append((parent_key, child_key, parent_entity, child_entity, mate_type, joint_key))
    joint_list.sort(key=lambda x: (node_level[x[0]], node_level[x[1]]))

    return central_node, joint_list


@dataclass
class MimicRelation:
    parent: Key
    multiplier: float


def get_relations(assembly: Assembly) -> dict[Key, MimicRelation]:
    relations: dict[Key, MimicRelation] = {}
    for path, mate_relation_feature in assembly.key_to_mate_relation_feature.items():
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


def get_joint(
    assembly: Assembly,
    parent_key: Key,
    child_key: Key,
    mate_type: MateType,
    relations: dict[Key, MimicRelation],
    origin: urdf.Origin,
    info: JointInformation,
    resolver: ExpressionResolver,
    default_prismatic_joint_limits: urdf.JointLimits,
    default_revolute_joint_limits: urdf.JointLimits,
    joint_key: Key,
    name: str,
) -> urdf.BaseJoint:
    def resolve(expression: str | None) -> float | None:
        return None if expression is None else resolver.read_expression(expression)

    match mate_type:
        case MateType.FASTENED:
            parent, child = assembly.key_name(parent_key, "link"), assembly.key_name(child_key, "link")
            return urdf.FixedJoint(
                name=name,
                parent=parent,
                child=child,
                origin=origin,
            )

        case MateType.REVOLUTE:
            parent, child = assembly.key_name(parent_key, "link"), assembly.key_name(child_key, "link")
            mimic_joint = relations.get(joint_key)

            min_value = resolve(info.axial_z_min_expression)
            max_value = resolve(info.axial_z_max_expression)

            if min_value is None or max_value is None:
                raise ValueError(f"Revolute joint {name} ({parent} -> {child}) does not have limits defined.")

            return urdf.RevoluteJoint(
                name=name,
                parent=parent,
                child=child,
                origin=origin,
                axis=urdf.Axis((0.0, 0.0, 1.0)),
                limits=urdf.JointLimits(
                    effort=default_revolute_joint_limits.effort,
                    velocity=default_revolute_joint_limits.velocity,
                    lower=min_value,
                    upper=max_value,
                ),
                mimic=(
                    None
                    if mimic_joint is None
                    else urdf.JointMimic(
                        joint=assembly.key_name(mimic_joint.parent, "joint"),
                        multiplier=mimic_joint.multiplier,
                        offset=0.0,
                    )
                ),
            )

        case MateType.SLIDER:
            parent, child = assembly.key_name(parent_key, "link"), assembly.key_name(child_key, "link")
            mimic_joint = relations.get(joint_key)

            min_value = resolve(info.z_min_expression)
            max_value = resolve(info.z_max_expression)

            if min_value is None or max_value is None:
                raise ValueError(f"Slider joint {name} ({parent} -> {child}) does not have limits defined.")

            return urdf.PrismaticJoint(
                name=name,
                parent=parent,
                child=child,
                origin=origin,
                axis=urdf.Axis((0.0, 0.0, 1.0)),
                limits=urdf.JointLimits(
                    effort=default_prismatic_joint_limits.effort,
                    velocity=default_prismatic_joint_limits.velocity,
                    lower=min_value,
                    upper=max_value,
                ),
                mimic=(
                    None
                    if mimic_joint is None
                    else urdf.JointMimic(
                        joint=assembly.key_name(mimic_joint.parent, "joint"),
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


def import_onshape(
    document_id_or_url: str,
    assembly_id: str | None = None,
    workspace_id: str | None = None,
    version_id: str | None = None,
    output_dir: str | Path | None = None,
    api: OnshapeApi | None = None,
    ignore_cache: bool = False,
    clear_cache: bool = False,
    clear_meshes: bool = False,
    default_prismatic_joint_limits: urdf.JointLimits = urdf.JointLimits(80.0, 5.0, -1.0, 1.0),
    default_revolute_joint_limits: urdf.JointLimits = urdf.JointLimits(80.0, 5.0, -np.pi, np.pi),
    configuration: str = "default",
) -> None:
    """Import the robot model from OnShape.

    Args:
        document_id_or_url: The ID of the document to import.
        assembly_id: The ID of the assembly to import.
        workspace_id: The ID of the workspace to import.
        version_id: The ID of the version to import.
        output_dir: The directory to save the imported model.
        api: The OnShape API to use for the import.
        ignore_cache: If set, ignore cached files.
        clear_cache: If set, clear the cache before importing.
        clear_meshes: If set, clear the meshes before importing.
        default_prismatic_joint_limits: The default limits for prismatic joints.
        default_revolute_joint_limits: The default limits for revolute joints.
        configuration: The configuration to use for the import.
    """
    output_dir, cache_dir, meshes_dir = get_directories(output_dir, ignore_cache, clear_cache, clear_meshes)

    # Construct with default parameters if no client was provided.
    api = OnshapeApi(OnshapeClient()) if api is None else api

    # Resolves the workspace ID and type.
    document_id, item_type, item_id, assembly_id = resolve_lookup_id(
        document_id_or_url=document_id_or_url,
        workspace_id=workspace_id,
        version_id=version_id,
        assembly_id=assembly_id,
        api=api,
    )

    # Retrieves the assembly.
    assembly = get_or_use_cached(
        cache_dir,
        "assembly",
        lambda: api.get_assembly(document_id, item_id, assembly_id, item_type, configuration),
        lambda asm_str: Assembly.model_validate_json(asm_str),
        lambda asm: asm.model_dump_json(indent=2),
        invalidate_after_n_minutes=1,
    )

    # Gets the assembly metadata.
    assembly_metadata = get_or_use_cached(
        cache_dir,
        "assembly_metadata",
        lambda: api.get_assembly_metadata(assembly.rootAssembly),
        lambda asm_str: AssemblyMetadata.model_validate_json(asm_str),
        lambda asm: asm.model_dump_json(indent=2),
    )

    # Downloads the STL files for each part in the assembly.
    part_file_names, part_colors, part_dynamics = download_parts(cache_dir, meshes_dir, assembly, api)

    # Gets the central node and ordered joint list.
    joint_information = get_joint_information(cache_dir, assembly, api)
    central_node, joint_list = get_central_node_and_ordered_joint_list(assembly)

    # Gets the mimic relations for the joints.
    relations = get_relations(assembly)

    # Link and joint lists for the final URDF.
    links: list[urdf.Link] = []
    joints: list[urdf.BaseJoint] = []

    # Because of the URDF parent-child relationship constraint, each part frame
    # should have it's origin at the parent joint's origin. The first node
    # can just use the world frame as the parent joint origin.
    world_to_mate_tfs: dict[Key, np.matrix] = {}
    world_to_mate_tfs[central_node] = np.matrix(np.eye(4))

    def get_link_for_part_by_key(key: Key, mate_to_part_tf: np.matrix) -> urdf.Link:
        return get_link_for_part(
            assembly,
            key,
            part_colors,
            part_file_names,
            part_dynamics,
            mate_to_part_tf,
        )

    # Add the first link, since it has no incoming joint.
    central_node_mate_to_world_tf = assembly.key_to_occurrence[central_node].world_to_part_tf
    part_link = get_link_for_part_by_key(central_node, central_node_mate_to_world_tf)
    links.append(part_link)

    # Creates a URDF joint for each feature connecting two parts.
    for parent_key, child_key, _, child_entity, mate_type, joint_key in joint_list:
        child_occurrence = assembly.key_to_occurrence[child_key]
        child_world_to_part_tf = child_occurrence.world_to_part_tf
        child_part_to_mate_tf = child_entity.matedCS.part_to_mate_tf
        child_world_to_mate_tf = child_world_to_part_tf @ child_part_to_mate_tf
        world_to_mate_tfs[child_key] = child_world_to_mate_tf
        parent_mate_to_child_mate_tf = b_to_a_tf(world_to_mate_tfs[parent_key]) @ child_world_to_mate_tf

        # Gets the joint limits.
        joint_assembly_id, feature_id = assembly.assembly_key_to_id[joint_key[:-1]], joint_key[-1]
        joint_info_key = (
            joint_assembly_id.document_id,
            joint_assembly_id.document_microversion,
            joint_assembly_id.element_id,
            feature_id,
        )
        joint_info = joint_information[joint_info_key]
        expression_resolver = ExpressionResolver(joint_assembly_id.configuration)

        # Adds the link for the part.
        part_link = get_link_for_part_by_key(child_key, b_to_a_tf(child_part_to_mate_tf))
        links.append(part_link)

        # Adds the joint for the parent.
        joint_origin = urdf.Origin.from_matrix(parent_mate_to_child_mate_tf)
        joint_name = assembly.key_name(joint_key, "joint")
        joint = get_joint(
            assembly=assembly,
            parent_key=parent_key,
            child_key=child_key,
            mate_type=mate_type,
            relations=relations,
            origin=joint_origin,
            info=joint_info,
            resolver=expression_resolver,
            default_prismatic_joint_limits=default_prismatic_joint_limits,
            default_revolute_joint_limits=default_revolute_joint_limits,
            joint_key=joint_key,
            name=joint_name,
        )
        joints.append(joint)

    # Saves the final URDF.
    robot_name = clean_name(str(assembly_metadata.property_map.get("Name", "robot")))
    robot = urdf.Robot(name=robot_name, links=links, joints=joints)
    robot.save(output_dir / f"{robot_name}.urdf")


def main(args: Sequence[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Import the robot model from OnShape")
    parser.add_argument("document_id_or_url", type=str, help="The ID of the document to import")
    parser.add_argument("-a", "--assembly-id", type=str, help="The ID of the assembly to import")
    parser.add_argument("-w", "--workspace-id", type=str, help="The ID of the workspace to import")
    parser.add_argument("-v", "--version-id", type=str, help="The ID of the version to import")
    parser.add_argument("-o", "--output-dir", type=str, help="The path to save the imported model")
    parser.add_argument("--ignore-cache", action="store_true", help="Ignore the cache when importing")
    parser.add_argument("--clear-cache", action="store_true", help="Clear the cache before importing")
    parser.add_argument("--clear-meshes", action="store_true", help="Clear the meshes before importing")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    parser.add_argument("--max-force", type=float, default=80.0, help="The maximum force for a prismatic joint")
    parser.add_argument("--max-velocity", type=float, default=5.0, help="The maximum velocity for a prismatic joint")
    parser.add_argument("--max-length", type=float, default=1.0, help="The maximum length for a prismatic joint")
    parser.add_argument("--max-torque", type=float, default=80.0, help="The maximum force for a revolute joint")
    parser.add_argument("--max-ang-velocity", type=float, default=5.0, help="The maximum velocity for a revolute joint")
    parser.add_argument("--max-angle", type=float, default=np.pi, help="The maximum angle for a revolute joint")
    parsed_args = parser.parse_args(args)

    configure_logging(level=logging.DEBUG if parsed_args.debug else logging.INFO)

    import_onshape(
        document_id_or_url=parsed_args.document_id_or_url,
        assembly_id=parsed_args.assembly_id,
        workspace_id=parsed_args.workspace_id,
        version_id=parsed_args.version_id,
        output_dir=parsed_args.output_dir,
        ignore_cache=parsed_args.ignore_cache,
        clear_cache=parsed_args.clear_cache,
        clear_meshes=parsed_args.clear_meshes,
        default_prismatic_joint_limits=urdf.JointLimits(
            effort=parsed_args.max_force,
            velocity=parsed_args.max_velocity,
            lower=-parsed_args.max_length,
            upper=parsed_args.max_length,
        ),
        default_revolute_joint_limits=urdf.JointLimits(
            effort=parsed_args.max_torque,
            velocity=parsed_args.max_ang_velocity,
            lower=-parsed_args.max_angle,
            upper=parsed_args.max_angle,
        ),
    )


if __name__ == "__main__":
    # python -m kol.scripts.import_onshape
    main()
