# mypy: disable-error-code="attr-defined"
"""Defines shared functions."""

import asyncio
import datetime
import hashlib
import io
import itertools
import logging
import re
import traceback
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import (
    Any,
    AsyncIterator,
    Callable,
    Coroutine,
    Deque,
    Literal,
    Self,
    Sequence,
    TypeVar,
    cast,
)

import asyncstdlib
import networkx as nx
import numpy as np
import stl
from omegaconf import MISSING, OmegaConf
from scipy.spatial.transform import Rotation as R

from kol.cleanup import cleanup_mesh_dir
from kol.formats import mjcf, urdf
from kol.geometry import apply_matrix_, inv_tf, transform_inertia_tensor
from kol.merge_fixed_joints import get_merged_urdf
from kol.mesh import get_mesh_type, stl_to_fmt
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
    MateGroupFeature,
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
from kol.onshape.schema.features import Feature, Features, FeatureStatus
from kol.onshape.schema.part import PartDynamics, PartMetadata
from kol.resolvers import ExpressionResolver
from kol.simplify_all import simplify_all
from kol.update_joints import update_joints

logger = logging.getLogger(__name__)

T = TypeVar("T")
Tk = TypeVar("Tk")
Tv = TypeVar("Tv")

Color = tuple[int, int, int, int]


def clean_name(name: str) -> str:
    return re.sub(r"\s+", "_", re.sub(r"[<>]", "", name)).lower()


async def gather_dict(d: dict[Tk, Coroutine[Any, Any, Tv]]) -> dict[Tk, Tv]:
    dict_items = [(k, v) for k, v in d.items()]
    values = await asyncio.gather(*(v for _, v in dict_items))
    return {k: v for (k, _), v in zip(dict_items, values)}


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

    document_url: str = field(
        default=MISSING,
        metadata={"help": "The URL of the Onshape document to convert."},
    )
    output_dir: str | Path | None = field(
        default=None,
        metadata={"help": "The output directory for the URDF files."},
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
    default_color: tuple[int, int, int, int] = field(
        default=(0, 0, 255, 255),
        metadata={"help": "The default color to use for parts."},
    )
    default_mass: float = field(
        default=0.001,
        metadata={"help": "The default mass to use for parts which have their masses missing."},
    )
    debug: bool = field(
        default=False,
        metadata={"help": "Enables debug mode."},
    )

    @classmethod
    def from_cli_args(cls, args: Sequence[str]) -> Self:
        cfg = OmegaConf.structured(cls)
        cli_args: list[str] = []
        for arg in args:
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
        return cfg


class Converter:
    """Defines a utility class for getting document components efficiently."""

    def __init__(
        self,
        config: ConverterConfig,
        *,
        api: OnshapeApi | None = None,
    ) -> None:
        # Gets a default output directory.
        self.output_dir = (Path.cwd() / "robot" if config.output_dir is None else Path(config.output_dir)).resolve()

        # Creates a new directory for cached artifacts.
        self.cache_dir = self.output_dir / ".cache"
        self.cache_dir.mkdir(parents=True, exist_ok=True)

        # Creates a new directory for meshes.
        self.mesh_dir = self.output_dir / "meshes"
        self.mesh_dir.mkdir(parents=True, exist_ok=True)

        self.api = OnshapeApi(OnshapeClient()) if api is None else api
        self.document = self.api.parse_url(config.document_url)
        self.default_prismatic_joint_limits = urdf.JointLimits(*config.default_prismatic_joint_limits)
        self.default_revolute_joint_limits = urdf.JointLimits(*config.default_revolute_joint_limits)
        self.suffix_to_joint_effort = [(k.lower().strip(), v) for k, v in config.suffix_to_joint_effort.items()]
        self.suffix_to_joint_velocity = [(k.lower().strip(), v) for k, v in config.suffix_to_joint_velocity.items()]
        self.disable_mimics = config.disable_mimics
        self.mesh_ext = get_mesh_type(config.mesh_ext)
        self.override_central_node = config.override_central_node
        self.remove_inertia = config.remove_inertia
        self.merge_fixed_joints = config.merge_fixed_joints
        self.simplify_meshes = config.simplify_meshes
        self.override_joint_names = config.override_joint_names
        self.override_nonfixed = config.override_nonfixed
        self.override_limits = config.override_limits
        self.override_torques = config.override_torques
        self.sim_ignore_key = config.sim_ignore_key
        self.voxel_size = config.voxel_size
        self.default_color = tuple(config.default_color)
        self.default_mass = config.default_mass

        # Map containing all cached items.
        self.cache_map: dict[str, Any] = {}

        # Map containing the transformations from the STL origin to the part frame.
        self.stl_origin_to_part_tf: dict[Key, np.ndarray] = {}

    async def get_or_use_cached(
        self,
        cache_key: str,
        get_fn: Callable[[], Coroutine[Any, Any, T]],
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
        item = await get_fn()
        with open(cache_path, "w") as f:
            f.write(to_json_fn(item))
        self.cache_map[cache_key] = item
        return item

    @property
    async def assembly(self) -> Assembly:
        return await self.get_or_use_cached(
            "assembly",
            lambda: self.api.get_assembly(self.document),
            lambda asm_str: Assembly.model_validate_json(asm_str),
            lambda asm: asm.model_dump_json(indent=2),
            invalidate_after_n_minutes=1,
        )

    @property
    async def assembly_metadata(self) -> AssemblyMetadata:
        assembly = await self.assembly
        return await self.get_or_use_cached(
            "assembly_metadata",
            lambda: self.api.get_assembly_metadata(assembly.rootAssembly),
            lambda asm_str: AssemblyMetadata.model_validate_json(asm_str),
            lambda asm: asm.model_dump_json(indent=2),
        )

    async def part_metadata(self, part: Part) -> PartMetadata:
        return await self.get_or_use_cached(
            f"part_{part.key.unique_id}_metadata",
            lambda: self.api.get_part_metadata(part),
            lambda part_str: PartMetadata.model_validate_json(part_str),
            lambda part: part.model_dump_json(indent=2),
        )

    async def part_color(self, part: Part) -> Color:
        part_color = (await self.part_metadata(part)).property_map.get("Appearance")
        if part_color is None:
            return cast(Color, tuple(self.default_color))
        return (
            part_color["color"]["red"],
            part_color["color"]["green"],
            part_color["color"]["blue"],
            part_color["opacity"],
        )

    async def part_dynamics(self, part: Part) -> PartDynamics:
        return await self.get_or_use_cached(
            f"part_{part.key.unique_id}_mass_properties",
            lambda: self.api.get_part_mass_properties(part),
            lambda props_str: PartDynamics.model_validate_json(props_str),
            lambda props: props.model_dump_json(indent=2),
        )

    async def part_name(self, part: Part) -> str:
        part_prop_name = (await self.part_metadata(part)).property_map.get("Name")
        return clean_name(part_prop_name if isinstance(part_prop_name, str) else part.key.unique_id)

    @asyncstdlib.cached_property
    async def euid_to_assembly(self) -> dict[ElementUid, RootAssembly | SubAssembly]:
        assembly = await self.assembly
        assemblies: list[RootAssembly | SubAssembly] = [assembly.rootAssembly, *assembly.subAssemblies]
        return {assembly.key: assembly for assembly in assemblies}

    @asyncstdlib.cached_property
    async def euid_to_subassembly(self) -> dict[ElementUid, SubAssembly]:
        return {sub.key: sub for sub in (await self.assembly).subAssemblies}

    @asyncstdlib.cached_property
    async def euid_to_part(self) -> dict[ElementUid, Part]:
        return {part.key: part for part in (await self.assembly).parts}

    async def traverse_assemblies(self) -> AsyncIterator[tuple[Key, AssemblyInstance, SubAssembly]]:
        subassembly_deque: Deque[tuple[Key, SubAssembly]] = deque()
        visited: set[Key] = set()

        # Adds the root assembly to the traversal.
        for instance in (await self.assembly).rootAssembly.instances:
            if isinstance(instance, AssemblyInstance):
                instance_path: Key = (instance.id,)
                if instance_path in visited:
                    continue
                visited.add(instance_path)
                subassembly = (await self.euid_to_subassembly)[instance.euid]
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
                    subassembly = (await self.euid_to_subassembly)[instance.euid]
                    yield instance_path, instance, subassembly
                    subassembly_deque.append((instance_path, subassembly))

    @asyncstdlib.cached_property
    async def assembly_key_to_id(self) -> dict[Key, ElementUid]:
        key_map = {key: assembly.key async for key, _, assembly in self.traverse_assemblies()}
        key_map[()] = (await self.assembly).rootAssembly.key
        return key_map

    @asyncstdlib.cached_property
    async def key_to_occurrence(self) -> dict[Key, Occurrence]:
        return {occurrence.key: occurrence for occurrence in (await self.assembly).rootAssembly.occurrences}

    @asyncstdlib.cached_property
    async def key_to_instance(self) -> dict[Key, Instance]:
        instance_mapping: dict[Key, Instance] = {}
        for instance in (await self.assembly).rootAssembly.instances:
            instance_mapping[(instance.id,)] = instance
        async for path, assembly_instance, sub_assembly in self.traverse_assemblies():
            instance_mapping[path] = assembly_instance
            for instance in sub_assembly.instances:
                instance_mapping[path + (instance.id,)] = instance
        return instance_mapping

    @property
    async def key_to_part_instance(self) -> dict[Key, PartInstance]:
        return {p: i for p, i in (await self.key_to_instance).items() if isinstance(i, PartInstance)}

    @property
    async def key_to_assembly_instance(self) -> dict[Key, AssemblyInstance]:
        return {p: i for p, i in (await self.key_to_instance).items() if isinstance(i, AssemblyInstance)}

    @asyncstdlib.cached_property
    async def key_to_feature(self) -> dict[Key, MateRelationFeature | MateFeature | MateGroupFeature]:
        feature_mapping: dict[Key, MateRelationFeature | MateFeature | MateGroupFeature] = {}
        for feature in (await self.assembly).rootAssembly.features:
            feature_mapping[(feature.id,)] = feature
        async for key, _, sub_assembly in self.traverse_assemblies():
            for feature in sub_assembly.features:
                feature_mapping[key + (feature.id,)] = feature
        return feature_mapping

    @asyncstdlib.cached_property
    async def key_to_mate_feature(self) -> dict[Key, MateFeature]:
        return {p: f for p, f in (await self.key_to_feature).items() if isinstance(f, MateFeature)}

    @asyncstdlib.cached_property
    async def key_to_mate_relation_feature(self) -> dict[Key, MateRelationFeature]:
        return {p: f for p, f in (await self.key_to_feature).items() if isinstance(f, MateRelationFeature)}

    @asyncstdlib.cached_property
    async def key_to_name(self) -> dict[Key, list[str]]:
        key_name_mapping: dict[Key, list[str]] = {}
        key_name_mapping[()] = []
        for instance in (await self.assembly).rootAssembly.instances:
            key_name_mapping[(instance.id,)] = [instance.name]
        for feature in (await self.assembly).rootAssembly.features:
            key_name_mapping[(feature.id,)] = [feature.featureData.name]
        async for key, assembly_instance, sub_assembly in self.traverse_assemblies():
            key_name_mapping[key] = key_name_mapping[key[:-1]] + [assembly_instance.name]
            for instance in sub_assembly.instances:
                key_name_mapping[key + (instance.id,)] = key_name_mapping[key] + [instance.name]
            for feature in sub_assembly.features:
                key_name_mapping[key + (feature.id,)] = key_name_mapping[key] + [feature.featureData.name]
        return key_name_mapping

    async def key_name(self, key: Key, prefix: Literal["link", "joint", None]) -> str:
        return ("" if prefix is None else f"{prefix}_") + clean_name("_".join((await self.key_to_name).get(key, key)))

    @asyncstdlib.cached_property
    async def name_to_key(self) -> dict[str, Key]:
        return {await self.key_name(key, None): key for key in await self.key_to_name}

    @asyncstdlib.cached_property
    async def graph_and_ignored_joints(self) -> tuple[nx.Graph, set[Key]]:
        """Converts the assembly to an undirected graph of joints and parts.

        This checks that the assembly is connected as a single piece (meaning
        that there are no unconnected parts or joints) and that it is a tree
        (meaning that there are no parallel connections). Parallel connections
        arise when a part has two parents.

        Returns:
            The graph of the assembly, along with a set of the ignored joints.
        """
        graph = nx.Graph()
        graph_none_ignored = nx.Graph()
        sim_ignored_joints: set[Key] = set()

        for key, _ in (await self.key_to_part_instance).items():
            graph_none_ignored.add_node(key)
            if self.sim_ignore_key not in (await self.key_name(key, "joint")).lower():
                graph.add_node(key)
            else:
                logger.info(f"Ignoring joint due to sim_ignore: {self.key_name(key, 'joint')}")
                sim_ignored_joints.add(key)

        async def add_edge_safe(node_a: Key, node_b: Key, name: str) -> None:
            # Here, we are considering a graph with no ignored joints
            # to make sure the original graph is connected.
            for node_lhs, node_rhs in ((node_a, node_b), (node_b, node_a)):
                if node_lhs not in graph_none_ignored:
                    raise ValueError(
                        f"Node {self.key_name(node_lhs, 'link')} (from {self.key_name(node_rhs, 'link')}) not found "
                        "in graph. Do you have a mate between a part and the origin?"
                    )

            graph_none_ignored.add_edge(node_a, node_b, name=name)

            for node_lhs, node_rhs in ((node_a, node_b), (node_b, node_a)):
                if "sim_ignore" in await self.key_name(node_lhs, "link"):
                    logger.info(f"Ignoring link due to sim_ignore: {await self.key_name(node_lhs, 'link')}")
                    return
                if "sim_ignore" in await self.key_name(node_rhs, "link"):
                    logger.info(f"Ignoring link due to sim_ignore: {await self.key_name(node_rhs, 'link')}")
                    return

                if node_lhs not in graph:
                    raise ValueError(
                        f"Node {await self.key_name(node_lhs, 'link')} (from {await self.key_name(node_rhs, 'link')}) "
                        "not found in graph. Do you have a mate between a part and the origin?"
                    )

            graph.add_edge(node_a, node_b, name=name)

        # Add edges between nodes that have a feature connecting them.
        for key, mate_feature in (await self.key_to_mate_feature).items():
            if mate_feature.suppressed:
                continue
            for mate_pair in itertools.combinations(mate_feature.featureData.matedEntities, 2):
                name = await self.key_name(key, "joint")
                await add_edge_safe(key[:-1] + mate_pair[0].key, key[:-1] + mate_pair[1].key, name)

        # Logs all of the nodes and edges in the graph.
        for edge in graph.edges:
            logger.debug(
                'Edge: Part "%s" -> Path "%s" (Feature "%s")',
                await self.key_name(edge[0], "link"),
                await self.key_name(edge[1], "link"),
                graph.edges[edge]["name"],
            )

        # Debug for viewing original URDF graph with possible parallels

        # G_renamed = nx.relabel_nodes(graph_none_ignored, node_relabel_mapping)
        # nx.draw(G_renamed, with_labels=True, node_color='red', node_size=30, edge_color='k', font_size=4)
        # plt.savefig('graph_none_ignored.png', dpi=300)
        # plt.clf()

        # Debug for viewing URDF graph with parallels removed (and possibly disconnected)

        # node_relabel_mapping = {node: self.key_name(node, "joint") for node in graph.nodes()}
        # G_renamed = nx.relabel_nodes(graph, node_relabel_mapping)
        # nx.draw(G_renamed, with_labels=True, node_color='skyblue', node_size=30, edge_color='k', font_size=4)
        # plt.savefig('graph.png', dpi=300)
        # plt.clf()

        components: list[str] = []

        # If there are any unconnected nodes in the graph, raise an error.
        if not nx.is_connected(graph_none_ignored):
            num_components = nx.number_connected_components(graph_none_ignored)
            for component in nx.connected_components(graph_none_ignored):
                component_list = sorted(await asyncio.gather(*(self.key_name(c, "link") for c in component)))
                components.append("\n      ".join(component_list))
            components_string = "\n\n".join(f"  {i + 1: <3} {component}" for i, component in enumerate(components))

            raise ValueError(
                "The assembly is not fully connected! URDF export requires a single fully-connected robot, "
                f"but the graph has {num_components} components. Components:\n{components_string}"
            )

        largest_component = max(nx.connected_components(graph), key=len)

        # Remove smaller components
        for component in list(nx.connected_components(graph)):
            if component != largest_component:
                sim_ignored_joints.add(component)
                graph.remove_nodes_from(component)

        # If there are any unconnected nodes in the graph, raise an error.
        if not nx.is_connected(graph):
            num_components = nx.number_connected_components(graph)
            components.clear()
            for component in nx.connected_components(graph):
                component_list = sorted([await self.key_name(c, "link") for c in component])
                components.append("\n      ".join(component_list))
            components_string = "\n\n".join(f"  {i + 1: <3} {component}" for i, component in enumerate(components))

            raise ValueError(
                "The assembly constructed with sim_ignore is not fully connected! URDF export requires a single "
                f"fully-connected robot, but the graph has {num_components} components. "
                f"Components:\n{components_string}"
            )

        # Debug for viewing final simulated URDF graph

        # node_relabel_mapping = {node: self.key_name(node, "joint") for node in graph.nodes()}
        # G_renamed = nx.relabel_nodes(graph, node_relabel_mapping)
        # nx.draw(G_renamed, with_labels=True, node_color='skyblue', node_size=30, edge_color='k', font_size=4)
        # plt.savefig('graph_one.png', dpi=300)
        # plt.clf()

        return graph, sim_ignored_joints

    @property
    async def graph(self) -> nx.Graph:
        return (await self.graph_and_ignored_joints)[0]

    @property
    async def sim_ignored_joints(self) -> set[Key]:
        return (await self.graph_and_ignored_joints)[1]

    @asyncstdlib.cached_property
    async def central_node(self) -> Key:
        """Identifies the most central node of the assembly.

        Since the assembly is undirected by default, we need to choose a node
        to act as the central node. We use the node with the highest closeness
        centrality as the root.

        Returns:
            The key of the central node.
        """
        if self.override_central_node is not None:
            if self.override_central_node not in await self.name_to_key:
                first_five = list((await self.name_to_key).keys())[:5]
                raise ValueError(
                    f"Central node {self.override_central_node} not found in the assembly. "
                    f"First five keys: {first_five}"
                )
            return (await self.name_to_key)[self.override_central_node]
        graph = await self.graph
        closeness_centrality = nx.closeness_centrality(graph)
        central_node: Key = max(closeness_centrality, key=closeness_centrality.get)
        return central_node

    @asyncstdlib.cached_property
    async def digraph(self) -> tuple[Key, nx.DiGraph]:
        """Converts the undirected graph to a directed graph.

        In order to convert the undirected graph to a directed graph, we need
        to choose a node to act as the root of the tree. We choose the node
        with the highest closeness centrality as the root.

        We also verify that the graph is actually a tree, and doesn't have any
        parallel connections, meaning there are no parts with two parents.

        Returns:
            The central node key and the directed graph.
        """
        graph = await self.graph
        if not nx.is_tree(graph):
            # Detects the parallel connections in the graph.
            for cycle in nx.cycle_basis(graph):
                for i, _ in enumerate(cycle):
                    if i == len(cycle) - 1:
                        break
                    logger.error(
                        "Parallel connection: %s -> %s",
                        await self.key_name(cycle[i], "link"),
                        await self.key_name(cycle[i + 1], "link"),
                    )
            raise ValueError("The assembly has parallel connections! URDF export requires no parallel connections.")

        central_node = await self.central_node
        logger.debug("Central node: %s", await self.key_name(central_node, "link"))
        return nx.bfs_tree(graph, central_node)

    @asyncstdlib.cached_property
    async def relations(self) -> dict[Key, MimicRelation]:
        relations: dict[Key, MimicRelation] = {}
        if self.disable_mimics:
            return relations
        for path, mate_relation_feature in (await self.key_to_mate_relation_feature).items():
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

    @asyncstdlib.cached_property
    async def ordered_joint_list(self) -> list[Joint]:
        digraph = await self.digraph
        bfs_node_ordering = list(nx.topological_sort(digraph))
        node_level = {node: i for i, node in enumerate(bfs_node_ordering)}

        # Creates a topologically-sorted list of joints.
        joint_list: list[Joint] = []
        for joint_key, mate_feature in (await self.key_to_mate_feature).items():
            if mate_feature.suppressed:
                continue

            lhs_entity, rhs_entity = mate_feature.featureData.matedEntities
            lhs_key, rhs_key = joint_key[:-1] + lhs_entity.key, joint_key[:-1] + rhs_entity.key

            sim_ignored_joints = await self.sim_ignored_joints
            if lhs_key in sim_ignored_joints or rhs_key in sim_ignored_joints:
                continue

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

    @asyncstdlib.cached_property
    async def joint_limits(self) -> dict[ElementUid, JointLimits]:
        """Gets the feature information for each assembly.

        Features are the properties of each part in an assembly. In our case,
        we use features to get things like the joint limits for the parts,
        which are not given in the default assembly or assembly metadata APIs.

        Returns:
            A dictionary mapping each assembly key to its features.
        """

        async def download_or_get_cached_features(assembly: RootAssembly | SubAssembly) -> Features:
            return await self.get_or_use_cached(
                f"assembly_{assembly.key.unique_id}",
                lambda: self.api.get_features(assembly),
                lambda feats_str: Features.model_validate_json(feats_str),
                lambda feats: feats.model_dump_json(indent=2),
            )

        # Gets the features for the assembly and all subassemblies.
        assembly = await self.assembly
        assembly_features = await gather_dict(
            {
                assembly.rootAssembly.key: download_or_get_cached_features(assembly.rootAssembly),
                **{
                    sub_assembly.key: download_or_get_cached_features(sub_assembly)
                    for sub_assembly in assembly.subAssemblies
                },
            }
        )

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
            for feature_state in assembly_feature.featureStates:
                if feature_state.value.message.featureStatus != FeatureStatus.OK:
                    logging.warn(
                        "Feature %s has status %s", feature_state.key, feature_state.value.message.featureStatus
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

    async def get_urdf_part(self, key: Key, joint: Joint | None = None) -> urdf.Link:
        part_name = await self.key_name(key, None)
        part_instance = (await self.key_to_part_instance)[key]
        part = (await self.euid_to_part)[part_instance.euid]

        # Gets the configuration string suffix.
        if part.configuration == "default":
            configuration_str = ""
        elif len(part.configuration) > 40:
            configuration_str = hashlib.md5(part.configuration.encode()).hexdigest()[:16]
        else:
            configuration_str = part.configuration

        part_color = await self.part_color(part)
        part_dynamic = (await self.part_dynamics(part)).bodies[part_instance.partId]

        # If the part is the root part, move the STL to be relative to the
        # center of mass and principle inertia axes, otherwise move it to
        # the origin of the part frame.
        com_to_part_tf = np.eye(4)
        com_to_part_tf[:3, 3] = -np.array(part_dynamic.center_of_mass).reshape(3)
        if joint is None:
            stl_origin_to_part_tf = com_to_part_tf
        else:
            stl_origin_to_part_tf = inv_tf(joint.child_entity.matedCS.part_to_mate_tf)
        self.stl_origin_to_part_tf[key] = stl_origin_to_part_tf

        part_file_name = f"{part_name}{configuration_str}.{self.mesh_ext}"
        part_file_path = self.mesh_dir / part_file_name

        if part_file_path.exists():
            logger.info("Using cached file %s", part_file_path)
        else:
            # Downloads the STL file.
            part_file_path_stl = part_file_path.with_suffix(".stl")
            if not part_file_path_stl.exists():
                logger.info("Downloading file %s", part_file_path_stl)
                buffer = io.BytesIO()
                await self.api.download_stl(part, buffer)
                buffer.seek(0)
                mesh_obj = stl.mesh.Mesh.from_file(None, fh=buffer)
                mesh_obj = apply_matrix_(mesh_obj, stl_origin_to_part_tf)
                mesh_obj.save(part_file_path_stl)

            # Converts the mesh to the desired format.
            if get_mesh_type(part_file_path) != "stl":
                logger.info("Converting STL file to %s", part_file_path)
                stl_to_fmt(part_file_path_stl, part_file_path)

        mass = part_dynamic.mass[0]
        if mass <= 0.0:
            logger.warning("Part %s has a mass of %f, which is invalid", part_name, mass)
            logger.warning("Part %s set to default mass %f", part_name, self.default_mass)
            mass = self.default_mass

        # Move the mesh origin and dynamics from the part frame to the parent
        # joint frame (since URDF expects this by convention).
        mesh_origin = urdf.Origin.zero_origin()
        center_of_mass = part_dynamic.center_of_mass_in_frame(stl_origin_to_part_tf)

        inertia = part_dynamic.inertia_matrix
        inertia_transformed = transform_inertia_tensor(inertia, stl_origin_to_part_tf[:3, :3])

        principal_axes = part_dynamic.principal_axes_in_frame(stl_origin_to_part_tf)
        principal_axes_rpy = R.from_matrix(principal_axes).as_euler("xyz", degrees=False)

        urdf_file_path = f"./meshes/{part_file_name}"
        urdf_link_name = await self.key_name(key, "link")

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
            collision=urdf.CollisionLink(
                origin=mesh_origin,
                geometry=urdf.MeshGeometry(filename=urdf_file_path),
            ),
        )

        inertial = urdf.InertialLink(
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
        )

        if not self.remove_inertia:
            urdf_part_link.inertial = inertial

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

    async def get_urdf_joint(self, joint: Joint) -> urdf.BaseJoint:
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
        joint_assembly_id, feature_id = (await self.assembly_key_to_id)[joint.joint_key[:-1]], joint.joint_key[-1]
        joint_info_key = ElementUid(
            document_id=joint_assembly_id.document_id,
            document_microversion=joint_assembly_id.document_microversion,
            element_id=joint_assembly_id.element_id,
            part_id=feature_id,
        )
        joint_limits = (await self.joint_limits)[joint_info_key]
        expression_resolver = ExpressionResolver(joint_assembly_id.configuration)

        def resolve(expression: str | None) -> float | None:
            return None if expression is None else expression_resolver.read_expression(expression)

        name = await self.key_name(joint.joint_key, "joint")
        origin = urdf.Origin.from_matrix(parent_stl_origin_to_mate_tf)
        mate_type = joint.mate_type

        match mate_type:
            case MateType.FASTENED:
                parent, child = await self.key_name(joint.parent, "link"), await self.key_name(joint.child, "link")
                return urdf.FixedJoint(
                    name=name,
                    parent=parent,
                    child=child,
                    origin=origin,
                )

            case MateType.REVOLUTE:
                parent, child = await self.key_name(joint.parent, "link"), await self.key_name(joint.child, "link")
                mimic_joint = (await self.relations).get(joint.joint_key)

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
                            joint=await self.key_name(mimic_joint.parent, "joint"),
                            multiplier=mimic_joint.multiplier,
                            offset=0.0,
                        )
                    ),
                )

            case MateType.SLIDER:
                parent, child = await self.key_name(joint.parent, "link"), await self.key_name(joint.child, "link")
                mimic_joint = (await self.relations).get(joint.joint_key)

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
                            joint=await self.key_name(mimic_joint.parent, "joint"),
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

    async def get_part_name_from_key(self, key: tuple) -> str:
        try:
            part_instance = (await self.key_to_part_instance)[key]
            part = (await self.euid_to_part)[part_instance.euid]
            part_name = await self.part_name(part)
            return part_name
        except KeyError:
            return "Unknown part"

    async def save_urdf(self) -> None:
        """Saves a URDF file for the assembly to the output directory."""
        urdf_parts: list[urdf.Link | urdf.BaseJoint] = []

        # Add the first link, since it has no incoming joint.
        part_link = await self.get_urdf_part(await self.central_node)
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

        async def process_joint(joint: Joint) -> tuple[urdf.Link | None, urdf.BaseJoint | None]:
            urdf_joint, urdf_link = None, None
            joint_name = await self.key_name(joint.joint_key, "joint")  # Get the joint name here

            try:
                urdf_joint, urdf_link = await asyncio.gather(
                    self.get_urdf_joint(joint),
                    self.get_urdf_part(joint.child, joint),
                )

            except KeyError as e:
                parent_part_name = self.get_part_name_from_key(joint.parent)
                child_part_name = self.get_part_name_from_key(joint.child)
                logging.error("KeyError creating joint: %s", joint_name)
                logging.error("KeyError details: %s", e)
                logging.error("Parent part: %s, Child part: %s", parent_part_name, child_part_name)
                logging.error("Traceback: %s", traceback.format_exc())
                raise

            except Exception as e:
                logging.error("Exception creating joint: %s", joint_name)
                logging.error(e)
                logging.error("Traceback: %s", traceback.format_exc())
                if urdf_joint is not None:
                    logging.warning("Joint %s", urdf_joint)
                if urdf_link is not None:
                    logging.warning("Link %s", urdf_link)
                raise

            return urdf_link, urdf_joint

        all_joints = await asyncio.gather(*(process_joint(joint) for joint in await self.ordered_joint_list))
        urdf_parts.extend(filter(None, (joint for joints in all_joints for joint in joints)))

        # Saves the final URDF.
        robot_name = clean_name(str((await self.assembly_metadata).property_map.get("Name", "robot")))
        urdf_robot = urdf.Robot(name=robot_name, parts=urdf_parts)
        urdf_robot.save(self.output_dir / f"{robot_name}.urdf")

        # Rename joints if flags on
        if self.override_joint_names is not None or self.override_nonfixed is not None:
            update_joints(
                self.output_dir / f"{robot_name}.urdf",
                self.override_joint_names,
                self.override_nonfixed,
                self.override_limits,
                self.override_torques,
            )

        if self.merge_fixed_joints:
            get_merged_urdf(self.output_dir / f"{robot_name}.urdf", 1.0)
            robot_name += "_merged"
            if self.simplify_meshes:
                simplify_all(self.output_dir / f"{robot_name}.urdf", self.voxel_size)
                robot_name += "_simplified"
        elif self.simplify_meshes:
            simplify_all(self.output_dir / f"{robot_name}.urdf", self.voxel_size)
            robot_name += "_simplified"

        # cleanup mesh dir, unlink other urdfs
        if self.merge_fixed_joints or self.simplify_meshes:
            cleanup_mesh_dir(self.output_dir / f"{robot_name}.urdf")
        if self.merge_fixed_joints or self.simplify_meshes:
            for f in self.output_dir.glob("*.urdf"):
                if f.name != f"{robot_name}.urdf":
                    f.unlink()

    async def save_mjcf(self) -> None:
        """Saves a MJCF file for the assembly to the output directory."""
        if self.mesh_ext != "stl":
            raise ValueError("MJCF only supports STL meshes")
        await self.save_urdf()

        robot_name = clean_name(str((await self.assembly_metadata).property_map.get("Name", "robot")))

        # update robot_name if flags on
        if self.merge_fixed_joints:
            robot_name += "_merged"
        if self.simplify_meshes:
            robot_name += "_simplified"

        mjcf_robot = mjcf.Robot(robot_name, self.output_dir, mjcf.Compiler(angle="radian", meshdir="meshes"))
        mjcf_robot.adapt_world()
        mjcf_robot.save(self.output_dir / f"{robot_name}.xml")
