# ruff: noqa: N815
"""Defines the schema for the Assembly API."""

import functools
import re
from collections import deque
from enum import Enum
from typing import Any, Deque, Iterator, Literal

import numpy as np
from pydantic import BaseModel

from kol.onshape.schema.common import ElementUid

# Key = NewType("Key", tuple[str, ...])
Key = tuple[str, ...]


def clean_name(name: str) -> str:
    return re.sub(r"\s+", "_", re.sub(r"[<>]", "", name)).lower()


class BaseInstance(BaseModel):
    name: str
    suppressed: bool
    id: str
    fullConfiguration: str
    configuration: str
    documentMicroversion: str
    documentId: str
    elementId: str

    @property
    def key(self) -> ElementUid:
        return ElementUid(
            self.documentId,
            self.documentMicroversion,
            self.elementId,
            "",
            self.fullConfiguration,
        )


class AssemblyInstance(BaseInstance):
    type: Literal["Assembly"]


class PartInstance(BaseInstance):
    type: Literal["Part"]
    isStandardContent: bool
    partId: str

    @property
    def key(self) -> ElementUid:
        return ElementUid(
            self.documentId,
            self.documentMicroversion,
            self.elementId,
            self.partId,
            self.fullConfiguration,
        )


Instance = AssemblyInstance | PartInstance


class Occurrence(BaseModel):
    hidden: bool
    fixed: bool
    transform: list[float]
    path: list[str]

    @property
    def world_to_part_tf(self) -> np.matrix:
        return np.matrix(np.array(self.transform).reshape(4, 4))

    @property
    def key(self) -> Key:
        return tuple(self.path)


class MateType(str, Enum):
    FASTENED = "FASTENED"
    REVOLUTE = "REVOLUTE"
    SLIDER = "SLIDER"
    PLANAR = "PLANAR"
    CYLINDRICAL = "CYLINDRICAL"
    PIN_SLOT = "PIN_SLOT"
    BALL = "BALL"
    PARALLEL = "PARALLEL"


class MatedCS(BaseModel):
    xAxis: list[float]
    yAxis: list[float]
    zAxis: list[float]
    origin: list[float]

    @property
    def part_to_mate_tf(self) -> np.matrix:
        part_to_mate_tf = np.eye(4)
        part_to_mate_tf[:3, :3] = np.stack(
            (
                np.array(self.xAxis),
                np.array(self.yAxis),
                np.array(self.zAxis),
            )
        ).T
        part_to_mate_tf[:3, 3] = self.origin
        return np.matrix(part_to_mate_tf)


class MatedEntity(BaseModel):
    matedOccurrence: list[str]
    matedCS: MatedCS

    @property
    def key(self) -> Key:
        return tuple(self.matedOccurrence)


class MateFeatureData(BaseModel):
    mateType: MateType
    matedEntities: list[MatedEntity]
    name: str


class RelationType(str, Enum):
    GEAR = "GEAR"


class MateRelationMate(BaseModel):
    featureId: str
    occurrence: list[str]

    @property
    def key(self) -> Key:
        return tuple(self.occurrence + [self.featureId])


class MateRelationFeatureData(BaseModel):
    relationType: RelationType
    mates: list[MateRelationMate]
    reverseDirection: bool
    relationRatio: float
    name: str


class MateRelationFeature(BaseModel):
    id: str
    suppressed: bool
    featureType: Literal["mateRelation"]
    featureData: MateRelationFeatureData

    def keys(self, root_key: Key = ()) -> list[Key]:
        return [root_key + mated.key for mated in self.featureData.mates]


class MateFeature(BaseModel):
    id: str
    suppressed: bool
    featureType: Literal["mate"]
    featureData: MateFeatureData

    def keys(self, root_key: Key = ()) -> list[Key]:
        return [root_key + mated.key for mated in self.featureData.matedEntities]


Feature = MateRelationFeature | MateFeature


class Pattern(BaseModel):
    pass


class RootAssembly(BaseModel):
    occurrences: list[Occurrence]
    instances: list[Instance]
    patterns: list[Pattern]
    features: list[Feature]
    fullConfiguration: str
    configuration: str
    documentMicroversion: str
    documentId: str
    elementId: str

    @property
    def key(self) -> ElementUid:
        return ElementUid(
            self.documentId,
            self.documentMicroversion,
            self.elementId,
            "",
            self.fullConfiguration,
        )


class SubAssembly(BaseModel):
    instances: list[Instance]
    patterns: list[Pattern]
    features: list[Feature]
    fullConfiguration: str
    configuration: str
    documentMicroversion: str
    documentId: str
    elementId: str

    @property
    def key(self) -> ElementUid:
        return ElementUid(
            self.documentId,
            self.documentMicroversion,
            self.elementId,
            "",
            self.fullConfiguration,
        )


class AssemblyProperty(BaseModel):
    name: str
    value: Any


class AssemblyMetadata(BaseModel):
    jsonType: str
    elementType: int
    mimeType: str
    elementId: str
    properties: list[AssemblyProperty]
    href: str

    @functools.cached_property
    def property_map(self) -> dict[str, Any]:
        return {prop.name: prop.value for prop in self.properties}


class PartBodyType(str, Enum):
    solid = "solid"


class Part(BaseModel):
    isStandardContent: bool
    partId: str
    bodyType: PartBodyType
    fullConfiguration: str
    configuration: str
    documentMicroversion: str
    documentId: str
    elementId: str

    @property
    def key(self) -> ElementUid:
        return ElementUid(
            self.documentId,
            self.documentMicroversion,
            self.elementId,
            self.partId,
            self.fullConfiguration,
        )


class PartStudioFeature(BaseModel):
    pass


class Assembly(BaseModel):
    rootAssembly: RootAssembly
    subAssemblies: list[SubAssembly]
    parts: list[Part]
    partStudioFeatures: list[PartStudioFeature]

    @functools.cached_property
    def key_to_subassembly(self) -> dict[ElementUid, SubAssembly]:
        return {sub.key: sub for sub in self.subAssemblies}

    @functools.cached_property
    def key_to_part(self) -> dict[ElementUid, Part]:
        return {part.key: part for part in self.parts}

    @functools.cached_property
    def path_to_occurrence(self) -> dict[Key, Occurrence]:
        return {occurrence.key: occurrence for occurrence in self.rootAssembly.occurrences}

    def traverse_assemblies(self) -> Iterator[tuple[Key, AssemblyInstance, SubAssembly]]:
        subassembly_deque: Deque[tuple[Key, SubAssembly]] = deque()
        visited: set[Key] = set()

        # Adds the root assembly to the traversal.
        for instance in self.rootAssembly.instances:
            if isinstance(instance, AssemblyInstance):
                instance_path: Key = (instance.id,)
                if instance_path in visited:
                    continue
                visited.add(instance_path)
                subassembly = self.key_to_subassembly[instance.key]
                yield instance_path, instance, subassembly
                subassembly_deque.append((instance_path, subassembly))

        # Adds all the subassemblies to the traversal, recursively.
        while subassembly_deque:
            path, sub_assembly = subassembly_deque.popleft()
            for instance in sub_assembly.instances:
                if isinstance(instance, AssemblyInstance):
                    assert isinstance(instance, AssemblyInstance)  # For mypy.
                    instance_path = path + (instance.id,)
                    if instance_path in visited:
                        continue
                    visited.add(instance_path)
                    subassembly = self.key_to_subassembly[instance.key]
                    yield instance_path, instance, subassembly
                    subassembly_deque.append((instance_path, subassembly))

    @functools.cached_property
    def path_to_instance(self) -> dict[Key, Instance]:
        instance_mapping: dict[Key, Instance] = {}
        for instance in self.rootAssembly.instances:
            instance_mapping[(instance.id,)] = instance
        for path, assembly_instance, sub_assembly in self.traverse_assemblies():
            instance_mapping[path] = assembly_instance
            for instance in sub_assembly.instances:
                instance_mapping[path + (instance.id,)] = instance
        return instance_mapping

    @property
    def path_to_part_instance(self) -> dict[Key, PartInstance]:
        return {p: i for p, i in self.path_to_instance.items() if isinstance(i, PartInstance)}

    @property
    def path_to_assembly_instance(self) -> dict[Key, AssemblyInstance]:
        return {p: i for p, i in self.path_to_instance.items() if isinstance(i, AssemblyInstance)}

    @functools.cached_property
    def path_to_feature(self) -> dict[Key, Feature]:
        feature_mapping: dict[Key, Feature] = {}
        for feature in self.rootAssembly.features:
            feature_mapping[(feature.id,)] = feature
        for path, _, sub_assembly in self.traverse_assemblies():
            for feature in sub_assembly.features:
                feature_mapping[path + (feature.id,)] = feature
        return feature_mapping

    @property
    def path_to_mate_feature(self) -> dict[Key, MateFeature]:
        return {p: f for p, f in self.path_to_feature.items() if isinstance(f, MateFeature)}

    @property
    def path_to_mate_relation_feature(self) -> dict[Key, MateRelationFeature]:
        return {p: f for p, f in self.path_to_feature.items() if isinstance(f, MateRelationFeature)}

    @functools.cached_property
    def path_to_name(self) -> dict[Key, list[str]]:
        path_name_mapping: dict[Key, list[str]] = {}
        path_name_mapping[()] = []
        for instance in self.rootAssembly.instances:
            path_name_mapping[(instance.id,)] = [instance.name]
        for feature in self.rootAssembly.features:
            path_name_mapping[(feature.id,)] = [feature.featureData.name]
        for path, assembly_instance, sub_assembly in self.traverse_assemblies():
            path_name_mapping[path] = path_name_mapping[path[:-1]] + [assembly_instance.name]
            for instance in sub_assembly.instances:
                path_name_mapping[path + (instance.id,)] = path_name_mapping[path] + [instance.name]
            for feature in sub_assembly.features:
                path_name_mapping[path + (feature.id,)] = path_name_mapping[path] + [feature.featureData.name]
        return path_name_mapping

    def key_name(self, key: Key, prefix: Literal["link", "joint"]) -> str:
        return prefix + "_" + clean_name("_".join(self.path_to_name.get(key, key)))
