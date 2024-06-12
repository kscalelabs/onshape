# ruff: noqa: N815
"""Defines the schema for the Assembly API."""

import functools
import logging
from dataclasses import dataclass
from enum import Enum
from typing import Any, Literal

import numpy as np
from pydantic import BaseModel

from kol.onshape.schema.common import ElementUid

logger = logging.getLogger(__name__)

Key = tuple[str, ...]


@dataclass
class MimicRelation:
    parent: Key
    multiplier: float


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
    def euid(self) -> ElementUid:
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
    def euid(self) -> ElementUid:
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
    LINEAR = "LINEAR"


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


class MateGroupFeatureOccurrence(BaseModel):
    occurrence: list[str]


class MateGroupFeatureData(BaseModel):
    occurrences: list[MateGroupFeatureOccurrence]
    name: str


class MateGroupFeature(BaseModel):
    id: str
    suppressed: bool
    featureType: Literal["mateGroup"]
    featureData: MateGroupFeatureData


class Pattern(BaseModel):
    pass


class RootAssembly(BaseModel):
    occurrences: list[Occurrence]
    instances: list[Instance]
    patterns: list[Pattern]
    features: list[MateGroupFeature | MateRelationFeature | MateFeature]
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
    features: list[MateGroupFeature | MateRelationFeature | MateFeature]
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
