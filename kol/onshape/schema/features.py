# ruff: noqa: N815
"""Defines the schema for the Features API."""

import functools
from enum import Enum
from typing import Any, Literal

from pydantic import BaseModel, Field


class FeatureType(str, Enum):
    mate = "mate"
    mateRelation = "mateRelation"


class InferenceType(str, Enum):
    CENTROID = "CENTROID"


class ParameterMessage(BaseModel):
    parameterId: str
    hasUserCode: bool
    nodeId: str


Parameter = Any


class MateConnectorMessage(BaseModel):
    isHidden: bool
    implicit: bool
    isAuxiliaryTreeMateConnector: bool
    version: int
    featureType: str
    featureId: str
    name: str
    parameters: list[Parameter] = Field(discriminator="type")


class MateConnector(BaseModel):
    type: int
    typeName: str
    message: MateConnectorMessage


class SubFeature(BaseModel):
    pass


class SuppressionState(BaseModel):
    type: int


class FeatureMessage(BaseModel):
    version: int
    featureType: FeatureType
    featureId: str
    name: str
    parameters: list[Parameter]
    suppressed: bool
    namespace: str
    subFeatures: list[SubFeature]
    returnAfterSubfeatures: bool
    suppressionState: SuppressionState
    hasUserCode: bool
    nodeId: str
    mateConnectors: list[MateConnector] | None = None

    @functools.cached_property
    def parameter_dict(self) -> dict[str, dict]:
        return {param["message"]["parameterId"]: param for param in self.parameters}


class Feature(BaseModel):
    type: int
    typeName: str
    message: FeatureMessage


class FeatureStatus(str, Enum):
    OK = "OK"
    ERROR = "ERROR"


class FeatureStateMessage(BaseModel):
    featureStatus: FeatureStatus
    inactive: bool


class FeatureStateValue(BaseModel):
    type: Literal[1688]
    typeName: Literal["BTFeatureState"]
    message: FeatureStateMessage


class FeatureState(BaseModel):
    key: str
    value: FeatureStateValue


class Features(BaseModel):
    features: list[Feature]
    featureStates: list[FeatureState]
    isComplete: bool
    serializationVersion: str
    sourceMicroversion: str
    rejectMicroversionSkew: bool
    microversionSkew: bool
    libraryVersion: int
