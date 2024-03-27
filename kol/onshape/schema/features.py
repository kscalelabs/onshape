# ruff: noqa: N815
"""Defines the schema for the Features API."""

import functools
from enum import Enum
from typing import Literal

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


# -----------------------------------
# BTMParameterQueryWithOccurrenceList
# -----------------------------------


class InferenceQueryWithOccurrenceMessage(BaseModel):
    secondGeometryId: str
    inferenceType: str
    geometryIds: list[str]
    path: list[str]
    hasUserCode: bool
    nodeId: str


class InferenceQueryWithOccurrence(BaseModel):
    type: Literal[1083]
    typeName: Literal["BTMInferenceQueryWithOccurrence"]
    message: InferenceQueryWithOccurrenceMessage


class FeatureQueryWithOccurrenceMessage(BaseModel):
    featureId: str
    queryData: str
    path: list[str]
    hasUserCode: bool
    nodeId: str


class FeatureQueryWithOccurrence(BaseModel):
    type: Literal[157]
    typeName: Literal["BTMFeatureQueryWithOccurrence"]
    message: FeatureQueryWithOccurrenceMessage


class ParameterQueryWithOccurrenceListMessage(ParameterMessage):
    queries: list[InferenceQueryWithOccurrence | FeatureQueryWithOccurrence]


class ParameterQueryWithOccurrenceList(BaseModel):
    type: Literal[67]
    typeName: Literal["BTMParameterQueryWithOccurrenceList"]
    message: ParameterQueryWithOccurrenceListMessage


# -------------------
# BTMParameterBoolean
# -------------------


class ParameterBooleanMessage(ParameterMessage):
    value: bool


class ParameterBoolean(BaseModel):
    type: Literal[144]
    typeName: Literal["BTMParameterBoolean"]
    message: ParameterBooleanMessage


# ----------------
# BTMParameterEnum
# ----------------


class ParameterEnumMessage(ParameterMessage):
    enumName: str
    value: str
    namespace: str


class ParameterEnum(BaseModel):
    type: Literal[145]
    typeName: Literal["BTMParameterEnum"]
    message: ParameterEnumMessage


# --------------------
# BTMParameterQuantity
# --------------------


class ParameterQuantityMessage(ParameterMessage):
    units: str
    value: float
    expression: str
    isInteger: bool


class ParameterQuantity(BaseModel):
    type: Literal[147]
    typeName: Literal["BTMParameterQuantity"]
    message: ParameterQuantityMessage


# ----------------------------
# BTMParameterNullableQuantity
# ----------------------------


class ParameterNullableQuantityMessage(ParameterMessage):
    isNull: bool
    nullValue: str
    units: str
    value: float
    expression: str
    isInteger: bool


class ParameterNullableQuantity(BaseModel):
    type: Literal[807]
    typeName: Literal["BTMParameterNullableQuantity"]
    message: ParameterNullableQuantityMessage


Parameter = (
    ParameterBoolean | ParameterEnum | ParameterQuantity | ParameterQueryWithOccurrenceList | ParameterNullableQuantity
)


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
    def parameter_dict(self) -> dict[str, Parameter]:
        return {param.message.parameterId: param for param in self.parameters}


class Feature(BaseModel):
    type: int
    typeName: str
    message: FeatureMessage


class FeatureStatus(str, Enum):
    OK = "OK"


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
