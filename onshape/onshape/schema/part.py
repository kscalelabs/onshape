# ruff: noqa: N815
"""Defines the schema for parts APIs."""

import functools
from typing import Any

import numpy as np
from pydantic import BaseModel


class PartProperty(BaseModel):
    name: str
    value: Any


class PartMetadata(BaseModel):
    jsonType: str
    isMesh: bool
    partId: str
    isFlattenedBody: bool
    partType: str
    meshState: int
    properties: list[PartProperty]
    href: str

    @functools.cached_property
    def property_map(self) -> dict[str, Any]:
        return {prop.name: prop.value for prop in self.properties}


class PrincipleAxis(BaseModel):
    x: float
    y: float
    z: float

    @property
    def array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


class PartBody(BaseModel):
    mass: list[float]
    volume: list[float]
    periphery: list[float]
    centroid: list[float]
    inertia: list[float]
    hasMass: bool
    massMissingCount: int
    principalInertia: list[float]
    principalAxes: list[PrincipleAxis]

    @property
    def principal_inertia(self) -> np.ndarray:
        return np.array(self.principalInertia)

    @property
    def principal_axes(self) -> np.matrix:
        return np.matrix(np.array([axis.array for axis in self.principalAxes]))

    def principal_axes_in_frame(self, frame: np.ndarray) -> np.ndarray:
        return frame[:3, :3] @ self.principal_axes

    @property
    def inertia_matrix(self) -> np.matrix:
        return np.matrix(np.array(self.inertia[:9]).reshape(3, 3))

    def inertia_in_frame(self, frame: np.matrix) -> np.matrix:
        frame_r = frame[:3, :3].T
        return frame_r * self.inertia_matrix * frame_r.T

    @property
    def center_of_mass(self) -> tuple[float, float, float]:
        return (self.centroid[0], self.centroid[1], self.centroid[2])

    def center_of_mass_in_frame(self, frame: np.ndarray) -> tuple[float, float, float]:
        com = np.matrix(list(self.center_of_mass) + [1.0])
        com_in_frame = (frame * com.T)[:3]
        return (float(com_in_frame[0, 0]), float(com_in_frame[1, 0]), float(com_in_frame[2, 0]))


class PartDynamics(BaseModel):
    microversionId: str
    bodies: dict[str, PartBody]


class ThumbnailSize(BaseModel):
    size: str
    href: str
    mediaType: str
    uniqueId: str | None
    viewOrientation: str
    renderMode: str
    sheetName: str | None


class ThumbnailInfo(BaseModel):
    sizes: list[ThumbnailSize]
    secondarySizes: list[ThumbnailSize] | None
    id: str | None
    href: str
