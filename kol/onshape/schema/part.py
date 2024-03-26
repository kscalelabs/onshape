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
    partIdentity: str
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
    def inertia_matrix(self) -> np.matrix:
        return np.matrix(np.array(self.inertia[:9]).reshape(3, 3))

    def inertia_in_frame(self, frame: np.matrix) -> np.matrix:
        frame_r = frame[:3, :3]
        return frame_r * self.inertia_matrix * frame_r.T

    @property
    def center_of_mass(self) -> np.ndarray:
        return np.array(self.centroid[:3])

    def center_of_mass_in_frame(self, frame: np.matrix) -> tuple[float, float, float]:
        com = np.matrix(self.centroid[:3] + [1.0])
        com = (frame * com.T)[:3]
        return (float(com[0, 0]), float(com[1, 0]), float(com[2, 0]))


class PartDynamics(BaseModel):
    microversionId: str
    bodies: dict[str, PartBody]
