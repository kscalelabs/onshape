# ruff: noqa: N815
"""Defines the schema for the Document API."""

from enum import Enum

from pydantic import BaseModel, RootModel


class ElementType(str, Enum):
    ASSEMBLY = "ASSEMBLY"
    PARTSTUDIO = "PARTSTUDIO"
    DRAWING = "DRAWING"
    BLOB = "BLOB"


class Element(BaseModel):
    name: str
    id: str
    type: str
    elementType: ElementType


class Elements(RootModel):
    root: list[Element]
