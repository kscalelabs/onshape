# ruff: noqa: N815
"""Defines the schema for the Document API."""

from pydantic import BaseModel


class DefaultWorkspace(BaseModel):
    id: str
    href: str


class Document(BaseModel):
    defaultWorkspace: DefaultWorkspace
