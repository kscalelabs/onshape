"""Defines common classes and functions to use with schemas."""

import hashlib
from dataclasses import dataclass


@dataclass(frozen=True)
class ElementUid:
    document_id: str
    document_microversion: str
    element_id: str
    part_id: str = ""
    configuration: str = ""

    @property
    def unique_id(self) -> str:
        key = ":".join(
            [
                self.document_id,
                self.document_microversion,
                self.element_id,
                self.part_id,
                self.configuration,
            ],
        )
        return hashlib.sha256(key.encode()).hexdigest()[0:16]
