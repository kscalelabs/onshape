"""Utility functions common to different formats."""

from __future__ import annotations

import io
import json
import re
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Self
from xml.dom import minidom


@dataclass
class JointMetadata:
    actuator_type: str
    id: int
    nn_id: int
    kp: float
    kd: float
    soft_torque_limit: float
    min_angle_deg: float
    max_angle_deg: float

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(**data)

    def to_dict(self) -> dict:
        return {k: v for k, v in asdict(self).items()}

    @classmethod
    def from_json(cls, json_path: Path) -> Self:
        with open(json_path, "r") as f:
            data = json.load(f)
        return cls(**data)


@dataclass
class ActuatorMetadata:
    actuator_type: str
    sysid: str = ""
    max_torque: float = 0.0
    max_velocity: float = 0.0
    armature: float = 0.0
    damping: float = 0.0
    frictionloss: float = 0.0
    vin: float | None = None
    kt: float | None = None
    R: float | None = None
    max_pwm: float | None = None
    error_gain: float | None = None

    def to_dict(self) -> dict:
        return {k: v for k, v in asdict(self).items()}

    @classmethod
    def from_dict(cls, data: dict) -> Self:
        return cls(**data)

    @classmethod
    def from_json(cls, json_path: Path) -> Self:
        with open(json_path, "r") as f:
            data = json.load(f)
        return cls(**data)


def save_xml(
    path: str | Path | io.StringIO,
    tree: ET.ElementTree[ET.Element | None] | ET.ElementTree[ET.Element] | ET.Element,
) -> None:
    if isinstance(tree, ET.ElementTree):
        root = tree.getroot()
        if root is None:
            raise ValueError("ElementTree has no root element")
        tree = root
    xmlstr = minidom.parseString(ET.tostring(tree)).toprettyxml(indent="  ")
    xmlstr = re.sub(r"\n\s*\n", "\n", xmlstr)
    if isinstance(path, io.StringIO):
        path.write(xmlstr)
    else:
        with open(path, "w") as f:
            f.write(xmlstr)
