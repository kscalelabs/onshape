"""Defines common types and functions for exporting MJCF files."""

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Literal


@dataclass
class Compiler:
    coordinate: Literal["local", "global"]
    angle: Literal["radian", "degree"]
    meshdir: str
    eulerseq: Literal["xyz", "zxy", "zyx", "yxz", "yzx", "xzy"]

    def to_xml(self, root: ET.Element) -> ET.Element:
        compiler = ET.SubElement(root, "compiler")
        compiler.set("coordinate", self.coordinate)
        compiler.set("angle", self.angle)
        compiler.set("meshdir", self.meshdir)
        compiler.set("eulerseq", self.eulerseq)
        return compiler


@dataclass
class Option:
    timestep: float
    viscosity: float

    def to_xml(self, root: ET.Element) -> ET.Element:
        option = ET.SubElement(root, "option")
        option.set("timestep", str(self.timestep))
        option.set("viscosity", str(self.viscosity))
        return option


@dataclass
class Robot:
    name: str
    compiler: Compiler
    option: Option

    def to_xml(self) -> ET.Element:
        robot = ET.Element("mujoco", name=self.name)
        self.compiler.to_xml(robot)
        self.option.to_xml(robot)
        return robot

    def save(self, path: str | Path) -> None:
        tree = ET.ElementTree(self.to_xml())
        root = tree.getroot()

        def indent(elem: ET.Element, level: int = 0) -> ET.Element:
            i = "\n" + level * "  "
            if len(elem):
                if not elem.text or not elem.text.strip():
                    elem.text = i + "  "
                if not elem.tail or not elem.tail.strip():
                    elem.tail = i
                for elem in elem:
                    indent(elem, level + 1)
                if not elem.tail or not elem.tail.strip():
                    elem.tail = i
            else:  # noqa: PLR5501
                if level and (not elem.tail or not elem.tail.strip()):
                    elem.tail = i
            return elem

        indent(root)
        tree.write(path, encoding="utf-8", xml_declaration=True, method="xml")
