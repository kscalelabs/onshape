"""Defines common types and functions for exporting MJCF files.

API reference

https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_native_writer.cc#L780
Mujoco XML format reference
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, List

from kol.formats.common import save_xml


@dataclass
class Compiler:
    coordinate: Literal["local", "global"] = None
    angle: Literal["radian", "degree"] = "radian"
    meshdir: str = None
    eulerseq: Literal["xyz", "zxy", "zyx", "yxz", "yzx", "xzy"] = None

    def to_xml(self, root: ET.Element) -> ET.Element:
        compiler = ET.SubElement(root, "compiler")
        if self.coordinate is not None:
            compiler.set("coordinate", self.coordinate)
        compiler.set("angle", self.angle)
        if  self.meshdir is not None:
            compiler.set("meshdir", self.meshdir)
        if self.eulerseq is not None:
            compiler.set("eulerseq", self.eulerseq)
        return compiler


@dataclass
class Mesh:
    name: str
    file: float
    scale: tuple[float, float, float] = (1, 1, 1)

    def to_xml(self, root: ET.Element) -> ET.Element:
        mesh = ET.SubElement(root, "mesh")
        mesh.set("name", self.name)
        mesh.set("file", self.file)
        mesh.set("scale", " ".join(map(str, self.scale)))
        return mesh


# body parts
@dataclass
class Body:
    name: str
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    #geom: Geom
    #join
    # inertial?

    def to_xml(self, root: ET.Element) -> ET.Element:
        body = ET.SubElement(root, "body")
        body.set("name", self.name)
        body.set("pos", " ".join(map(str, self.pos)))
        body.set("quat", " ".join(map(str, self.quat)))
        # TODO
        # self.geom.to_xml(body)
        return body


@dataclass
class Joint:
    name: str
    type: Literal["hinge", "slide", "ball", "free"]
    pos: tuple[float, float, float]
    axis: tuple[float, float, float]
    range: tuple[float, float]
    damping: float
    stiffness: float

    def to_xml(self, root: ET.Element) -> ET.Element:
        joint = ET.SubElement(root, "joint")
        joint.set("name", self.name)
        joint.set("type", self.type)
        joint.set("pos", " ".join(map(str, self.pos)))
        joint.set("axis", " ".join(map(str, self.axis)))
        joint.set("range", " ".join(map(str, self.range)))
        joint.set("damping", str(self.damping))
        joint.set("stiffness", str(self.stiffness))
        return joint


@dataclass
class Geom:
    name: str
    type: Literal["plane", "sphere", "cylinder", "box", "capsule", "ellipsoid", "mesh"]
    # size: float
    rgba: tuple[float, float, float, float]
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]

    def to_xml(self, root: ET.Element) -> ET.Element:
        geom = ET.SubElement(root, "geom")
        geom.set("name", self.name)
        geom.set("type", self.type)
        geom.set("size", " ".join(map(str, self.size)))
        geom.set("rgba", " ".join(map(str, self.rgba)))
        geom.set("pos", " ".join(map(str, self.pos)))
        geom.set("quat", " ".join(map(str, self.quat)))
        return geom

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
class Actuator:
    name: str
    joint: str
    ctrlrange: tuple[float, float]

    def to_xml(self, root: ET.Element) -> ET.Element:
        actuator = ET.SubElement(root, "actuator")
        actuator.set("name", self.name)
        actuator.set("joint", self.joint)
        actuator.set("ctrlrange", " ".join(map(str, self.ctrlrange)))
        return actuator


@dataclass
class Robot:
    name: str
    parts: List
    compiler: Compiler = field(default_factory=Compiler)
    # option: Option = field(default_factory=Option)

    def to_xml(self) -> ET.Element:
        robot = ET.Element("mujoco", name=self.name)
        self.compiler.to_xml(robot)
        # self.option.to_xml(robot)
        # add dependency here
        assets = ET.SubElement(robot, "assets")
        for asset in self.parts:
            asset.to_xml(assets)

        worldbody = ET.SubElement(robot, "worldbody")
        return robot

    def save(self, path: str | Path) -> None:
        tree = ET.ElementTree(self.to_xml())
        save_xml(path, tree)
