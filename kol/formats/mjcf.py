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
    scale: tuple[float, float, float] = None

    def to_xml(self, root: ET.Element) -> ET.Element:
        mesh = ET.SubElement(root, "mesh")
        mesh.set("name", self.name)
        mesh.set("file", self.file)
        if self.scale is not None:
            mesh.set("scale", " ".join(map(str, self.scale)))
        return mesh


@dataclass
class Joint:
    name: str
    type: Literal["hinge", "slide", "ball", "free"]
    pos: tuple[float, float, float]
    axis: tuple[float, float, float]
    limited: bool
    range: tuple[float, float]
    damping: float
    stiffness: float

    def to_xml(self, root: ET.Element) -> ET.Element:
        joint = ET.SubElement(root, "joint")
        joint.set("name", self.name)
        joint.set("type", self.type)
        if self.pos is not None:
            joint.set("pos", " ".join(map(str, self.pos)))
        if self.axis is not None:
            joint.set("axis", " ".join(map(str, self.axis)))
        if self.range is not None:
            self.limited = True
            joint.set("range", " ".join(map(str, self.range)))
        else:
            self.limited = False
        joint.set("limited", str(self.limited))
        if self.damping is not None:
            joint.set("damping", str(self.damping))
        if self.stiffness is not None:  
            joint.set("stiffness", str(self.stiffness))
        return joint


@dataclass
class Geom:
    mesh: str
    type: Literal["plane", "sphere", "cylinder", "box", "capsule", "ellipsoid", "mesh"]
    # size: float
    rgba: tuple[float, float, float, float]
    pos: tuple[float, float, float] = None
    quat: tuple[float, float, float, float] = None

    def to_xml(self, root: ET.Element) -> ET.Element:
        geom = ET.SubElement(root, "geom")
        geom.set("mesh", self.mesh)
        geom.set("type", self.type)
        geom.set("rgba", " ".join(map(str, self.rgba)))
        if self.pos is not None:
            geom.set("pos", " ".join(map(str, self.pos)))
        if self.quat is not None:
            geom.set("quat", " ".join(map(str, self.quat)))
        return geom


@dataclass
class Body:
    name: str
    pos: tuple[float, float, float] = field(default=None)
    quat: tuple[float, float, float, float] = field(default=None)
    geom: Geom = field(default=None)
    joint: Joint = field(default=None)
    # TODO - fix inertia, until then rely on Mujoco's engine
    # inertial: Inertial = None

    def to_xml(self, root: ET.Element) -> ET.Element:
        body = ET.SubElement(root, "body")
        body.set("name", self.name)
        if self.pos is not None:
            body.set("pos", " ".join(map(str, self.pos)))
        if self.quat is not None:
            # TODO fix this
            body.set("quat", " ".join(f"{x:.8g}" for x in self.quat))
        if self.joint is not None:
            self.joint.to_xml(body)
        if self.geom is not None:
            self.geom.to_xml(body)
        return body


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
    assets: List | None = None
    parts: List | None = None
    compiler: Compiler = field(default_factory=Compiler)
    # option: Option = field(default_factory=Option)
    actuators: List | None = None

    def to_xml(self) -> ET.Element:
        robot = ET.Element("mujoco", name=self.name)
        self.compiler.to_xml(robot)
        # self.option.to_xml(robot)

        assets = ET.SubElement(robot, "assets")
        for asset in self.assets:
            asset.to_xml(assets)

        worldbody = ET.SubElement(robot, "worldbody")
        for part in self.parts:
            part.to_xml(worldbody)
        
        if self.actuators is not None:
            actuators = ET.SubElement(robot, "actuators")
            for actuator in self.actuators:
                actuator.to_xml(actuators)
    
        return robot

    def save(self, path: str | Path) -> None:
        tree = ET.ElementTree(self.to_xml())
        save_xml(path, tree)
