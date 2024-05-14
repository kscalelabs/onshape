"""Defines common types and functions for exporting URDFs."""

import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Self
from xml.sax.saxutils import escape

import numpy as np

from kol.formats.common import save_xml
from kol.geometry import rotation_matrix_to_euler_angles


def xml_escape(unescaped: str) -> str:
    return escape(unescaped, entities={"'": "&apos;", '"': "&quot;"})


def format_number(value: float) -> str:
    # Clip epsilon to avoid scientific notation
    if abs(value) < 1e-6:
        value = 0.0
    return f"{value:.8g}"


@dataclass
class Origin:
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        origin = ET.Element("origin") if root is None else ET.SubElement(root, "origin")
        origin.set("xyz", " ".join(format_number(v) for v in self.xyz))
        origin.set("rpy", " ".join(format_number(v) for v in self.rpy))
        return origin

    @classmethod
    def from_matrix(cls, matrix: np.matrix) -> Self:
        x = float(matrix[0, 3])
        y = float(matrix[1, 3])
        z = float(matrix[2, 3])
        roll, pitch, yaw = rotation_matrix_to_euler_angles(matrix[:3, :3])
        return cls((x, y, z), (roll, pitch, yaw))

    @classmethod
    def zero_origin(cls) -> Self:
        return cls((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


@dataclass
class Axis:
    xyz: tuple[float, float, float]

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        axis = ET.Element("axis") if root is None else ET.SubElement(root, "axis")
        axis.set("xyz", " ".join(format_number(v) for v in self.xyz))
        return axis


@dataclass
class JointLimits:
    effort: float  # N*m for revolute, N for prismatic
    velocity: float  # rad/s for revolute, m/s for prismatic
    lower: float  # radians for revolute, meters for prismatic
    upper: float  # radians for revolute, meters for prismatic

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        limit = ET.Element("limit") if root is None else ET.SubElement(root, "limit")
        limit.set("effort", format_number(self.effort))
        limit.set("velocity", format_number(self.velocity))
        limit.set("lower", format_number(self.lower))
        limit.set("upper", format_number(self.upper))
        return limit


@dataclass
class JointMimic:
    joint: str
    multiplier: float = 1.0
    offset: float = 0.0

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        mimic = ET.Element("mimic") if root is None else ET.SubElement(root, "mimic")
        mimic.set("joint", self.joint)
        mimic.set("multiplier", format_number(self.multiplier))
        mimic.set("offset", format_number(self.offset))
        return mimic


@dataclass
class JointDynamics:
    damping: float
    friction: float

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        joint = ET.Element("dynamics") if root is None else ET.SubElement(root, "dynamics")
        joint.set("damping", format_number(self.damping))
        joint.set("friction", format_number(self.friction))
        return joint


@dataclass
class BaseJoint(ABC):
    name: str
    parent: str
    child: str
    origin: Origin

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        joint = ET.Element("joint") if root is None else ET.SubElement(root, "joint")
        joint.set("name", self.name)
        joint.set("type", self.joint_type())
        self.origin.to_xml(joint)
        ET.SubElement(joint, "parent", link=self.parent)
        ET.SubElement(joint, "child", link=self.child)
        return joint

    @abstractmethod
    def joint_type(self) -> str: ...


@dataclass
class RevoluteJoint(BaseJoint):
    limits: JointLimits
    axis: Axis  # The axis of rotation
    dynamics: JointDynamics | None = None  # N*m*s/rad for damping, N*m for friction
    mimic: JointMimic | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        joint = super().to_xml(root)
        self.limits.to_xml(joint)
        self.axis.to_xml(joint)
        if self.dynamics is not None:
            self.dynamics.to_xml(joint)
        if self.mimic is not None:
            self.mimic.to_xml(joint)
        return joint

    def joint_type(self) -> str:
        return "revolute"


@dataclass
class ContinuousJoint(BaseJoint):
    mimic: JointMimic | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        joint = super().to_xml(root)
        if self.mimic is not None:
            self.mimic.to_xml(joint)
        return joint

    def joint_type(self) -> str:
        return "continuous"


@dataclass
class PrismaticJoint(BaseJoint):
    limits: JointLimits
    axis: Axis  # The axis of translation
    dynamics: JointDynamics | None = None  # N*s/m for damping, N for friction
    mimic: JointMimic | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        joint = super().to_xml(root)
        self.limits.to_xml(joint)
        self.axis.to_xml(joint)
        if self.dynamics is not None:
            self.dynamics.to_xml(joint)
        if self.mimic is not None:
            self.mimic.to_xml(joint)
        return joint

    def joint_type(self) -> str:
        return "prismatic"


@dataclass
class FixedJoint(BaseJoint):
    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        joint = super().to_xml(root)
        return joint

    def joint_type(self) -> str:
        return "fixed"


@dataclass
class FloatingJoint(BaseJoint):
    mimic: JointMimic | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        joint = super().to_xml(root)
        if self.mimic is not None:
            self.mimic.to_xml(joint)
        return joint

    def joint_type(self) -> str:
        return "floating"


@dataclass
class PlanarJoint(BaseJoint):
    limits: JointLimits
    axis: Axis  # The surface normal
    mimic: JointMimic | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        joint = super().to_xml(root)
        self.limits.to_xml(joint)
        self.axis.to_xml(joint)
        if self.mimic is not None:
            self.mimic.to_xml(joint)
        return joint

    def joint_type(self) -> str:
        return "planar"


@dataclass
class Inertia:
    ixx: float
    iyy: float
    izz: float
    ixy: float
    ixz: float
    iyz: float

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        inertia = ET.Element("inertia") if root is None else ET.SubElement(root, "inertia")
        inertia.set("ixx", format_number(self.ixx))
        inertia.set("iyy", format_number(self.iyy))
        inertia.set("izz", format_number(self.izz))
        inertia.set("ixy", format_number(self.ixy))
        inertia.set("ixz", format_number(self.ixz))
        inertia.set("iyz", format_number(self.iyz))
        return inertia


@dataclass
class InertialLink:
    mass: float
    inertia: Inertia
    origin: Origin

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        inertial = ET.Element("inertial") if root is None else ET.SubElement(root, "inertial")
        ET.SubElement(inertial, "mass", value=format_number(self.mass))
        self.inertia.to_xml(inertial)
        self.origin.to_xml(inertial)
        return inertial


@dataclass
class BaseGeometry(ABC):
    @abstractmethod
    def to_xml(self, root: ET.Element | None = None) -> ET.Element: ...


@dataclass
class BoxGeometry(BaseGeometry):
    size: tuple[float, float, float]

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        geometry = ET.Element("geometry") if root is None else ET.SubElement(root, "geometry")
        ET.SubElement(geometry, "box", size=" ".join(format_number(v) for v in self.size))
        return geometry


@dataclass
class CylinderGeometry(BaseGeometry):
    radius: float
    length: float

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        geometry = ET.Element("geometry") if root is None else ET.SubElement(root, "geometry")
        ET.SubElement(
            geometry,
            "cylinder",
            radius=format_number(self.radius),
            length=format_number(self.length),
        )
        return geometry


@dataclass
class SphereGeometry(BaseGeometry):
    radius: float

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        geometry = ET.Element("geometry") if root is None else ET.SubElement(root, "geometry")
        ET.SubElement(geometry, "sphere", radius=format_number(self.radius))
        return geometry


@dataclass
class MeshGeometry(BaseGeometry):
    filename: str

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        geometry = ET.Element("geometry") if root is None else ET.SubElement(root, "geometry")
        ET.SubElement(geometry, "mesh", filename=self.filename)
        return geometry

    def __post_init__(self) -> None:
        self.filename = xml_escape(self.filename)


@dataclass
class Material:
    name: str
    color: list[float]

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        material = ET.Element("material") if root is None else ET.SubElement(root, "material")
        material.set("name", self.name)
        ET.SubElement(material, "color", rgba=" ".join(format_number(v) for v in self.color))
        return material

    @classmethod
    def from_color(cls, color: Literal["red", "green", "blue", "yellow", "white", "black"]) -> Self:
        color_value = {
            "red": [1.0, 0.0, 0.0],
            "green": [0.0, 1.0, 0.0],
            "blue": [0.0, 0.0, 1.0],
            "yellow": [1.0, 1.0, 0.0],
            "white": [1.0, 1.0, 1.0],
            "black": [0.0, 0.0, 0.0],
        }[color]
        return cls(color, color_value)

    def __post_init__(self) -> None:
        if len(self.color) == 3:
            self.color.append(0.5)
        if len(self.color) != 4:
            raise ValueError(f"Color must have 3 or 4 components, got {len(self.color)}")


@dataclass
class VisualLink:
    origin: Origin
    geometry: BaseGeometry
    material: Material

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        visual = ET.Element("visual") if root is None else ET.SubElement(root, "visual")
        self.origin.to_xml(visual)
        self.geometry.to_xml(visual)
        self.material.to_xml(visual)
        return visual


@dataclass
class CollisionLink:
    origin: Origin
    geometry: BaseGeometry

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        collision = ET.Element("collision") if root is None else ET.SubElement(root, "collision")
        self.origin.to_xml(collision)
        self.geometry.to_xml(collision)
        return collision


@dataclass
class Link:
    name: str
    visual: VisualLink | None = None
    collision: CollisionLink | None = None
    inertial: InertialLink | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        link = ET.Element("link") if root is None else ET.SubElement(root, "link")
        link.set("name", self.name)
        if self.visual is not None:
            self.visual.to_xml(link)
        if self.collision is not None:
            self.collision.to_xml(link)
        if self.inertial is not None:
            self.inertial.to_xml(link)
        return link


@dataclass
class Robot:
    name: str
    parts: list[Link | BaseJoint]

    def to_xml(self) -> ET.Element:
        robot = ET.Element("robot", name=self.name)
        for part in self.parts:
            part.to_xml(robot)
        return robot

    def save(self, path: str | Path) -> None:
        tree = ET.ElementTree(self.to_xml())
        save_xml(path, tree)
