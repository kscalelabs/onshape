# mypy: disable-error-code="operator,union-attr"
"""Defines common types and functions for exporting MJCF files.

API reference:
https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_native_writer.cc#L780
"""

import glob
import os
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

import mujoco


@dataclass
class Compiler:
    coordinate: Literal["local", "global"] | None = None
    angle: Literal["radian", "degree"] = "radian"
    meshdir: str | None = None
    eulerseq: Literal["xyz", "zxy", "zyx", "yxz", "yzx", "xzy"] | None = None

    def to_xml(self, compiler: ET.Element | None = None) -> ET.Element:
        if compiler is None:
            compiler = ET.Element("compiler")
        if self.coordinate is not None:
            compiler.set("coordinate", self.coordinate)
        compiler.set("angle", self.angle)
        if self.meshdir is not None:
            compiler.set("meshdir", self.meshdir)
        if self.eulerseq is not None:
            compiler.set("eulerseq", self.eulerseq)
        return compiler


@dataclass
class Mesh:
    name: str
    file: str
    scale: tuple[float, float, float] | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        mesh = ET.Element("mesh") if root is None else ET.SubElement(root, "mesh")
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

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        joint = ET.Element("joint") if root is None else ET.SubElement(root, "joint")
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
    pos: tuple[float, float, float] | None = None
    quat: tuple[float, float, float, float] | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        geom = ET.Element("geom") if root is None else ET.SubElement(root, "geom")
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
    pos: tuple[float, float, float] | None = field(default=None)
    quat: tuple[float, float, float, float] | None = field(default=None)
    geom: Geom | None = field(default=None)
    joint: Joint | None = field(default=None)
    # TODO - fix inertia, until then rely on Mujoco's engine
    # inertial: Inertial = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        body = ET.Element("body") if root is None else ET.SubElement(root, "body")
        body.set("name", self.name)
        if self.pos is not None:
            body.set("pos", " ".join(map(str, self.pos)))
        if self.quat is not None:
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

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        option = ET.Element("option") if root is None else ET.SubElement(root, "option")
        option.set("timestep", str(self.timestep))
        option.set("viscosity", str(self.viscosity))
        return option


@dataclass
class Actuator:
    name: str
    joint: str
    ctrlrange: tuple[float, float]

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        actuator = ET.Element("actuator") if root is None else ET.SubElement(root, "actuator")
        actuator.set("name", self.name)
        actuator.set("joint", self.joint)
        actuator.set("ctrlrange", " ".join(map(str, self.ctrlrange)))
        return actuator


def _copy_stl_files(source_directory: str | Path, destination_directory: str | Path) -> None:
    # Ensure the destination directory exists, create if not
    os.makedirs(destination_directory, exist_ok=True)

    # Use glob to find all .stl files in the source directory
    pattern = os.path.join(source_directory, "*.stl")
    for file_path in glob.glob(pattern):
        destination_path = os.path.join(destination_directory, os.path.basename(file_path))
        shutil.copy(file_path, destination_path)
        print(f"Copied {file_path} to {destination_path}")


def _remove_stl_files(source_directory: str | Path) -> None:
    for filename in os.listdir(source_directory):
        if filename.endswith(".stl"):
            file_path = os.path.join(source_directory, filename)
            os.remove(file_path)


class Robot:
    """A class to adapt the world in a Mujoco XML file."""

    def __init__(self, robot_name: str, output_dir: str | Path, compiler: Compiler | None = None) -> None:
        """Initialize the robot.

        Args:
            robot_name (str): The name of the robot.
            output_dir (str | Path): The output directory.
            compiler (Compiler, optional): The compiler settings. Defaults to None.
        """
        self.robot_name = robot_name
        self.output_dir = output_dir
        self.compiler = compiler
        self._set_clean_up()
        self.tree = ET.parse(self.output_dir / f"{self.robot_name}.xml")

    def _set_clean_up(self) -> None:
        # HACK
        # mujoco has a hard time reading meshes
        _copy_stl_files(self.output_dir / "meshes", self.output_dir)
        # remove inertia tags
        urdf_tree = ET.parse(self.output_dir / f"{self.robot_name}.urdf")
        root = urdf_tree.getroot()
        for link in root.findall(".//link"):
            inertial = link.find("inertial")
            if inertial is not None:
                link.remove(inertial)

        tree = ET.ElementTree(root)
        tree.write(self.output_dir / f"{self.robot_name}.urdf", encoding="utf-8", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path((self.output_dir / f"{self.robot_name}.urdf").as_posix())
        mujoco.mj_saveLastXML((self.output_dir / f"{self.robot_name}.xml").as_posix(), model)
        # remove all the files
        _remove_stl_files(self.output_dir)

    def adapt_world(self) -> None:
        root = self.tree.getroot()

        compiler = root.find("compiler")
        if self.compiler is not None:
            compiler = self.compiler.to_xml(compiler)

        worldbody = root.find("worldbody")
        new_root_body = Body(name="root", pos=(0, 0, 0), quat=(1, 0, 0, 0)).to_xml()

        # List to store items to be moved to the new root body
        items_to_move = []
        # Gather all children (geoms and bodies) that need to be moved under the new root body
        for element in worldbody:
            items_to_move.append(element)
        # Move gathered elements to the new root body
        for item in items_to_move:
            worldbody.remove(item)
            new_root_body.append(item)

        # Add the new root body to the worldbody
        worldbody.append(new_root_body)
        self.tree = ET.ElementTree(root)

    def save(self, path: str | Path) -> None:
        self.tree.write(path, encoding="utf-8", xml_declaration=True)
