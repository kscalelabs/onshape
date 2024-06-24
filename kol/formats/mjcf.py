# mypy: disable-error-code="operator,union-attr"
"""Defines common types and functions for exporting MJCF files.

API reference:
https://github.com/google-deepmind/mujoco/blob/main/src/xml/xml_native_writer.cc#L780

Todo:
    1. Clean up the inertia config
    2. Add visual and imu support
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
    autolimits: bool | None = None

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
        if self.autolimits is not None:
            compiler.set("autolimits", str(self.autolimits).lower())
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
    name: str | None = None
    type: Literal["hinge", "slide", "ball", "free"] | None = None
    pos: tuple[float, float, float] | None = None
    axis: tuple[float, float, float] | None = None
    limited: bool | None = None
    range: tuple[float, float] | None = None
    damping: float | None = None
    stiffness: float | None = None
    armature: float | None = None
    frictionloss: float | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        joint = ET.Element("joint") if root is None else ET.SubElement(root, "joint")
        if self.name is not None:
            joint.set("name", self.name)
        if self.type is not None:
            joint.set("type", self.type)
        if self.pos is not None:
            joint.set("pos", " ".join(map(str, self.pos)))
        if self.axis is not None:
            joint.set("axis", " ".join(map(str, self.axis)))
        if self.range is not None:
            joint.set("range", " ".join(map(str, self.range)))
        if self.limited is not None:
            joint.set("limited", str(self.limited).lower())
        if self.damping is not None:
            joint.set("damping", str(self.damping))
        if self.stiffness is not None:
            joint.set("stiffness", str(self.stiffness))
        if self.armature is not None:
            joint.set("armature", str(self.armature))
        if self.frictionloss is not None:
            joint.set("frictionloss", str(self.frictionloss))
        return joint


@dataclass
class Geom:
    name: str | None = None
    type: Literal["plane", "sphere", "cylinder", "box", "capsule", "ellipsoid", "mesh"] | None = None
    plane: str | None = None
    rgba: tuple[float, float, float, float] | None = None
    pos: tuple[float, float, float] | None = None
    quat: tuple[float, float, float, float] | None = None
    matplane: str | None = None
    material: str | None = None
    condim: int | None = None
    contype: int | None = None
    conaffinity: int | None = None
    size: tuple[float, float, float] | None = None
    friction: tuple[float, float, float] | None = None
    solref: tuple[float, float] | None = None
    density: float | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        geom = ET.Element("geom") if root is None else ET.SubElement(root, "geom")
        if self.name is not None:
            geom.set("name", self.name)
        if self.type is not None:
            geom.set("type", self.type)
        if self.rgba is not None:
            geom.set("rgba", " ".join(map(str, self.rgba)))
        if self.pos is not None:
            geom.set("pos", " ".join(map(str, self.pos)))
        if self.quat is not None:
            geom.set("quat", " ".join(map(str, self.quat)))
        if self.matplane is not None:
            geom.set("matplane", self.matplane)
        if self.material is not None:
            geom.set("material", self.material)
        if self.condim is not None:
            geom.set("condim", str(self.condim))
        if self.contype is not None:
            geom.set("contype", str(self.contype))
        if self.conaffinity is not None:
            geom.set("conaffinity", str(self.conaffinity))
        if self.plane is not None:
            geom.set("plane", self.plane)
        if self.size is not None:
            geom.set("size", " ".join(map(str, self.size)))
        if self.friction is not None:
            geom.set("friction", " ".join(map(str, self.friction)))
        if self.solref is not None:
            geom.set("solref", " ".join(map(str, self.solref)))
        if self.density is not None:
            geom.set("density", str(self.density))
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
class Flag:
    frictionloss: str | None = None
    # removed at 3.1.4
    # sensornoise: str | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        flag = ET.Element("flag") if root is None else ET.SubElement(root, "flag")
        if self.frictionloss is not None:
            flag.set("frictionloss", self.frictionloss)
        return flag


@dataclass
class Option:
    timestep: float | None = None
    viscosity: float | None = None
    iterations: int | None = None
    solver: Literal["PGS", "CG", "Newton"] | None = None
    gravity: tuple[float, float, float] | None = None
    flag: Flag | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        if root is None:
            option = ET.Element("option")
        else:
            option = ET.SubElement(root, "option")
        if self.iterations is not None:
            option.set("iterations", str(self.iterations))
        if self.timestep is not None:
            option.set("timestep", str(self.timestep))
        if self.viscosity is not None:
            option.set("viscosity", str(self.viscosity))
        if self.solver is not None:
            option.set("solver", self.solver)
        if self.gravity is not None:
            option.set("gravity", " ".join(map(str, self.gravity)))
        if self.flag is not None:
            self.flag.to_xml(option)
        return option


@dataclass
class Motor:
    name: str | None = None
    joint: str | None = None
    ctrlrange: tuple[float, float] | None = None
    ctrllimited: bool | None = None
    gear: float | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        if root is None:
            motor = ET.Element("motor")
        else:
            motor = ET.SubElement(root, "motor")
        if self.name is not None:
            motor.set("name", self.name)
        if self.joint is not None:
            motor.set("joint", self.joint)
        if self.ctrllimited is not None:
            motor.set("ctrllimited", str(self.ctrllimited).lower())
        if self.ctrlrange is not None:
            motor.set("ctrlrange", " ".join(map(str, self.ctrlrange)))
        if self.gear is not None:
            motor.set("gear", str(self.gear))
        return motor


@dataclass
class Light:
    directional: bool = True
    diffuse: tuple[float, float, float] | None = None
    specular: tuple[float, float, float] | None = None
    pos: tuple[float, float, float] | None = None
    dir: tuple[float, float, float] | None = None
    castshadow: bool | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        if root is None:
            light = ET.Element("light")
        else:
            light = ET.SubElement(root, "light")
        if self.directional is not None:
            light.set("directional", str(self.directional).lower())
        if self.diffuse is not None:
            light.set("diffuse", " ".join(map(str, self.diffuse)))
        if self.specular is not None:
            light.set("specular", " ".join(map(str, self.specular)))
        if self.pos is not None:
            light.set("pos", " ".join(map(str, self.pos)))
        if self.dir is not None:
            light.set("dir", " ".join(map(str, self.dir)))
        if self.castshadow is not None:
            light.set("castshadow", str(self.castshadow).lower())
        return light


@dataclass
class Equality:
    solref: tuple[float, float]

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        equality = ET.Element("equality") if root is None else ET.SubElement(root, "equality")
        equality.set("solref", " ".join(map(str, self.solref)))
        return equality


@dataclass
class Site:
    name: str | None = None
    size: float | None = None
    pos: tuple[float, float, float] | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        site = ET.Element("site") if root is None else ET.SubElement(root, "site")
        if self.name is not None:
            site.set("name", self.name)
        if self.size is not None:
            site.set("size", str(self.size))
        if self.pos is not None:
            site.set("pos", " ".join(map(str, self.pos)))
        return site


@dataclass
class Default:
    joint: Joint | None = None
    geom: Geom | None = None
    class_: str | None = None
    motor: Motor | None = None
    equality: Equality | None = None
    visual_geom: ET.Element | None = None

    def to_xml(self, default: ET.Element | None = None) -> ET.Element:
        default = ET.Element("default") if default is None else ET.SubElement(default, "default")
        if self.joint is not None:
            self.joint.to_xml(default)
        if self.geom is not None:
            self.geom.to_xml(default)
        if self.class_ is not None:
            default.set("class", self.class_)
        if self.motor is not None:
            self.motor.to_xml(default)
        if self.equality is not None:
            self.equality.to_xml(default)
        if self.visual_geom is not None:
            default.append(self.visual_geom)
        return default


@dataclass
class Actuator:
    motors: list[Motor]

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        actuator = ET.Element("actuator") if root is None else ET.SubElement(root, "actuator")
        for motor in self.motors:
            motor.to_xml(actuator)
        return actuator


@dataclass
class Actuatorpos:
    name: str | None = None
    actuator: str | None = None
    user: str | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        actuatorpos = ET.Element("actuatorpos") if root is None else ET.SubElement(root, "actuatorpos")
        if self.name is not None:
            actuatorpos.set("name", self.name)
        if self.actuator is not None:
            actuatorpos.set("actuator", self.actuator)
        if self.user is not None:
            actuatorpos.set("user", self.user)
        return actuatorpos


@dataclass
class Actuatorvel:
    name: str | None = None
    actuator: str | None = None
    user: str | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        actuatorvel = ET.Element("actuatorvel") if root is None else ET.SubElement(root, "actuatorvel")
        if self.name is not None:
            actuatorvel.set("name", self.name)
        if self.actuator is not None:
            actuatorvel.set("actuator", self.actuator)
        if self.user is not None:
            actuatorvel.set("user", self.user)
        return actuatorvel


@dataclass
class Actuatorfrc:
    name: str | None = None
    actuator: str | None = None
    user: str | None = None
    noise: float | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        actuatorfrc = ET.Element("actuatorfrc") if root is None else ET.SubElement(root, "actuatorfrc")
        if self.name is not None:
            actuatorfrc.set("name", self.name)
        if self.actuator is not None:
            actuatorfrc.set("actuator", self.actuator)
        if self.user is not None:
            actuatorfrc.set("user", self.user)
        if self.noise is not None:
            actuatorfrc.set("noise", str(self.noise))
        return actuatorfrc


@dataclass
class Sensor:
    actuatorpos: list[Actuatorpos] | None = None
    actuatorvel: list[Actuatorvel] | None = None
    actuatorfrc: list[Actuatorfrc] | None = None

    def to_xml(self, root: ET.Element | None = None) -> ET.Element:
        sensor = ET.Element("sensor") if root is None else ET.SubElement(root, "sensor")
        if self.actuatorpos is not None:
            for actuatorpos in self.actuatorpos:
                actuatorpos.to_xml(sensor)
        if self.actuatorvel is not None:
            for actuatorvel in self.actuatorvel:
                actuatorvel.to_xml(sensor)
        if self.actuatorfrc is not None:
            for actuatorfrc in self.actuatorfrc:
                actuatorfrc.to_xml(sensor)
        return sensor


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

    def __init__(
        self,
        robot_name: str,
        output_dir: str | Path,
        compiler: Compiler | None = None,
    ) -> None:
        """Initialize the robot.

        Args:
            robot_name (str): The name of the robot.
            output_dir (str | Path): The output directory.
            compiler (Compiler, optional): The compiler settings.
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

        urdf_tree = ET.parse(self.output_dir / f"{self.robot_name}.urdf")
        root = urdf_tree.getroot()

        tree = ET.ElementTree(root)
        tree.write(self.output_dir / f"{self.robot_name}.urdf", encoding="utf-8", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path((self.output_dir / f"{self.robot_name}.urdf").as_posix())
        mujoco.mj_saveLastXML((self.output_dir / f"{self.robot_name}.xml").as_posix(), model)
        # remove all the files
        _remove_stl_files(self.output_dir)

    def adapt_world(self) -> None:
        root = self.tree.getroot()

        # Turn off internal collisions
        for element in root:
            if element.tag == "geom":
                element.attrib["contype"] = str(1)
                element.attrib["conaffinity"] = str(0)

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
