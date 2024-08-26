"""Convert URDF to MJCF with physics properties and collision geoms."""

import argparse
import logging
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from kol.formats.common import save_xml


def convert_joint_type(urdf_type: str) -> str:
    type_map = {
        "revolute": "hinge",
        "prismatic": "slide",
        "continuous": "hinge",
        "fixed": "fixed",
        "floating": "free",
        "planar": "slide",
    }
    return type_map.get(urdf_type, "hinge")


def parse_xyz(xyz_str: str) -> str:
    return " ".join([str(float(x)) for x in xyz_str.split()])


def convert_urdf_to_mjcf(urdf_file: str | Path, mjcf_file: str | Path) -> None:
    urdf_tree = ET.parse(urdf_file)
    urdf_root = urdf_tree.getroot()

    mjcf_root = ET.Element("mujoco")
    mjcf_root.set("model", "urdf_model")

    # Add default settings.
    default = ET.SubElement(mjcf_root, "default")

    default_joint = ET.SubElement(default, "joint")
    default_joint.set("limited", "true")

    default_motor = ET.SubElement(default, "motor")
    default_motor.set("ctrllimited", "true")

    default_geom = ET.SubElement(default, "geom")
    default_geom.set("condim", "4")
    default_geom.set("contype", "1")
    default_geom.set("conaffinity", "15")
    default_geom.set("solref", "0.001 2")
    default_geom.set("friction", "0.9 0.2 0.2")

    default_equality = ET.SubElement(default, "equality")
    default_equality.set("solref", "0.001 2")

    compiler = ET.SubElement(mjcf_root, "compiler")
    compiler.set("angle", "radian")
    compiler.set("coordinate", "local")

    option = ET.SubElement(mjcf_root, "option")
    option.set("gravity", "0 0 -9.81")
    option.set("timestep", "0.002")
    option.set("iterations", "50")
    option.set("solver", "Newton")
    option.set("tolerance", "1e-10")

    asset = ET.SubElement(mjcf_root, "asset")

    # Add textures.
    skybox = ET.SubElement(asset, "texture")
    skybox.set("type", "skybox")
    skybox.set("builtin", "gradient")
    skybox.set("rgb1", "0.3 0.5 0.7")
    skybox.set("rgb2", "0 0 0")
    skybox.set("width", "512")
    skybox.set("height", "512")

    texplane = ET.SubElement(asset, "texture")
    texplane.set("name", "texplane")
    texplane.set("type", "2d")
    texplane.set("builtin", "checker")
    texplane.set("rgb1", ".2 .3 .4")
    texplane.set("rgb2", ".1 0.15 0.2")
    texplane.set("width", "512")
    texplane.set("height", "512")
    texplane.set("mark", "cross")
    texplane.set("markrgb", ".8 .8 .8")

    # Add materials.
    matplane = ET.SubElement(asset, "material")
    matplane.set("name", "matplane")
    matplane.set("reflectance", "0.")
    matplane.set("texture", "texplane")
    matplane.set("texrepeat", "1 1")
    matplane.set("texuniform", "true")

    worldbody = ET.SubElement(mjcf_root, "worldbody")

    # Add a floor plane.
    floor = ET.SubElement(worldbody, "geom")
    floor.set("name", "floor")
    floor.set("type", "plane")
    floor.set("pos", "0.01 0 0")
    floor.set("size", "10 10 0.1")
    floor.set("material", "matplane")

    # Add lights.
    light1 = ET.SubElement(worldbody, "light")
    light1.set("directional", "true")
    light1.set("diffuse", ".4 .4 .4")
    light1.set("specular", "0.1 0.1 0.1")
    light1.set("pos", "0 0 5.0")
    light1.set("dir", "0 0 -1")
    light1.set("castshadow", "false")

    light2 = ET.SubElement(worldbody, "light")
    light2.set("directional", "true")
    light2.set("diffuse", ".6 .6 .6")
    light2.set("specular", "0.2 0.2 0.2")
    light2.set("pos", "0 0 4")
    light2.set("dir", "0 0 -1")

    # Add actuators.
    actuator = ET.SubElement(mjcf_root, "actuator")

    # Add sensors.
    sensor = ET.SubElement(mjcf_root, "sensor")

    bodies = {}

    for link in urdf_root.findall("link"):
        body = ET.SubElement(worldbody, "body")
        body.set("name", link.get("name") or "")
        bodies[link.get("name") or ""] = body

        # Add IMU site and freejoint for gravity to the base link (assuming the first link is the base).
        if link == urdf_root.find("link"):
            imu_site = ET.SubElement(body, "site")
            imu_site.set("name", "imu_site")
            imu_site.set("pos", "0 0 0")
            imu_site.set("size", "0.01")

            freejoint = ET.SubElement(body, "freejoint")
            freejoint.set("name", "root")

            body.set("pos", "0 0 1")

        # Handle inertial properties.
        inertial = link.find("inertial")
        if inertial is not None:
            inertial_mjcf = ET.SubElement(body, "inertial")

            mass = inertial.find("mass")
            if mass is not None:
                inertial_mjcf.set("mass", mass.get("value", "1"))

            inertia = inertial.find("inertia")
            if inertia is not None:
                inertial_mjcf.set(
                    "diaginertia", f"{inertia.get('ixx', '1')} {inertia.get('iyy', '1')} {inertia.get('izz', '1')}"
                )

            origin = inertial.find("origin")
            if origin is not None:
                pos = origin.get("xyz")
                if pos:
                    inertial_mjcf.set("pos", parse_xyz(pos))
                rpy = origin.get("rpy")
                if rpy:
                    # Default orientation, you may need to convert rpy to quaternion
                    inertial_mjcf.set("quat", "0 0 0 1")

        for visual in link.findall("visual"):
            geom = ET.SubElement(body, "geom")
            geom.set("name", f"{link.get('name')}_visual")

            geometry = visual.find("geometry")
            if geometry is not None:
                if geometry.find("mesh") is not None:
                    mesh = geometry.find("mesh")
                    filename = mesh.get("filename") if mesh is not None else None
                    if filename:
                        mesh_name = os.path.basename(filename)
                        mesh_asset = ET.SubElement(asset, "mesh")
                        mesh_asset.set("name", mesh_name)
                        mesh_asset.set("file", filename)
                        geom.set("type", "mesh")
                        geom.set("mesh", mesh_name)
                        scale = mesh.get("scale") if mesh is not None else None
                        if scale:
                            geom.set("scale", parse_xyz(scale))

                elif geometry.find("box") is not None:
                    box = geometry.find("box")
                    geom.set("type", "box")
                    size = box.get("size", "0.1 0.1 0.1") if box is not None else "0.1 0.1 0.1"
                    geom.set("size", parse_xyz(size))

                elif geometry.find("cylinder") is not None:
                    cylinder = geometry.find("cylinder")
                    geom.set("type", "cylinder")
                    radius = cylinder.get("radius", "0.1") if cylinder is not None else "0.1"
                    length = cylinder.get("length", "0.1") if cylinder is not None else "0.1"
                    geom.set("size", f"{radius} {length}")

                elif geometry.find("sphere") is not None:
                    sphere = geometry.find("sphere")
                    geom.set("type", "sphere")
                    radius = sphere.get("radius", "0.1") if sphere is not None else "0.1"
                    geom.set("size", radius)

                else:
                    geom.set("type", "sphere")
                    geom.set("size", "0.01")

            material = visual.find("material")
            if material is not None:
                color = material.find("color")
                if color is not None:
                    rgba = color.get("rgba", "0.8 0.8 0.8 1")
                    geom.set("rgba", rgba)

            else:
                geom.set("rgba", "0.8 0.8 0.8 1")

    for joint in urdf_root.findall("joint"):
        parent_elem = joint.find("parent")
        child_elem = joint.find("child")
        parent = parent_elem.get("link") if parent_elem is not None else None
        child = child_elem.get("link") if child_elem is not None else None

        if child in bodies:
            urdf_joint_type = joint.get("type")
            if urdf_joint_type is not None:
                mjcf_joint_type = convert_joint_type(urdf_joint_type)

                if mjcf_joint_type != "fixed":
                    mjcf_joint = ET.SubElement(bodies[child], "joint")
                    mjcf_joint.set("name", joint.get("name") or "")
                    mjcf_joint.set("type", mjcf_joint_type)

                axis = joint.find("axis")
                if axis is not None:
                    mjcf_joint.set("axis", parse_xyz(axis.get("xyz", "0 0 1")))

                limit = joint.find("limit")
                if limit is not None:
                    lower = limit.get("lower")
                    upper = limit.get("upper")
                    if lower is not None and upper is not None:
                        mjcf_joint.set("range", f"{lower} {upper}")

                    # Add damping and friction to joints
                    mjcf_joint.set("damping", "1")
                    mjcf_joint.set("frictionloss", "0.1")

                    # Add motor actuator for the joint
                    motor = ET.SubElement(actuator, "motor")
                    motor.set("name", f"motor_{joint.get('name') or ''}")
                    motor.set("joint", joint.get("name") or "")
                    motor.set("gear", "100")  # You may want to adjust this value

                # Add control limits to the motor
                motor.set("ctrlrange", "-1 1")  # Default range if limit is not specified

                # Add sensors for the joint
                joint_name = joint.get("name")

                actuatorpos = ET.SubElement(sensor, "actuatorpos")
                actuatorpos.set("name", f"{joint_name}_pos")
                actuatorpos.set("actuator", f"motor_{joint_name}")

                actuatorvel = ET.SubElement(sensor, "actuatorvel")
                actuatorvel.set("name", f"{joint_name}_vel")
                actuatorvel.set("actuator", f"motor_{joint_name}")

                actuatorfrc = ET.SubElement(sensor, "actuatorfrc")
                actuatorfrc.set("name", f"{joint_name}_frc")
                actuatorfrc.set("actuator", f"motor_{joint_name}")

        if parent in bodies and child in bodies:
            origin = joint.find("origin")
            if origin is not None:
                xyz = origin.get("xyz", "0 0 0")
                bodies[child].set("pos", parse_xyz(xyz))
                rpy = origin.get("rpy")
                if rpy:
                    bodies[child].set("euler", parse_xyz(rpy))
            else:
                bodies[child].set("pos", "0 0 0")
            bodies[parent].append(bodies[child])
            worldbody.remove(bodies[child])

    # Add IMU sensors
    framequat = ET.SubElement(sensor, "framequat")
    framequat.set("name", "orientation")
    framequat.set("objtype", "site")
    framequat.set("objname", "imu_site")

    framepos = ET.SubElement(sensor, "framepos")
    framepos.set("name", "position")
    framepos.set("objtype", "site")
    framepos.set("objname", "imu_site")

    gyro = ET.SubElement(sensor, "gyro")
    gyro.set("name", "angular_velocity")
    gyro.set("site", "imu_site")

    velocimeter = ET.SubElement(sensor, "velocimeter")
    velocimeter.set("name", "linear_velocity")
    velocimeter.set("site", "imu_site")

    accelerometer = ET.SubElement(sensor, "accelerometer")
    accelerometer.set("name", "linear_acceleration")
    accelerometer.set("site", "imu_site")

    magnetometer = ET.SubElement(sensor, "magnetometer")
    magnetometer.set("name", "magnetometer")
    magnetometer.set("site", "imu_site")

    save_xml(mjcf_file, ET.ElementTree(mjcf_root))


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    logger = logging.getLogger(__name__)

    parser = argparse.ArgumentParser(description="Convert URDF to MJCF")
    parser.add_argument("urdf_file", help="Input URDF file")
    parser.add_argument("mjcf_file", help="Output MJCF file")
    args = parser.parse_args()

    try:
        convert_urdf_to_mjcf(args.urdf_file, args.mjcf_file)
        logger.info("Conversion complete. MJCF file saved as %s", args.mjcf_file)
    except Exception:
        logger.exception("An error occurred during conversion")


if __name__ == "__main__":
    # python -m kol.formats.urdf_to_mjcf
    main()
