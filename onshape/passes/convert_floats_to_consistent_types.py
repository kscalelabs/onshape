"""Converts floating point numbers to consistent types."""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from onshape.formats.common import save_xml

logger = logging.getLogger(__name__)


def flstr(value: float) -> str:
    if abs(value) < 1e-6:
        return "0"
    if abs(value - 1) < 1e-6:
        return "1"
    return f"{value:.6f}"


def flstrs(values: str) -> str:
    return " ".join(flstr(float(value)) for value in values.split())


def convert_floats_to_consistent_types(urdf_path: Path) -> None:
    """Converts floating point numbers to consistent types."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Fix up joint.
    for joint in root.findall("joint"):
        logger.info("Converting joint %s", joint.get("name"))
        if (origin := joint.find("origin")) is not None:
            if (xyz := origin.get("xyz")) is not None:
                origin.set("xyz", flstrs(xyz))
            if (rpy := origin.get("rpy")) is not None:
                origin.set("rpy", flstrs(rpy))
        if (limit := joint.find("limit")) is not None:
            if (lower := limit.get("lower")) is not None:
                limit.set("lower", flstr(float(lower)))
            if (upper := limit.get("upper")) is not None:
                limit.set("upper", flstr(float(upper)))

    # Fix up link.
    for link in root.findall("link"):
        logger.info("Converting link %s", link.get("name"))
        if (visual := link.find("visual")) is not None:
            if (origin := visual.find("origin")) is not None:
                if (xyz := origin.get("xyz")) is not None:
                    origin.set("xyz", flstrs(xyz))
                if (rpy := origin.get("rpy")) is not None:
                    origin.set("rpy", flstrs(rpy))
            if (material := visual.find("material")) is not None:
                if (color := material.find("color")) is not None:
                    if (rgba := color.get("rgba")) is not None:
                        color.set("rgba", flstrs(rgba))
        if (collision := link.find("collision")) is not None:
            if (origin := collision.find("origin")) is not None:
                if (xyz := origin.get("xyz")) is not None:
                    origin.set("xyz", flstrs(xyz))
                if (rpy := origin.get("rpy")) is not None:
                    origin.set("rpy", flstrs(rpy))
        if (inertial := link.find("inertial")) is not None:
            if (mass := inertial.find("mass")) is not None:
                if (value := mass.get("value")) is not None:
                    mass.set("value", flstr(float(value)))
            if (origin := inertial.find("origin")) is not None:
                if (xyz := origin.get("xyz")) is not None:
                    origin.set("xyz", flstrs(xyz))
                if (rpy := origin.get("rpy")) is not None:
                    origin.set("rpy", flstrs(rpy))
            if (inertia := inertial.find("inertia")) is not None:
                if (ixx := inertia.get("ixx")) is not None:
                    inertia.set("ixx", flstr(float(ixx)))
                if (iyy := inertia.get("iyy")) is not None:
                    inertia.set("iyy", flstr(float(iyy)))
                if (izz := inertia.get("izz")) is not None:
                    inertia.set("izz", flstr(float(izz)))
                if (ixy := inertia.get("ixy")) is not None:
                    inertia.set("ixy", flstr(float(ixy)))
                if (ixz := inertia.get("ixz")) is not None:
                    inertia.set("ixz", flstr(float(ixz)))
                if (iyz := inertia.get("iyz")) is not None:
                    inertia.set("iyz", flstr(float(iyz)))

    save_xml(urdf_path, tree)


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("urdf_path", type=Path)
    args = parser.parse_args()

    convert_floats_to_consistent_types(args.urdf_path)


if __name__ == "__main__":
    main()
