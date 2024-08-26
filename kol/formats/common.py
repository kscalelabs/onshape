"""Utility functions common to different formats."""

import io
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom


def save_xml(path: str | Path | io.StringIO, tree: ET.ElementTree) -> None:
    xmlstr = minidom.parseString(ET.tostring(tree.getroot())).toprettyxml(indent="  ")
    if isinstance(path, io.StringIO):
        path.write(xmlstr)
    else:
        with open(path, "w") as f:
            f.write(xmlstr)
