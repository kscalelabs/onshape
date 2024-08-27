"""Utility functions common to different formats."""

import io
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom


def save_xml(path: str | Path | io.StringIO, tree: ET.ElementTree | ET.Element) -> None:
    if isinstance(tree, ET.ElementTree):
        tree = tree.getroot()
    xmlstr = minidom.parseString(ET.tostring(tree)).toprettyxml(indent="  ")
    xmlstr = re.sub(r"\n\s*\n", "\n", xmlstr)
    if isinstance(path, io.StringIO):
        path.write(xmlstr)
    else:
        with open(path, "w") as f:
            f.write(xmlstr)
