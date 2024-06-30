"""Utility functions common to different formats."""

import xml.etree.ElementTree as ET
from pathlib import Path


def save_xml(path: str | Path, tree: ET.ElementTree) -> None:
    root = tree.getroot()

    def indent(elem: ET.Element, level: int = 0) -> ET.Element:
        i = "\n" + level * "  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "  "
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for e in elem:
                indent(e, level + 1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:  # noqa: PLR5501
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i
        return elem

    indent(root)
    tree.write(path, encoding="utf-8", xml_declaration=True, method="xml")
