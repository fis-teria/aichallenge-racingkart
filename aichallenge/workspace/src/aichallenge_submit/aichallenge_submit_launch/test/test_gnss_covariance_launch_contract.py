#!/usr/bin/env python3
"""Static launch contract for simulation-only GNSS covariance fallback."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
REFERENCE = ROOT / "launch" / "reference.launch.xml"
GNSS_LAUNCH = (
    ROOT.parent / "racing_kart_gnss_poser" / "launch" / "gnss_poser.launch.xml"
)


def main() -> None:
    reference_root = ET.parse(REFERENCE).getroot()
    gnss_root = ET.parse(GNSS_LAUNCH).getroot()

    defaults = {
        node.attrib["name"]: node.attrib.get("default")
        for node in gnss_root.findall("arg")
    }
    assert defaults["unknown_position_covariance_m2"] == "10.0"

    forwarded = [
        node.attrib["value"]
        for node in reference_root.iter("arg")
        if node.attrib.get("name") == "unknown_position_covariance_m2"
    ]
    assert len(forwarded) == 1
    expression = forwarded[0]
    assert re.search(r"\b0\.1\b", expression)
    assert re.search(r"\b10\.0\b", expression)
    assert "$(var simulation)" in expression


if __name__ == "__main__":
    main()
