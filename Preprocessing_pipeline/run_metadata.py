import argparse
import json
import numpy as np

from pathlib import Path

def read_json(metadata: Path):
    """Load PLY and return height values"""

    locations = {}

    for pano_meta in metadata.glob("*.json"):
        with pano_meta.open("r", encoding="utf-8") as f:
            data = json.load(f)
            coord = [data["translation"]["x"], data["translation"]["y"], data["translation"]["z"]]
            locations.update({data["name"]:coord})

    return locations


def main(input_dir):

    metadata = input_dir / "metadata"
    locations = read_json(metadata)

    return locations