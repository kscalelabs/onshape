"""Tests the OnShape schema against sample data."""

import json

from kol.onshape.schema.assembly import Assembly
from kol.onshape.schema.features import Features


def test_assembly_schema() -> None:
    assembly_json_path = "tests/data/dummy_assembly.json"
    with open(assembly_json_path) as f:
        assembly_json = json.load(f)
    assembly = Assembly.model_validate(assembly_json)
    assert assembly is not None


def test_features_schema() -> None:
    features_json_path = "tests/data/dummy_features.json"
    with open(features_json_path) as f:
        features_json = json.load(f)
    features = Features.model_validate(features_json)
    assert features is not None


if __name__ == "__main__":
    # python -m tests.test_onshape_schema
    # test_assembly_schema()
    test_features_schema()
