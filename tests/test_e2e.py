"""Runs an end-to-end test of the URDF exporter on the Stompy model."""

import asyncio
from pathlib import Path
from typing import get_args

import pytest

from kol.logging import configure_logging
from kol.mesh import MeshType
from kol.onshape.converter import Converter, ConverterConfig

# STOMPY_ONSHAPE_URL = (
#     "https://cad.onshape.com/documents/71f793a23ab7562fb9dec82d/w/"
#     "e879bcf272425973f9b3d8ad/e/1a95e260677a2d2d5a3b1eb3"
# )

ONSHAPE_URL = (
    "https://cad.onshape.com/documents/4b3eeb430e3d28511ab9cba8/w/"
    "dc58c6e8cf7c1ee8bd864ee4/e/de14dfcca89a312f32f77d02"
)


@pytest.mark.skip(reason="This test is slow and requires an internet connection")
async def test_e2e(tmpdir: Path) -> None:
    """Runs an end-to-end test of the URDF exporter on the Stompy model.

    Args:
        tmpdir: The temporary directory to save the URDF file.
        ext: The mesh file format.
    """
    for mesh_ext in get_args(MeshType):
        config = ConverterConfig(
            document_url=ONSHAPE_URL,
            output_dir=tmpdir,
            default_prismatic_joint_limits=(10, 10, -10, 10),
            default_revolute_joint_limits=(10, 10, -10, 10),
            suffix_to_joint_effort={
                "dof_x4_h": 1.5,
                "dof_x4": 1.5,
                "dof_x6": 3,
                "dof_x8": 6,
                "dof_x10": 12,
                "knee_revolute": 13.9,
                "ankle_revolute": 6,
            },
            suffix_to_joint_velocity={},
            disable_mimics=True,
            mesh_ext=mesh_ext,
        )
        await Converter(config).save_urdf()


if __name__ == "__main__":
    # python -m tests.test_e2e
    configure_logging()
    asyncio.run(test_e2e(Path.cwd() / "test_e2e"))
