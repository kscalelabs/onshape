"""Runs an end-to-end test of the URDF exporter on the Stompy model."""

import asyncio
from pathlib import Path

import pytest

from onshape.onshape.config import ConverterConfig
from onshape.onshape.download import download
from onshape.onshape.postprocess import postprocess
from onshape.utils.logging import configure_logging

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
    config = ConverterConfig(
        document_url=ONSHAPE_URL,
        output_dir=str(tmpdir),
        default_prismatic_joint_effort=100,
        default_prismatic_joint_velocity=5,
        default_revolute_joint_effort=100,
        default_revolute_joint_velocity=5,
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
    )

    document_info = await download(
        document_url=config.document_url,
        output_dir=config.output_dir,
        config=config,
    )

    await postprocess(
        urdf_path=document_info.urdf_info.urdf_path,
        config=config,
    )


if __name__ == "__main__":
    # python -m tests.test_e2e
    configure_logging()
    asyncio.run(test_e2e(Path.cwd() / "test_e2e"))
