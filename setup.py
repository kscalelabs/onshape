# mypy: disable-error-code="import-untyped, import-not-found"
#!/usr/bin/env python
"""Setup script for the project."""

import re

from setuptools import setup

with open("README.md", "r", encoding="utf-8") as f:
    long_description: str = f.read()


with open("onshape/requirements.txt", "r", encoding="utf-8") as f:
    requirements: list[str] = f.read().splitlines()


with open("onshape/requirements-dev.txt", "r", encoding="utf-8") as f:
    requirements_dev: list[str] = f.read().splitlines()


requirements_mujoco = ["mujoco", "urdf2mjcf", "mujoco-python-viewer"]
requirements_open3d = ["open3d"]
requirements_all = requirements_dev + requirements_mujoco + requirements_open3d


with open("onshape/__init__.py", "r", encoding="utf-8") as fh:
    version_re = re.search(r"^__version__ = \"([^\"]*)\"", fh.read(), re.MULTILINE)
assert version_re is not None, "Could not find version in onshape/__init__.py"
version: str = version_re.group(1)


setup(
    name="onshape",
    version=version,
    description="K-Scale's library for programmatically interacting with OnShape",
    author="Benjamin Bolte",
    url="https://github.com/kscalelabs/onshape",
    license_files=("LICENSE",),
    long_description=long_description,
    long_description_content_type="text/markdown",
    python_requires=">=3.11",
    install_requires=requirements,
    tests_require=requirements_dev,
    extras_require={
        "dev": requirements_dev,
        "mujoco": requirements_mujoco,
        "open3d": requirements_open3d,
        "all": requirements_all,
    },
    package_data={
        "onshape": [
            "py.typed",
            "requirements*.txt",
            "scripts/bullet/*.obj",
            "scripts/bullet/*.urdf",
        ],
    },
)
