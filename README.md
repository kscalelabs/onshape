<p align="center">
  <picture>
    <img alt="K-Scale Open Source Robotics" src="https://media.kscale.dev/kscale-open-source-header.png" style="max-width: 100%;">
  </picture>
</p>

<div align="center">

[![License](https://img.shields.io/badge/license-MIT-green)](https://github.com/kscalelabs/onshape/blob/main/LICENSE)
[![Version](https://img.shields.io/pypi/v/kscale-onshape-library)](https://pypi.org/project/kscale-onshape-library/)
[![Discord](https://img.shields.io/discord/1224056091017478166)](https://discord.gg/k5mSvCkYQh)
[![Wiki](https://img.shields.io/badge/wiki-humanoids-black)](https://humanoids.wiki)
<br />
[![python](https://img.shields.io/badge/-Python_3.11-blue?logo=python&logoColor=white)](https://github.com/pre-commit/pre-commit)
[![black](https://img.shields.io/badge/Code%20Style-Black-black.svg?labelColor=gray)](https://black.readthedocs.io/en/stable/)
[![ruff](https://img.shields.io/badge/Linter-Ruff-red.svg?labelColor=gray)](https://github.com/charliermarsh/ruff)
<br />
[![Publish Python Package](https://github.com/kscalelabs/onshape/actions/workflows/publish.yml/badge.svg)](https://github.com/kscalelabs/onshape/actions/workflows/publish.yml)
[![Python Checks](https://github.com/kscalelabs/onshape/actions/workflows/test.yml/badge.svg)](https://github.com/kscalelabs/onshape/actions/workflows/test.yml)

</div>

# K-Scale OnShape Library

This library is what we use at K-Scale for interacting with OnShape. It is a wrapper around the OnShape API that allows us to easily import parts from OnShape into our projects.

### Getting Started

Install the library using pip:

```bash
pip install kscale-onshape-library
```

Or, if you want to install directly from Github:

```bash
pip install 'kscale-onshape-library @ git+https://github.com/kscalelabs/onshape.git@master'
```

Or, if you're developing the library, do:

```bash
git clone git@github.com:kscalelabs/onshape.git && cd onshape
conda create -y -n kol python=3.11 && conda activate kol
pip install -e '.[dev]'
```

In order to access the OnShape API, you need to define `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY` using a key generated [here](https://dev-portal.onshape.com/keys).

### Usage
To convert an assembly to a URDF, use the following command:

```bash
kol urdf https://cad.onshape.com/documents/DDDDDDDD/w/WWWWWWWW/e/EEEEEEEE
```
You can visualize the resulting URDF using PyBullet:

```bash
pip install pybullet
kol pybullet robot/<urdf-name>.urdf
```

To convert an assembly to a MJCF, use the following command:
```bash
kol mjcf https://cad.onshape.com/documents/DDDDDDDD/w/WWWWWWWW/e/EEEEEEEE
```
You can visualize the resulting MJCF using MuJoCO:

```bash
mjpython -m kol.scripts.cli mujoco robot/<mjcf-name>.xml
```

### Advanced Tools
For improving simulation speed and reducing internal collisions, you can choose to merge a URDF at each of 
its fixed joints.

``` bash
kol merge-fixed-joints robot/URDF.urdf
```

To simplify each of the meshes in a URDF using vertex clustering:
``` bash
kol simplify-all robot/URDF.urdf
```

To cleanup the meshes directory by removing all meshes not referenced in a urdf:
``` bash
kol cleanup-mesh-dir robot/URDF.urdf
```

For example, if you wanted to download the 5-DOF stompy arm, merge and simplify, then clean the directory.
``` bash
kol urdf https://cad.onshape.com/documents/afaee604f6ca311526a6aec8/w/29af84cb974c2d825b71de39/e/4fef6bce7179a665e62b03ba
&& kol merge-fixed-joints robot/assembly_1.urdf
&& kol simplify-all robot/assembly_1_merged.urdf
&& kol cleanup-mesh-dir robot/assembly_1_merged_simplified.urdf
```

### Notes

- [OnShape API explorer](https://cad.onshape.com/glassworks/explorer/#/Assembly/getFeatures)
- [OnShape API documentation](https://onshape-public.github.io/docs/api-intro/)
- [OnShape-to-Robot Library](https://github.com/Rhoban/onshape-to-robot)
