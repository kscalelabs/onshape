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
pip install 'kscale-onshape-library @ git+https://github.com/kscalelabs/onshape.git@master'  # Install from Github
pip install 'kscale-onshape-library[all]'  # Install all dependencies
```

In order to access the OnShape API, you need to define `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY` using a key generated [here](https://dev-portal.onshape.com/keys).

### Usage

To download and postprocess an assembly, run:

```bash
kol run https://cad.onshape.com/documents/DDDDDDDD/w/WWWWWWWW/e/EEEEEEEE
```

By default, this will automatically generate both a URDF and MJCF file

You can visualize the resulting artifacts using MuJoCo Viewer:

```bash
python -m mujoco.viewer
```

For URDF files it's sometimes necessary to first drag the file into the meshes folder before viewing it. 

You can also visualize the resulting URDF using PyBullet:

```bash
pip install pybullet
kol pybullet robot/<urdf-name>.urdf
```

### Simulation

The output of the onshape library is simply a robot floating in space. Luckily, most simulators which support URDFs are able to define an environment within code. More changes are needed to make MJCF files simulation ready. We have support for adapting MJCF files to isaac sim here: https://github.com/kscalelabs/sim/blob/master/sim/scripts/create_mjcf.py.

It will be useful to create more automatic scripts for adapting raw URDF and MJCF to various simulators. Feel free to contribute! Support for other file formats like USD files for IsaacLab will be helpful as well.

### Tips and Suggestions

- To avoid mishaps from occuring due to cached files, if making big changes in a single export, or if the robot has undergone major changes since the last export, it's good to start fresh with a new `/robot/` folder. 
- There's no guarantee that the robot's starting pose (when all joint positions are set to 0) will be good, or even matching what you see on onshape. It's reccomended to use MuJoCo Viewer to find a good starting position for each joint.
- Robots function much better in simulation when they have less parts. It's very good to make sure that link names are descriptive in onshape so small parts can be better removed using `--skip-small-parts`. This will also make export faster.
- When using functions that can be destructive like `simplify-meshes`, `merge-joints`, and especially `cleanup-mesh-dir`, it's good to make copies of the STL files elsewhere first.

### Development

Since this is a library, it's necessary to upversion after big changes. You can do this by changing the version number in `kol/__init__.py`, then running the Github Action to publish a new version. 

Please keep this repo well linted! Specifically, it is quite easy to run into errors with mypy since many of the functions and variables have ambiguous types. To check which files have issues locally, run `mypy kol` and `make static-checks`. Many mypy issues can be alleviated with one of the following:

- Ensure that type annotations properly include `None` when necessary. This is especially important for functions.
- Make good use of classes to ensure everything is well typed. See `kol/onshape/schema/assembly.py`. 
- When using functions such as `dict.find`, use a `None` check to guarantee that variable accesses are safe
- Make sure docstrings are descriptive and that there's one for every function, even if it seems trivial. This makes mouseovers and references to those functions much easier elsewhere. 
- Save often to make sure that black and ruff don't get confused. 

If creating new features, new files that get created that are large/have too many of them should  be added to the `.gitignore`. 

### Notes

- [OnShape API explorer](https://cad.onshape.com/glassworks/explorer/#/Assembly/getFeatures)
- [OnShape API documentation](https://onshape-public.github.io/docs/api-intro/)
- [OnShape-to-Robot Library](https://github.com/Rhoban/onshape-to-robot)
