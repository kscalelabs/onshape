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

Or, if you want to install all dependencies, do:

```bash
pip install 'kscale-onshape-library[all]'
```

In order to access the OnShape API, you need to define `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY` using a key generated [here](https://dev-portal.onshape.com/keys).

### Usage

To convert an assembly to a URDF, use the following command:

```bash
kol urdf https://cad.onshape.com/documents/DDDDDDDD/w/WWWWWWWW/e/EEEEEEEE
```

To convert an assembly to a MJCF, use the following command:

```bash
kol mjcf https://cad.onshape.com/documents/DDDDDDDD/w/WWWWWWWW/e/EEEEEEEE
```

You can visualize the resulting artifacts using MuJoCo Viewer:

```bash
python -m MuJoCo.viewer
```

For URDF files it's sometimes necessary to first drag the file into the meshes folder before viewing it. 

You can also visualize the resulting URDF using PyBullet:

```bash
pip install pybullet
kol pybullet robot/<urdf-name>.urdf
```

### Advanced Tools

For improving simulation speed and reducing internal collisions, you can choose to merge a URDF at each of
its fixed joints.

```bash
kol merge-fixed-joints robot/URDF.urdf
```

To simplify each of the meshes in a URDF using vertex clustering:

```bash
kol simplify-all robot/URDF.urdf
```

The size of the voxels is quite important when using this function, and can be passed in as a flag. Essentially,
the way this program works is by grabbing groups of vertices which fit into a single voxel and then combining
them into a single vertex. This funciton is especially useful because MuJoCo cannot render bodies with more 
than 20 coplanar faces. When choosing a voxel size, typically between `0.0001` and `0.0005` is suitable. Avoid
setting a very large voxel size, as although the visual and collision meshes themselves will not be too unaffected,
smaller parts can become 2d. 

To cleanup the meshes directory by removing all meshes not referenced in a urdf:

```bash
kol cleanup-mesh-dir robot/URDF.urdf
```

For example, if you wanted to download the 5-DOF stompy arm, merge and simplify, then clean the directory.

```bash
kol urdf https://cad.onshape.com/documents/afaee604f6ca311526a6aec8/w/29af84cb974c2d825b71de39/e/4fef6bce7179a665e62b03ba
&& kol merge-fixed-joints robot/assembly_1.urdf
&& kol simplify-all robot/assembly_1_merged.urdf
&& kol cleanup-mesh-dir robot/assembly_1_merged_simplified.urdf
```
## Useful Flags

For quicker development, we've additionally added these as flags in the base converter. These, and some other 
flags will be very helpful. 
- `--merge-joints`: Merge all fixed joints
- `--simplify-meshes`: Simplify meshes. In this case, the voxel size is passed in using the `--voxel_size` flag. 
- `--skip-small-parts`: Skip parts defined in the `small_part` list. This is necessary for simulation speed, and
because very small parts cause issues with inertia.
- `--remove-inertia`: Ignore all inertial objects in the assembly.
- `--sim_ignore_key`: Set a key in onshape link names to be skipped. Defaults to `sim_ignore`. 
- Cleaning up meshes is automatically done whenever using merge or simplify. 

For example, one might run

```
kol urdf https://cad.onshape.com/documents/afaee604f6ca311526a6aec8/w/29af84cb974c2d825b71de39/e/4fef6bce7179a665e62b03ba --simplify-meshes --merge-joints --skip-small-parts
```

to get a very simulation ready artifact. 

## Overwriting

Multiple methods of overwriting various aspects of simulation artifacts exist as flags as well. By specifying
a `config.json` file similar to the one in config_example.json with dictionaries, the following flags can be used.
- `override_joint_names`: Override joint names for joints. When specifying this dictionary, it's reccomended to 
put the robot into MuJoCo viewer, and compare names of joints to those in onshape. Naming joints descriptively
things like `left hip pitch` will make adapting to simulation far easier.
- `override_nonfixed`: Set nonfixed joints (`prismatic`, `revolute`, etc.) to be of the `fixed` type.
- `override_limits`: Override the limits on specific joints. 
- `override_torques`: Set torque limits for specific joints.

You can also set default values for all joints using the following flags.
- `default_color`: Set all links to a default RGBA color.
- `default_mass`: Set all masses to a certain default value if they don't exist. It's very reccomended to address
these in onshape however.
- `suffix_to_joint_effort`: Override effort limits for joints. 
- `suffix_to_joint_velocity`: Override velocity limits for joints. Very important for simulation stability. 

### Simulation

The output of the onshape library is simply a robot floating in space. Luckily, most simulators which support
URDFs are able to define an environment within code. More changes are needed to make MJCF files simulation ready.
We have support for adapting MJCF files to isaac sim here: https://github.com/kscalelabs/sim/blob/master/sim/scripts/create_mjcf.py.

It will be useful to create more automatic scripts for adapting raw URDF and MJCF to various simulators. Feel free
to contribute! Support for other file formats like USD files for IsaacLab will be helpful as well. 

### Tips and Suggestions

- To avoid mishaps from occuring due to cached files, if making big changes in a single export, or if the robot
has undergone major changes since the last export, it's good to start fresh with a new `/robot/` folder. 
- There's no guarantee that the robot's starting pose (when all joint positions are set to 0) will be good,
or even matching what you see on onshape. It's reccomended to use MuJoCo Viewer to find a good starting
position for each joint.
- Robots function much better in simulation when they have less parts. It's very good to make sure that link
names are descriptive in onshape so small parts can be better removed using `--skip-small-parts`. This will also make export faster.
- When using functions that can be destructive like `simplify-meshes`, `merge-joints`, and especially `cleanup-mesh-dir`, it's good to make copies of the STL files elsewhere first.

### Development

Since this is a library, it's necessary to upversion after big changes. You can do this by changing the
version number in `kol/__init__.py`, then running the Github Action to publish a new version. 

Please keep this repo well linted! Specifically, it is quite easy to run into errors with mypy since many
of the functions and variables have ambiguous types. To check which files have issues locally, run
`mypy kol` and `make static-checks`. Many mypy issues can be alleviated with one of the following:

- Ensure that type annotations properly include `None` when necessary. This is especially important for
functions.
- Make good use of classes to ensure everything is well typed. See `kol/onshape/schema/assembly.py`. 
- When using functions such as `dict.find`, use a `None` check to guarantee that variable accesses are safe
- Make sure docstrings are descriptive and that there's one for every function, even if it seems trivial.
This makes mouseovers and references to those functions much easier elsewhere. 
- Save often to make sure that black and ruff don't get confused. 

If creating new features, new files that get created that are large/have too many of them should 
be added to the `.gitignore`. 

### Notes

- [OnShape API explorer](https://cad.onshape.com/glassworks/explorer/#/Assembly/getFeatures)
- [OnShape API documentation](https://onshape-public.github.io/docs/api-intro/)
- [OnShape-to-Robot Library](https://github.com/Rhoban/onshape-to-robot)
