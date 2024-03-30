# K-Scale OnShape Library

This library is what we use at K-Scale for interacting with OnShape. It is a wrapper around the OnShape API that allows us to easily import parts from OnShape into our projects.

### Getting Started

Install the library using pip:

```bash
pip install kscale-onshape-library
```

Or, if you're developing the library, do:

```bash
git clone git@github.com:kscalelabs/onshape.git
cd onshape
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

### Notes

- [OnShape API explorer](https://cad.onshape.com/glassworks/explorer/#/Assembly/getFeatures)
- [OnShape API documentation](https://onshape-public.github.io/docs/api-intro/)
- [OnShape-to-Robot Library](https://github.com/Rhoban/onshape-to-robot)
