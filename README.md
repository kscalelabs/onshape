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

Welcome to the K-Scale OnShape Library! For more information, see the [documentation](https://docs.kscale.dev/utils/onshape).

# Quick Start

```
kol run -o "output_dir" -f "config_path" onshape_url
```

## Example Config
```
default_revolute_joint_effort: 17
default_revolute_joint_velocity: 360
suffix_to_joint_effort:
  "_00": 14
  "_02": 17
  "_03": 60
  "_04": 120
suffix_to_joint_velocity:
  "_00": 12.566
  "_02": 12.566
  "_03": 6.283
  "_04": 6.283
convex_collision_meshes: true
ignore_merging_fixed_joints:
  - "imu_link"
max_mesh_triangles: 10000
max_convex_collision_mesh_triangles: 100

exclude_collision_meshes:
  - "KB-C-101R_R_Shoulder_Drive"
  - "KB-C-101L_ShldYokeROLL"
  - "KB-D-102L_L_Hip_Yoke_Drive"
  - "KB-D-102R_R_Hip_Yoke_Drive"

shrink_collision_meshes:
  # Leg roll
  "RS03_4" : 0.8
  "RS03_3" : 0.8
  # Knee
  "KB-D-301R_R_Femur_Lower_Drive": 0.5
  "KB-D-301L_L_Femur_Lower_Drive": 0.5
  # Ankle
  "KB-D-401R_R_Shin_Drive": 0.85
  "KB-D-401L_L_Shin_Drive": 0.85
  # Shoulder roll
  "RS03_5": 0.8
  "RS03_6": 0.8
  # Elbow
  "R_Bicep_Lower_Drive": 0.4
  "L_Bicep_Lower_Drive": 0.4
  # Forearm
  "R_Forearm_Upper_Structural": 0.8
  "L_Forearm_Upper_Drive": 0.8
  # IMU
  "imu": 0.0
  # Base
  "KB-B-102B_TORSO_BOTTOM": 0.8

move_collision_meshes:
  "R_Bicep_Lower_Drive": [0.0, 0.0, -0.01]
  "L_Bicep_Lower_Drive": [0.0, 0.0, -0.01]

joint_separation_distance: 0.001

base_quaternion:
  - 0.0
  - 0.0
  - 1.0
  - 0.0

flip_joints:

mjcf_metadata:
  joint_params:
    default:
      kp: 20.0
      kd: 2.0
    suffix_to_pd_params:
      "_00":
        kp: 20.0
        kd: 2.0
      "_shoulder_yaw_02":
        kp: 50.0
        kd: 5.0
      "_elbow_02":
        kp: 50.0
        kd: 5.0
      "_wrist_02":
        kp: 20.0
        kd: 2.0
      "_ankle_02":
        kp: 80.0
        kd: 10.0
      "_03":
        kp: 150.0
        kd: 8.0
      "_knee_04":
        kp: 200.0
        kd: 8.0
      "_hip_pitch_04":
        kp: 250.0
        kd: 30.0
  imus:
    - site_name: "imu"
      pos: [0.0, 0.0, 0.0]
      quat: [1.0, 0.0, 0.0, 0.0]
      acc_noise: 0.01
      gyro_noise: 0.01
      mag_noise: 0.05
  remove_fixed_joints: true

# This is for debugging purposes.
# use_collision_meshes_as_visual_meshes: true
```
