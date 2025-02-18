"""Defines helper functions for MJCF accessors."""

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from urdf2mjcf.convert import ConversionMetadata as ConversionMetadataRef


@dataclass
class JointParam:
    name: str = field()
    suffixes: list[str] = field(default_factory=lambda: [])
    armature: float | None = field(default=None)
    frictionloss: float | None = field(default=None)
    actuatorfrc: float | None = field(default=None)


@dataclass
class ImuSensor:
    body_name: str = field()
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # (x, y, z)
    quat: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])  # (w, x, y, z)
    acc_noise: float | None = field(default=None)
    gyro_noise: float | None = field(default=None)
    mag_noise: float | None = field(default=None)


@dataclass
class ConversionMetadata:
    joint_params: list[JointParam] = field(default_factory=lambda: [])
    imus: list[ImuSensor] = field(default_factory=lambda: [])
    floating_base: bool = field(default=True)


def convert_to_mjcf_metadata(metadata: ConversionMetadata) -> "ConversionMetadataRef":
    try:
        from urdf2mjcf.model import (
            ConversionMetadata as ConversionMetadataRef,
            ImuSensor as ImuSensorRef,
            JointParam as JointParamRef,
        )

    except ImportError as e:
        raise ImportError(
            "Please install the package with `urdf2mjcf` as a dependency, using "
            "`pip install kscale-onshape-library[mujoco]`"
        ) from e

    return ConversionMetadataRef(
        joint_params=[
            JointParamRef(
                name=param.name,
                suffixes=param.suffixes,
                armature=param.armature,
                frictionloss=param.frictionloss,
                actuatorfrc=param.actuatorfrc,
            )
            for param in metadata.joint_params
        ],
        imus=[
            ImuSensorRef(
                body_name=imu.body_name,
                pos=imu.pos,
                quat=imu.quat,
                acc_noise=imu.acc_noise,
                gyro_noise=imu.gyro_noise,
                mag_noise=imu.mag_noise,
            )
            for imu in metadata.imus
        ],
        floating_base=metadata.floating_base,
    )
