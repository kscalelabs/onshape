"""Defines helper functions for MJCF accessors."""

import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

logger = logging.getLogger(__name__)

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
    pos: list[float] | None = field(default=None)  # (x, y, z)
    rpy: list[float] | None = field(default=None)  # (roll, pitch, yaw)
    acc_noise: float | None = field(default=None)
    gyro_noise: float | None = field(default=None)
    mag_noise: float | None = field(default=None)


@dataclass
class ConversionMetadata:
    suffix: str | None = field(default=None)
    freejoint: bool = field(default=True)
    joint_params: list[JointParam] = field(default_factory=lambda: [])
    imus: list[ImuSensor] = field(default_factory=lambda: [])
    flat_feet_links: list[str] = field(default_factory=lambda: [])
    explicit_floor_contacts: list[str] = field(default_factory=lambda: [])
    floating_base: bool = field(default=True)
    maxhullvert: int = field(default=64)


def convert_to_mjcf_metadata(metadata: ConversionMetadata) -> "ConversionMetadataRef":
    try:
        import urdf2mjcf

        logger.debug("urdf2mjcf version: %s", urdf2mjcf.__version__)

    except ImportError as e:
        raise ImportError(
            "urdf2mjcf is required to run this script. Install it with `pip install "
            "'onshape[mujoco]'` to install the required dependencies."
        ) from e

    from urdf2mjcf.model import (
        ConversionMetadata as ConversionMetadataRef,
        ImuSensor as ImuSensorRef,
        JointParam as JointParamRef,
    )

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
                rpy=imu.rpy,
                acc_noise=imu.acc_noise,
                gyro_noise=imu.gyro_noise,
                mag_noise=imu.mag_noise,
            )
            for imu in metadata.imus
        ],
        flat_feet_links=metadata.flat_feet_links,
        explicit_floor_contacts=metadata.explicit_floor_contacts,
        floating_base=metadata.floating_base,
        maxhullvert=metadata.maxhullvert,
        freejoint=metadata.freejoint,
    )
