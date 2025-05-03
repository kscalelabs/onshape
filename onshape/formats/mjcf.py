"""Defines helper functions for MJCF accessors."""

import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, cast, get_args

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
class ForceSensor:
    body_name: str = field()
    site_name: str = field()
    name: str | None = field(default=None)
    noise: float | None = field(default=None)


@dataclass
class ExplicitFloorContacts:
    contact_links: list[str] = field(default_factory=lambda: [])
    class_name: str = field(default="collision")


@dataclass
class ConversionMetadata:
    suffix: str | None = field(default=None)
    freejoint: bool = field(default=True)
    joint_params: list[JointParam] = field(default_factory=lambda: [])
    imus: list[ImuSensor] = field(default_factory=lambda: [])
    force_sensors: list[ForceSensor] = field(default_factory=lambda: [])
    feet_links: list[str] = field(default_factory=lambda: [])
    explicit_contacts: ExplicitFloorContacts | None = field(default_factory=ExplicitFloorContacts)
    remove_redundancies: bool = field(default=True)
    floating_base: bool = field(default=True)
    maxhullvert: int = field(default=64)
    angle: str = field(default="radian")
    floor_name: str = field(default="floor")
    add_floor: bool = field(default=True)
    backlash: float | None = field(default=None)
    backlash_damping: float = field(default=0.01)


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
        Angle,
        ConversionMetadata as ConversionMetadataRef,
        ExplicitFloorContacts as ExplicitFloorContactsRef,
        ForceSensor as ForceSensorRef,
        ImuSensor as ImuSensorRef,
        JointParam as JointParamRef,
    )

    if metadata.angle not in get_args(Angle):
        raise ValueError(f"Invalid angle type: {metadata.angle}. Must be one of {get_args(Angle)}")

    return ConversionMetadataRef(
        freejoint=metadata.freejoint,
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
        force_sensors=[
            ForceSensorRef(
                body_name=fs.body_name,
                site_name=fs.site_name,
                name=fs.name,
                noise=fs.noise,
            )
            for fs in metadata.force_sensors
        ],
        feet_links=metadata.feet_links,
        explicit_contacts=(
            None
            if metadata.explicit_contacts is None
            else ExplicitFloorContactsRef(
                contact_links=metadata.explicit_contacts.contact_links,
                class_name=metadata.explicit_contacts.class_name,
            )
        ),
        remove_redundancies=metadata.remove_redundancies,
        floating_base=metadata.floating_base,
        maxhullvert=metadata.maxhullvert,
        angle=cast(Angle, metadata.angle),
        floor_name=metadata.floor_name,
        add_floor=metadata.add_floor,
        backlash=metadata.backlash,
        backlash_damping=metadata.backlash_damping,
    )
