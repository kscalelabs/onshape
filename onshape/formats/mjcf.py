"""Defines helper functions for MJCF accessors."""

import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, cast, get_args

logger = logging.getLogger(__name__)

if TYPE_CHECKING:
    from urdf2mjcf.convert import ConversionMetadata as ConversionMetadataRef


@dataclass
class CollisionParams:
    condim: int = field(default=6)
    contype: int = field(default=0)
    conaffinity: int = field(default=1)
    solref: list[float] = field(default_factory=lambda: [0.005, 1.0])
    solimp: list[float] = field(default_factory=lambda: [0.99, 0.999, 0.00001])
    friction: list[float] = field(default_factory=lambda: [0.8, 0.02, 0.01])


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
class CollisionGeometry:
    name: str = field()
    collision_type: str = field()
    sphere_radius: float = field(default=0.01)
    axis_order: tuple[int, int, int] = field(default=(0, 1, 2))
    flip_axis: bool = field(default=False)


@dataclass
class ConversionMetadata:
    suffix: str | None = field(default=None)
    freejoint: bool = field(default=True)
    collision_params: CollisionParams = field(default_factory=lambda: CollisionParams())
    joint_params: list[JointParam] = field(default_factory=lambda: [])
    imus: list[ImuSensor] = field(default_factory=lambda: [])
    force_sensors: list[ForceSensor] = field(default_factory=lambda: [])
    collision_geometries: list[CollisionGeometry] = field(default_factory=lambda: [])
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
        CollisionGeometry as CollisionGeometryRef,
        CollisionParams as CollisionParamsRef,
        CollisionType,
        ConversionMetadata as ConversionMetadataRef,
        ExplicitFloorContacts as ExplicitFloorContactsRef,
        ForceSensor as ForceSensorRef,
        ImuSensor as ImuSensorRef,
        JointParam as JointParamRef,
    )

    if metadata.angle not in get_args(Angle):
        raise ValueError(f"Invalid angle type: {metadata.angle}. Must be one of {get_args(Angle)}")

    for cg in metadata.collision_geometries:
        if not hasattr(CollisionType, cg.collision_type.upper()):
            raise ValueError(f"Bad collision type: {cg.collision_type}. Must be in {CollisionType.__members__.keys()}")

    return ConversionMetadataRef(
        freejoint=metadata.freejoint,
        collision_params=CollisionParamsRef(
            condim=metadata.collision_params.condim,
            contype=metadata.collision_params.contype,
            solref=metadata.collision_params.solref,
            friction=metadata.collision_params.friction,
        ),
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
        collision_geometries=[
            CollisionGeometryRef(
                name=cg.name,
                collision_type=getattr(CollisionType, cg.collision_type.upper()),
                sphere_radius=cg.sphere_radius,
                axis_order=cg.axis_order,
                flip_axis=cg.flip_axis,
            )
            for cg in metadata.collision_geometries
        ],
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
