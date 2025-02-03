"""Defines helper functions for MJCF accessors."""

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from urdf2mjcf.convert import (
        ConversionMetadata as ConversionMetadataRef,
    )


@dataclass
class JointParam:
    kp: float = field(default=0.0)
    kd: float = field(default=0.0)


@dataclass
class JointParamsMetadata:
    suffix_to_pd_params: dict[str, JointParam] = field(default_factory=lambda: {})
    default: JointParam | None = field(default=None)


@dataclass
class ImuSensor:
    link_name: str = field(default="")
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quat: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    acc_noise: float | None = field(default=None)
    gyro_noise: float | None = field(default=None)
    mag_noise: float | None = field(default=None)


@dataclass
class ConversionMetadata:
    joint_params: JointParamsMetadata | None = field(default=None)
    imus: list[ImuSensor] = field(default_factory=lambda: [])


def convert_to_mjcf_metadata(metadata: ConversionMetadata) -> "ConversionMetadataRef":
    try:
        from urdf2mjcf.convert import (
            ConversionMetadata as ConversionMetadataRef,
            ImuSensor as ImuSensorRef,
            JointParam as JointParamRef,
            JointParamsMetadata as JointParamsMetadataRef,
        )

    except ImportError as e:
        raise ImportError(
            "Please install the package with `urdf2mjcf` as a dependency, using "
            "`pip install kscale-onshape-library[mujoco]`"
        ) from e

    return ConversionMetadataRef(
        joint_params=JointParamsMetadataRef(
            suffix_to_pd_params=(
                {}
                if metadata.joint_params is None
                else {
                    name: JointParamRef(
                        kp=param.kp,
                        kd=param.kd,
                    )
                    for name, param in metadata.joint_params.suffix_to_pd_params.items()
                }
            ),
            default=(
                None
                if metadata.joint_params is None or metadata.joint_params.default is None
                else JointParamRef(
                    kp=metadata.joint_params.default.kp,
                    kd=metadata.joint_params.default.kd,
                )
            ),
        ),
        imus=[
            ImuSensorRef(
                link_name=imu.link_name,
                pos=imu.pos,
                quat=imu.quat,
                acc_noise=imu.acc_noise,
                gyro_noise=imu.gyro_noise,
                mag_noise=imu.mag_noise,
            )
            for imu in metadata.imus
        ],
    )
