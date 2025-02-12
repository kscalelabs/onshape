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
    site_name: str = field()
    pos: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # (x, y, z)
    quat: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])  # (w, x, y, z)
    acc_noise: float | None = field(default=None)
    gyro_noise: float | None = field(default=None)
    mag_noise: float | None = field(default=None)


@dataclass
class ConversionMetadata:
    joint_params: JointParamsMetadata | None = field(default=None)
    imus: list[ImuSensor] = field(default_factory=lambda: [])
    remove_fixed_joints: bool = field(default=False)
    floating_base: bool = field(default=True)
    add_mjcf_scene: bool = field(default=True)


def convert_to_mjcf_metadata(metadata: ConversionMetadata) -> "ConversionMetadataRef":
    try:
        from urdf2mjcf.model import (
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
                site_name=imu.site_name,
                pos=imu.pos,
                quat=imu.quat,
                acc_noise=imu.acc_noise,
                gyro_noise=imu.gyro_noise,
                mag_noise=imu.mag_noise,
            )
            for imu in metadata.imus
        ],
        remove_fixed_joints=metadata.remove_fixed_joints,
        floating_base=metadata.floating_base,
    )
