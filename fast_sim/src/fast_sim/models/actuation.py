"""Physical actuation data structures."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True, slots=True)
class HopperActuation2D:
    """Physical actuation applied to the 2D rigid body.

    These are actual physical values, not controller commands.

    Attributes:
        thrust: Thrust-force magnitude [N].
        gimbal_angle: Actual thrust-vector angle relative to +z_B [rad].
            Positive angles tilt the thrust vector toward +x_B.
    """

    thrust: float
    gimbal_angle: float

    def __post_init__(self) -> None:
        if not isfinite(self.thrust):
            raise ValueError(f"thrust must be finite")
        if not isfinite(self.gimbal_angle):
            raise ValueError(f"gimbal_angle must be finite")
        if self.thrust < 0.0:
            raise ValueError("thrust must be >= 0")


@dataclass(frozen=True, slots=True)
class HopperActuationCommand2D:
    """Requested thrust and gimbal angle for the 2D hopper.

    Commands describe desired physical values before actuator models
    apply their limitations.

    Units:
        thrust: N
        gimbal_angle: rad
    """

    thrust: float
    gimbal_angle: float

    def __post_init__(self) -> None:
        if not isfinite(self.thrust):
            raise ValueError("thrust must be finite")

        if not isfinite(self.gimbal_angle):
            raise ValueError("gimbal_angle must be finite")

        if self.thrust < 0.0:
            raise ValueError("thrust must be >= 0")