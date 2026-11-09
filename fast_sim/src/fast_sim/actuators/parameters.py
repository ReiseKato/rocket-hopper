"""Parameters for actuator models."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True, slots=True)
class ThrustActuatorParameters:
    """Limits of the simplified thrust actuator."""

    min_thrust: float
    max_thrust: float

    def __post_init__(self) -> None:
        if not isfinite(self.min_thrust):
            raise ValueError("min_thrust must be finite")

        if not isfinite(self.max_thrust):
            raise ValueError("max_thrust must be finite")

        if self.min_thrust < 0.0:
            raise ValueError("min_thrust must be >= 0")

        if self.max_thrust < self.min_thrust:
            raise ValueError(
                "max_thrust must be >= min_thrust"
            )


@dataclass(frozen=True, slots=True)
class GimbalActuatorParameters:
    """Limits of the simplified gimbal actuator."""

    max_angle: float
    max_rate: float

    def __post_init__(self) -> None:
        if not isfinite(self.max_angle):
            raise ValueError("max_angle must be finite")

        if not isfinite(self.max_rate):
            raise ValueError("max_rate must be finite")

        if self.max_angle < 0.0:
            raise ValueError("max_angle must be >= 0")

        if self.max_rate <= 0.0:
            raise ValueError("max_rate must be > 0")