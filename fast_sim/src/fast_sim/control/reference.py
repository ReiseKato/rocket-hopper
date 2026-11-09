"""Reference values used by controllers."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True, slots=True)
class AttitudeReference2D:
    """Desired 2D attitude and angular velocity.

    Units:
        theta: rad
        omega: rad/s
    """

    theta: float
    omega: float = 0.0

    def __post_init__(self) -> None:
        if not isfinite(self.theta):
            raise ValueError("theta must be finite")

        if not isfinite(self.omega):
            raise ValueError("omega must be finite")


@dataclass(frozen=True, slots=True)
class AltitudeReference2D:
    """Desired vertical position and velocity.

    Units:
        pz: m
        vz: m/s
    """

    pz: float
    vz: float = 0.0

    def __post_init__(self) -> None:
        if not isfinite(self.pz):
            raise ValueError("pz must be finite")

        if not isfinite(self.vz):
            raise ValueError("vz must be finite")


@dataclass(frozen=True, slots=True)
class HorizontalPositionReference2D:
    """Desired horizontal position and velocity.

    Units:
        px: m
        vx: m/s
    """

    px: float
    vx: float = 0.0

    def __post_init__(self) -> None:
        if not isfinite(self.px):
            raise ValueError("px must be finite")

        if not isfinite(self.vx):
            raise ValueError("vx must be finite")
