"""Parameters for controllers."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True, slots=True)
class PDAttitudeControllerParameters:
    """Gains of the 2D PD attitude controller."""

    proportional_gain: float
    derivative_gain: float

    def __post_init__(self) -> None:
        if not isfinite(self.proportional_gain):
            raise ValueError(
                "proportional_gain must be finite"
            )

        if not isfinite(self.derivative_gain):
            raise ValueError(
                "derivative_gain must be finite"
            )

        if self.proportional_gain < 0.0:
            raise ValueError(
                "proportional_gain must be >= 0"
            )

        if self.derivative_gain < 0.0:
            raise ValueError(
                "derivative_gain must be >= 0"
            )


@dataclass(frozen=True, slots=True)
class PDAltitudeControllerParameters:
    """Gains of the 2D PD altitude controller.

    Units:
        proportional_gain: N/m
        derivative_gain: N/(m/s)
    """

    proportional_gain: float
    derivative_gain: float

    def __post_init__(self) -> None:
        if not isfinite(self.proportional_gain):
            raise ValueError(
                "proportional_gain must be finite"
            )

        if not isfinite(self.derivative_gain):
            raise ValueError(
                "derivative_gain must be finite"
            )

        if self.proportional_gain < 0.0:
            raise ValueError(
                "proportional_gain must be >= 0"
            )

        if self.derivative_gain < 0.0:
            raise ValueError(
                "derivative_gain must be >= 0"
            )


@dataclass(frozen=True, slots=True)
class PDHorizontalPositionControllerParameters:
    """Gains of the horizontal position PD controller.

    The controller first produces a horizontal acceleration command.

    Units:
        proportional_gain: 1/s^2
        derivative_gain: 1/s
    """

    proportional_gain: float
    derivative_gain: float

    def __post_init__(self) -> None:
        if not isfinite(self.proportional_gain):
            raise ValueError(
                "proportional_gain must be finite"
            )

        if not isfinite(self.derivative_gain):
            raise ValueError(
                "derivative_gain must be finite"
            )

        if self.proportional_gain < 0.0:
            raise ValueError(
                "proportional_gain must be >= 0"
            )

        if self.derivative_gain < 0.0:
            raise ValueError(
                "derivative_gain must be >= 0"
            )
