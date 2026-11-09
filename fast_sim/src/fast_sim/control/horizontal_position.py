"""Horizontal position controllers for the 2D hopper."""

from math import asin, isfinite, pi, sin

from fast_sim.control.base import (
    HorizontalPositionController2D,
)
from fast_sim.control.parameters import (
    PDHorizontalPositionControllerParameters,
)
from fast_sim.control.reference import (
    AttitudeReference2D,
    HorizontalPositionReference2D,
)
from fast_sim.models import HopperState2D


class PDHorizontalPositionController2D(
    HorizontalPositionController2D
):
    """Outer-loop PD controller for horizontal position.

    The controller computes a desired horizontal acceleration and maps
    it to an attitude reference using the upright-hover relation

        a_x = g * sin(theta_ref).

    The requested acceleration is limited consistently with the
    configured maximum attitude reference before the inverse mapping
    is applied.
    """

    def __init__(
        self,
        parameters: PDHorizontalPositionControllerParameters,
        *,
        gravity: float,
        max_attitude_angle: float,
    ) -> None:
        if not isfinite(gravity):
            raise ValueError(
                "gravity must be finite"
            )

        if gravity <= 0.0:
            raise ValueError(
                "gravity must be > 0"
            )

        if not isfinite(max_attitude_angle):
            raise ValueError(
                "max_attitude_angle must be finite"
            )

        if not 0.0 < max_attitude_angle < pi / 2.0:
            raise ValueError(
                "max_attitude_angle must be in (0, pi/2)"
            )

        self._parameters = parameters
        self._gravity = gravity
        self._max_attitude_angle = (
            max_attitude_angle
        )

    @property
    def parameters(
        self,
    ) -> PDHorizontalPositionControllerParameters:
        """Return the controller gains."""
        return self._parameters

    @property
    def gravity(self) -> float:
        """Return the gravity used by the acceleration mapping."""
        return self._gravity

    @property
    def max_attitude_angle(self) -> float:
        """Return the maximum absolute attitude reference in radians."""
        return self._max_attitude_angle

    @property
    def max_horizontal_acceleration(self) -> float:
        """Return the maximum requested horizontal acceleration."""
        return (
            self._gravity
            * sin(self._max_attitude_angle)
        )

    def compute_attitude_reference(
        self,
        state: HopperState2D,
        reference: HorizontalPositionReference2D,
    ) -> AttitudeReference2D:
        """Return the attitude reference for the inner attitude loop."""
        position_error = (
            reference.px - state.px
        )

        velocity_error = (
            reference.vx - state.vx
        )

        raw_acceleration_command = (
            self._parameters.proportional_gain
            * position_error
            + self._parameters.derivative_gain
            * velocity_error
        )

        max_acceleration = (
            self.max_horizontal_acceleration
        )

        limited_acceleration_command = min(
            max(
                raw_acceleration_command,
                -max_acceleration,
            ),
            max_acceleration,
        )

        theta_reference = asin(
            limited_acceleration_command
            / self._gravity
        )

        return AttitudeReference2D(
            theta=theta_reference,
            omega=0.0,
        )
