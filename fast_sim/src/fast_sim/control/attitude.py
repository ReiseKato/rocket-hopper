"""Attitude controllers for the 2D hopper."""

from fast_sim.control.angles import wrap_angle_pi
from fast_sim.control.base import AttitudeController2D
from fast_sim.control.parameters import (
    PDAttitudeControllerParameters,
)
from fast_sim.control.reference import AttitudeReference2D
from fast_sim.models import HopperState2D


class PDAttitudeController2D(AttitudeController2D):
    """Stateless PD controller for 2D hopper attitude."""

    def __init__(
        self,
        parameters: PDAttitudeControllerParameters,
    ) -> None:
        self._parameters = parameters

    @property
    def parameters(
        self,
    ) -> PDAttitudeControllerParameters:
        """Return the controller parameters."""
        return self._parameters

    def compute_gimbal_command(
        self,
        state: HopperState2D,
        reference: AttitudeReference2D,
    ) -> float:
        """Return the requested gimbal angle in radians.

        The sign follows the hopper convention

            positive gimbal angle -> negative angular acceleration.

        Therefore a positive attitude or angular velocity error requires
        a positive gimbal command to create negative feedback.
        """
        attitude_error = wrap_angle_pi(
            state.theta - reference.theta
        )

        angular_velocity_error = (
            state.omega - reference.omega
        )

        return (
            self._parameters.proportional_gain
            * attitude_error
            + self._parameters.derivative_gain
            * angular_velocity_error
        )
