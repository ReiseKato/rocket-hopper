"""Altitude controllers for the 2D hopper."""

from math import isfinite

from fast_sim.control.base import AltitudeController2D
from fast_sim.control.parameters import (
    PDAltitudeControllerParameters,
)
from fast_sim.control.reference import AltitudeReference2D
from fast_sim.models import HopperState2D


class PDAltitudeController2D(AltitudeController2D):
    """Stateless PD controller for vertical position and velocity."""

    def __init__(
        self,
        parameters: PDAltitudeControllerParameters,
        hover_thrust: float,
    ) -> None:
        if not isfinite(hover_thrust):
            raise ValueError(
                "hover_thrust must be finite"
            )

        if hover_thrust < 0.0:
            raise ValueError(
                "hover_thrust must be >= 0"
            )

        self._parameters = parameters
        self._hover_thrust = hover_thrust

    @property
    def parameters(
        self,
    ) -> PDAltitudeControllerParameters:
        """Return the controller gains."""
        return self._parameters

    @property
    def hover_thrust(self) -> float:
        """Return the feedforward hover thrust in newtons."""
        return self._hover_thrust

    def compute_thrust_command(
        self,
        state: HopperState2D,
        reference: AltitudeReference2D,
    ) -> float:
        """Return the requested thrust in newtons.

        The controller uses

            T_cmd = T_hover + Kp * e_z + Kd * e_vz

        with

            e_z  = pz_ref - pz
            e_vz = vz_ref - vz.

        No actuator-specific upper thrust limit is applied here.
        The lower bound at zero only preserves the valid command domain,
        because HopperActuationCommand2D does not allow negative thrust.
        """
        altitude_error = (
            reference.pz - state.pz
        )

        vertical_velocity_error = (
            reference.vz - state.vz
        )

        raw_thrust_command = (
            self._hover_thrust
            + self._parameters.proportional_gain
            * altitude_error
            + self._parameters.derivative_gain
            * vertical_velocity_error
        )

        return max(
            0.0,
            raw_thrust_command,
        )
