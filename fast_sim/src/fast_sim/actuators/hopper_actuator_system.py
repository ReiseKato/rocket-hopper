"""Composition of 2D hopper actuator models."""

from fast_sim.actuators.base import (
    GimbalActuator,
    ThrustActuator,
)
from fast_sim.models import (
    HopperActuation2D,
    HopperActuationCommand2D,
)


class HopperActuatorSystem2D:
    """Compose the thrust and gimbal actuator models."""

    def __init__(
        self,
        thrust_actuator: ThrustActuator,
        gimbal_actuator: GimbalActuator,
    ) -> None:
        self._thrust_actuator = thrust_actuator
        self._gimbal_actuator = gimbal_actuator

    def step(
        self,
        command: HopperActuationCommand2D,
        dt: float,
    ) -> HopperActuation2D:
        """Convert a command into physical actuation for one system step."""
        thrust = self._thrust_actuator.apply(
            command.thrust
        )

        gimbal_angle = self._gimbal_actuator.step(
            command=command.gimbal_angle,
            dt=dt,
        )

        return HopperActuation2D(
            thrust=thrust,
            gimbal_angle=gimbal_angle,
        )