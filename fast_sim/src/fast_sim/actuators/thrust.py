"""Simplified thrust actuator models."""

from fast_sim.actuators.base import ThrustActuator
from fast_sim.actuators.parameters import (
    ThrustActuatorParameters,
)


class LimitedThrustActuator(ThrustActuator):
    """Stateless thrust actuator with minimum and maximum thrust."""

    def __init__(
        self,
        parameters: ThrustActuatorParameters,
    ) -> None:
        self._parameters = parameters

    @property
    def parameters(self) -> ThrustActuatorParameters:
        """Return the actuator parameters."""
        return self._parameters

    def apply(
        self,
        command: float,
    ) -> float:
        """Clamp requested thrust to the configured limits."""
        return min(
            max(command, self._parameters.min_thrust),
            self._parameters.max_thrust,
        )