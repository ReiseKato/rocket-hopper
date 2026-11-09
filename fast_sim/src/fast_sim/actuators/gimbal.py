"""Simplified gimbal actuator models."""

from math import isfinite

from fast_sim.actuators.base import GimbalActuator
from fast_sim.actuators.parameters import (
    GimbalActuatorParameters,
)


class RateLimitedGimbalActuator(GimbalActuator):
    """Gimbal actuator with angle and rate limits."""

    def __init__(
        self,
        parameters: GimbalActuatorParameters,
        initial_angle: float = 0.0,
    ) -> None:
        if not isfinite(initial_angle):
            raise ValueError("initial_angle must be finite")

        if abs(initial_angle) > parameters.max_angle:
            raise ValueError(
                "initial_angle must be within the configured "
                "gimbal angle limits"
            )

        self._parameters = parameters
        self._angle = initial_angle

    @property
    def parameters(self) -> GimbalActuatorParameters:
        """Return the actuator parameters."""
        return self._parameters

    @property
    def angle(self) -> float:
        """Return the current physical gimbal angle."""
        return self._angle

    def step(
        self,
        command: float,
        dt: float,
    ) -> float:
        """Advance the discrete gimbal model by one step."""
        if not isfinite(command):
            raise ValueError("command must be finite")

        if not isfinite(dt):
            raise ValueError("dt must be finite")

        if dt <= 0.0:
            raise ValueError("dt must be > 0")

        target = min(
            max(
                command,
                -self._parameters.max_angle,
            ),
            self._parameters.max_angle,
        )

        max_delta = self._parameters.max_rate * dt

        error = target - self._angle

        angle_delta = min(
            max(error, -max_delta),
            max_delta,
        )

        self._angle += angle_delta

        return self._angle