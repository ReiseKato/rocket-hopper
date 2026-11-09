"""Common actuator interfaces."""

from abc import ABC, abstractmethod


class ThrustActuator(ABC):
    """Interface for thrust actuator models."""

    @abstractmethod
    def apply(
        self,
        command: float,
    ) -> float:
        """Return physical thrust for a requested thrust command."""


class GimbalActuator(ABC):
    """Interface for stateful gimbal actuator models."""

    @property
    @abstractmethod
    def angle(self) -> float:
        """Return the current physical gimbal angle."""

    @abstractmethod
    def step(
        self,
        command: float,
        dt: float,
    ) -> float:
        """Advance the gimbal model and return its physical angle."""