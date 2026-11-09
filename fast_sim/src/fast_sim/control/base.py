"""Common controller interfaces."""

from abc import ABC, abstractmethod

from fast_sim.control.reference import (
    AltitudeReference2D,
    AttitudeReference2D,
    HorizontalPositionReference2D,
)
from fast_sim.models import HopperState2D


class AttitudeController2D(ABC):
    """Interface for 2D attitude controllers."""

    @abstractmethod
    def compute_gimbal_command(
        self,
        state: HopperState2D,
        reference: AttitudeReference2D,
    ) -> float:
        """Return the requested gimbal angle in radians."""


class AltitudeController2D(ABC):
    """Interface for 2D altitude controllers."""

    @abstractmethod
    def compute_thrust_command(
        self,
        state: HopperState2D,
        reference: AltitudeReference2D,
    ) -> float:
        """Return the requested thrust in newtons."""


class HorizontalPositionController2D(ABC):
    """Interface for 2D horizontal position controllers."""

    @abstractmethod
    def compute_attitude_reference(
        self,
        state: HopperState2D,
        reference: HorizontalPositionReference2D,
    ) -> AttitudeReference2D:
        """Return the attitude reference for the inner attitude loop."""
