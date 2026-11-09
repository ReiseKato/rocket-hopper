"""Touchdown detection for the 2D hopper."""

from dataclasses import dataclass
from enum import Enum
from math import isfinite

from fast_sim.geometry import (
    LandingGearFootPositions2D,
    LandingGearGeometry2D,
)
from fast_sim.models import HopperState2D


class TouchdownContact2D(Enum):
    """Contact state of the two landing-gear feet."""

    NONE = "none"
    LEFT = "left"
    RIGHT = "right"
    BOTH = "both"


@dataclass(frozen=True, slots=True)
class TouchdownDetection2D:
    """Result of evaluating the landing gear against the ground.

    Clearances are measured vertically relative to the ground plane:

        clearance = foot_z - ground_height

    A clearance <= 0 means that the corresponding foot has reached or
    crossed the ground plane.
    """

    contact: TouchdownContact2D
    foot_positions: LandingGearFootPositions2D
    ground_height: float
    left_clearance: float
    right_clearance: float

    @property
    def is_touchdown(self) -> bool:
        """Return whether at least one foot has reached the ground."""
        return self.contact is not TouchdownContact2D.NONE

    @property
    def left_in_contact(self) -> bool:
        """Return whether the left foot has reached the ground."""
        return self.left_clearance <= 0.0

    @property
    def right_in_contact(self) -> bool:
        """Return whether the right foot has reached the ground."""
        return self.right_clearance <= 0.0

    @property
    def minimum_clearance(self) -> float:
        """Return the smaller vertical foot clearance."""
        return min(
            self.left_clearance,
            self.right_clearance,
        )


class TouchdownDetector2D:
    """Detect geometric touchdown against a horizontal ground plane.

    The detector contains no contact dynamics. It only evaluates the
    current rigid-body state and landing-gear geometry.
    """

    def __init__(
        self,
        geometry: LandingGearGeometry2D,
        *,
        ground_height: float = 0.0,
    ) -> None:
        if not isfinite(ground_height):
            raise ValueError(
                "ground_height must be finite"
            )

        self._geometry = geometry
        self._ground_height = ground_height

    @property
    def geometry(self) -> LandingGearGeometry2D:
        """Return the landing-gear geometry used by the detector."""
        return self._geometry

    @property
    def ground_height(self) -> float:
        """Return the world-frame height of the ground plane."""
        return self._ground_height

    def detect(
        self,
        state: HopperState2D,
    ) -> TouchdownDetection2D:
        """Evaluate the current rigid-body state for ground contact."""
        positions = self._geometry.foot_positions_world(
            state
        )

        left_clearance = (
            positions.left_z
            - self._ground_height
        )

        right_clearance = (
            positions.right_z
            - self._ground_height
        )

        left_in_contact = left_clearance <= 0.0
        right_in_contact = right_clearance <= 0.0

        if left_in_contact and right_in_contact:
            contact = TouchdownContact2D.BOTH
        elif left_in_contact:
            contact = TouchdownContact2D.LEFT
        elif right_in_contact:
            contact = TouchdownContact2D.RIGHT
        else:
            contact = TouchdownContact2D.NONE

        return TouchdownDetection2D(
            contact=contact,
            foot_positions=positions,
            ground_height=self._ground_height,
            left_clearance=left_clearance,
            right_clearance=right_clearance,
        )
