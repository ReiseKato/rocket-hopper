"""Minimal 2D landing-gear geometry."""

from dataclasses import dataclass
from math import cos, isfinite, sin

from fast_sim.models import HopperState2D


@dataclass(frozen=True, slots=True)
class LandingGearFootPositions2D:
    """World-frame positions of the two landing-gear contact points.

    Units:
        left_x: m
        left_z: m
        right_x: m
        right_z: m
    """

    left_x: float
    left_z: float
    right_x: float
    right_z: float

    @property
    def lowest_z(self) -> float:
        """Return the lower of the two world-frame foot heights."""
        return min(
            self.left_z,
            self.right_z,
        )


@dataclass(frozen=True, slots=True)
class LandingGearGeometry2D:
    """Minimal symmetric two-foot landing-gear geometry.

    The geometry is defined relative to the rigid-body center of gravity.

    The feet are located at

        left:  (-foot_spacing / 2, -cg_to_feet)
        right: (+foot_spacing / 2, -cg_to_feet)

    in the body x-z plane.
    """

    foot_spacing: float
    cg_to_feet: float

    def __post_init__(self) -> None:
        if not isfinite(self.foot_spacing):
            raise ValueError(
                "foot_spacing must be finite"
            )

        if not isfinite(self.cg_to_feet):
            raise ValueError(
                "cg_to_feet must be finite"
            )

        if self.foot_spacing <= 0.0:
            raise ValueError(
                "foot_spacing must be > 0"
            )

        if self.cg_to_feet <= 0.0:
            raise ValueError(
                "cg_to_feet must be > 0"
            )

    @property
    def half_foot_spacing(self) -> float:
        """Return half the distance between the two feet."""
        return 0.5 * self.foot_spacing

    def foot_positions_world(
        self,
        state: HopperState2D,
    ) -> LandingGearFootPositions2D:
        """Return both landing-foot positions in the world x-z plane.

        The body-to-world transformation follows the same coordinate
        convention as the rigid-body dynamics:

            x_W = p_x + x_B cos(theta) + z_B sin(theta)
            z_W = p_z - x_B sin(theta) + z_B cos(theta)

        with z_B = -cg_to_feet for both feet.
        """
        half_spacing = self.half_foot_spacing
        height = self.cg_to_feet

        sin_theta = sin(state.theta)
        cos_theta = cos(state.theta)

        left_body_x = -half_spacing
        right_body_x = half_spacing
        foot_body_z = -height

        left_x = (
            state.px
            + left_body_x * cos_theta
            + foot_body_z * sin_theta
        )

        left_z = (
            state.pz
            - left_body_x * sin_theta
            + foot_body_z * cos_theta
        )

        right_x = (
            state.px
            + right_body_x * cos_theta
            + foot_body_z * sin_theta
        )

        right_z = (
            state.pz
            - right_body_x * sin_theta
            + foot_body_z * cos_theta
        )

        return LandingGearFootPositions2D(
            left_x=left_x,
            left_z=left_z,
            right_x=right_x,
            right_z=right_z,
        )
