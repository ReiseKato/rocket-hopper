"""Physical parameter data structures for the 2D rigid body."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True, slots=True)
class RigidBody2DParameters:
    """Constant parameters of the 2D rigid-body dynamics model.

    Attributes:
        mass: Total vehicle mass [kg].
        inertia_y: Moment of inertia about the body y-axis through the CG
            [kg m^2].
        engine_offset: Positive distance from the CG to the thrust application
            point along -z_B [m].
        gravity: Magnitude of gravitational acceleration [m/s^2].
    """

    mass: float
    inertia_y: float
    engine_offset: float
    gravity: float = 9.81

    def __post_init__(self) -> None:
        values = {
            "mass": self.mass,
            "inertia_y": self.inertia_y,
            "engine_offset": self.engine_offset,
            "gravity": self.gravity,
        }
        for name, value in values.items():
            if not isfinite(value):
                raise ValueError(f"{name} must be finite")

        if self.mass <= 0.0:
            raise ValueError("mass must be > 0")
        if self.inertia_y <= 0.0:
            raise ValueError("inertia_y must be > 0")
        if self.engine_offset < 0.0:
            raise ValueError("engine_offset must be >= 0")
        if self.gravity < 0.0:
            raise ValueError("gravity must be >= 0")
