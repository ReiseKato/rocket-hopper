"""State data structures for the 2D hopper model."""

from dataclasses import dataclass
from typing import ClassVar, Self

import numpy as np
from numpy.typing import ArrayLike, NDArray


@dataclass(frozen=True, slots=True)
class HopperState2D:
    """Mechanical state of the 2D rigid-body hopper.

    Attributes:
        px: Horizontal CG position in world coordinates [m].
        pz: Vertical CG position in world coordinates [m].
        vx: Horizontal CG velocity in world coordinates [m/s].
        vz: Vertical CG velocity in world coordinates [m/s].
        theta: Vehicle attitude angle [rad]. Positive tilts the nose right.
        omega: Angular velocity about the positive y-axis [rad/s].
    """

    px: float
    pz: float
    vx: float
    vz: float
    theta: float
    omega: float

    VECTOR_SIZE: ClassVar[int] = 6
    VECTOR_ORDER: ClassVar[tuple[str, ...]] = (
        "px",
        "pz",
        "vx",
        "vz",
        "theta",
        "omega",
    )

    def to_vector(self) -> NDArray[np.float64]:
        """Return the state as a new numerical vector."""
        return np.array(
            [
                self.px,
                self.pz,
                self.vx,
                self.vz,
                self.theta,
                self.omega,
            ],
            dtype=np.float64,
        )

    @classmethod
    def from_vector(cls, values: ArrayLike) -> Self:
        """Create a state from the canonical numerical vector representation."""
        vector = np.asarray(values, dtype=np.float64)

        if vector.shape != (cls.VECTOR_SIZE,):
            raise ValueError(
                f"state vector must have shape ({cls.VECTOR_SIZE},), "
                f"got {vector.shape}"
            )

        return cls(
            px=float(vector[0]),
            pz=float(vector[1]),
            vx=float(vector[2]),
            vz=float(vector[3]),
            theta=float(vector[4]),
            omega=float(vector[5]),
        )


@dataclass(frozen=True, slots=True)
class HopperStateDerivative2D:
    """Time derivative of :class:`HopperState2D`.

    Attributes:
        px_dot: Horizontal position derivative [m/s].
        pz_dot: Vertical position derivative [m/s].
        vx_dot: Horizontal acceleration [m/s^2].
        vz_dot: Vertical acceleration [m/s^2].
        theta_dot: Attitude-angle derivative [rad/s].
        omega_dot: Angular acceleration [rad/s^2].
    """

    px_dot: float
    pz_dot: float
    vx_dot: float
    vz_dot: float
    theta_dot: float
    omega_dot: float

    VECTOR_SIZE: ClassVar[int] = 6
    VECTOR_ORDER: ClassVar[tuple[str, ...]] = (
        "px_dot",
        "pz_dot",
        "vx_dot",
        "vz_dot",
        "theta_dot",
        "omega_dot",
    )

    def to_vector(self) -> NDArray[np.float64]:
        """Return the derivative as a new numerical vector."""
        return np.array(
            [
                self.px_dot,
                self.pz_dot,
                self.vx_dot,
                self.vz_dot,
                self.theta_dot,
                self.omega_dot,
            ],
            dtype=np.float64,
        )
