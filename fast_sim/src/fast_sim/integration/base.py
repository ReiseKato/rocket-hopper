"""Common interfaces for numerical integration."""

from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import TypeAlias

import numpy as np
from numpy.typing import NDArray


StateVector: TypeAlias = NDArray[np.float64]

DerivativeFunction: TypeAlias = Callable[
    [float, StateVector],
    StateVector,
]


class Integrator(ABC):
    """Base class for numerical state integrators.

    Integrators operate exclusively on numerical state vectors.

    They do not know about hopper-specific state classes, vehicle
    parameters, actuators, controllers, or physical models.
    """

    @abstractmethod
    def step(
        self,
        derivative: DerivativeFunction,
        state: StateVector,
        time: float,
        dt: float,
    ) -> StateVector:
        """Advance a numerical state by one integration step.

        Args:
            derivative:
                Function implementing dx/dt = f(t, x).
            state:
                State vector at the beginning of the step.
            time:
                Simulation time at the beginning of the step in seconds.
            dt:
                Positive integration step size in seconds.

        Returns:
            State vector at time + dt.
        """

    @staticmethod
    def _prepare_state(state: StateVector) -> StateVector:
        """Return a validated float64 copy of a state vector."""
        vector = np.array(
            state,
            dtype=np.float64,
            copy=True,
        )

        if vector.ndim != 1:
            raise ValueError(
                "state must be a one-dimensional vector"
            )

        if vector.size == 0:
            raise ValueError(
                "state vector must not be empty"
            )

        return vector

    @staticmethod
    def _validate_time_step(
        time: float,
        dt: float,
    ) -> None:
        """Validate simulation time and integration step size."""
        if not np.isfinite(time):
            raise ValueError("time must be finite")

        if not np.isfinite(dt):
            raise ValueError("dt must be finite")

        if dt <= 0.0:
            raise ValueError("dt must be > 0")

    @staticmethod
    def _evaluate_derivative(
        derivative: DerivativeFunction,
        time: float,
        state: StateVector,
    ) -> StateVector:
        """Evaluate and validate a derivative function."""
        state_derivative = np.asarray(
            derivative(time, state),
            dtype=np.float64,
        )

        if state_derivative.shape != state.shape:
            raise ValueError(
                "derivative must have the same shape as state: "
                f"expected {state.shape}, "
                f"got {state_derivative.shape}"
            )

        return state_derivative