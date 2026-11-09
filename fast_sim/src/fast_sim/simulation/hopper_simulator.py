"""Simulation orchestration for the 2D hopper."""

from math import isfinite

from fast_sim.dynamics import Hopper2DDynamics
from fast_sim.integration import Integrator, StateVector
from fast_sim.models import HopperActuation2D, HopperState2D


class HopperSimulator:
    """Coordinate one running 2D hopper simulation.

    The simulator owns the current simulation state and simulation time.

    It does not implement vehicle dynamics or numerical integration
    itself. Those responsibilities remain delegated to Hopper2DDynamics
    and Integrator respectively.
    """

    def __init__(
        self,
        dynamics: Hopper2DDynamics,
        integrator: Integrator,
        initial_state: HopperState2D,
        initial_time: float = 0.0,
    ) -> None:
        if not isfinite(initial_time):
            raise ValueError("initial_time must be finite")

        self._dynamics = dynamics
        self._integrator = integrator
        self._state = initial_state
        self._time = initial_time

    @property
    def state(self) -> HopperState2D:
        """Return the current simulation state."""
        return self._state

    @property
    def time(self) -> float:
        """Return the current simulation time in seconds."""
        return self._time

    def step(
        self,
        actuation: HopperActuation2D,
        dt: float,
    ) -> HopperState2D:
        """Advance the simulation by one integration step.

        The supplied physical actuation is held constant for the complete
        integration step.

        Args:
            actuation:
                Physical thrust and gimbal angle applied during the step.
            dt:
                Integration step size in seconds.

        Returns:
            Updated hopper state at time + dt.
        """

        current_state_vector = self._state.to_vector()

        def derivative(
            time: float,
            state_vector: StateVector,
        ) -> StateVector:
            hopper_state = HopperState2D.from_vector(
                state_vector
            )

            state_derivative = self._dynamics.derivative(
                state=hopper_state,
                actuation=actuation,
            )

            return state_derivative.to_vector()

        next_state_vector = self._integrator.step(
            derivative=derivative,
            state=current_state_vector,
            time=self._time,
            dt=dt,
        )

        next_state = HopperState2D.from_vector(
            next_state_vector
        )

        next_time = self._time + dt

        if not isfinite(next_time):
            raise ValueError(
                "resulting simulation time must be finite"
            )

        self._state = next_state
        self._time = next_time

        return next_state