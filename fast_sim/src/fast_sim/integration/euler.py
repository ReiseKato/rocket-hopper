"""Explicit Euler reference integrator."""

from fast_sim.integration.base import (
    DerivativeFunction,
    Integrator,
    StateVector,
)


class EulerIntegrator(Integrator):
    """Explicit Euler reference implementation."""

    def step(
        self,
        derivative: DerivativeFunction,
        state: StateVector,
        time: float,
        dt: float,
    ) -> StateVector:
        """Advance the state by one explicit Euler step."""
        self._validate_time_step(time, dt)

        current_state = self._prepare_state(state)

        state_derivative = self._evaluate_derivative(
            derivative=derivative,
            time=time,
            state=current_state,
        )

        return current_state + dt * state_derivative