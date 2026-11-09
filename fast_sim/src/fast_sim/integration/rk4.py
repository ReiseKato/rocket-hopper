"""Classical fourth-order Runge-Kutta integrator."""

from fast_sim.integration.base import (
    DerivativeFunction,
    Integrator,
    StateVector,
)


class RK4Integrator(Integrator):
    """Classical fourth-order Runge-Kutta integrator."""

    def step(
        self,
        derivative: DerivativeFunction,
        state: StateVector,
        time: float,
        dt: float,
    ) -> StateVector:
        """Advance the state by one classical RK4 step."""
        self._validate_time_step(time, dt)

        current_state = self._prepare_state(state)

        k1 = self._evaluate_derivative(
            derivative=derivative,
            time=time,
            state=current_state,
        )

        k2_state = current_state + 0.5 * dt * k1
        k2 = self._evaluate_derivative(
            derivative=derivative,
            time=time + 0.5 * dt,
            state=k2_state,
        )

        k3_state = current_state + 0.5 * dt * k2
        k3 = self._evaluate_derivative(
            derivative=derivative,
            time=time + 0.5 * dt,
            state=k3_state,
        )

        k4_state = current_state + dt * k3
        k4 = self._evaluate_derivative(
            derivative=derivative,
            time=time + dt,
            state=k4_state,
        )

        return current_state + (
            dt / 6.0
        ) * (
            k1
            + 2.0 * k2
            + 2.0 * k3
            + k4
        )