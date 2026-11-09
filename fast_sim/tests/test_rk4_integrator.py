import numpy as np
import pytest

from fast_sim.integration import RK4Integrator


def test_rk4_integrates_constant_derivative() -> None:
    integrator = RK4Integrator()

    state = np.array(
        [1.0, 2.0],
        dtype=np.float64,
    )

    def derivative(
        time: float,
        state: np.ndarray,
    ) -> np.ndarray:
        return np.array(
            [3.0, -4.0],
            dtype=np.float64,
        )

    next_state = integrator.step(
        derivative=derivative,
        state=state,
        time=0.0,
        dt=0.1,
    )

    np.testing.assert_allclose(
        next_state,
        np.array([1.3, 1.6]),
    )


def test_rk4_integrates_exponential_growth() -> None:
    integrator = RK4Integrator()

    state = np.array(
        [1.0],
        dtype=np.float64,
    )

    def derivative(
        time: float,
        state: np.ndarray,
    ) -> np.ndarray:
        return state

    next_state = integrator.step(
        derivative=derivative,
        state=state,
        time=0.0,
        dt=0.1,
    )

    np.testing.assert_allclose(
        next_state,
        np.array([np.exp(0.1)]),
        rtol=1e-6,
    )


def test_rk4_evaluates_derivative_at_expected_times() -> None:
    integrator = RK4Integrator()

    state = np.array(
        [0.0],
        dtype=np.float64,
    )

    evaluated_times: list[float] = []

    def derivative(
        time: float,
        state: np.ndarray,
    ) -> np.ndarray:
        evaluated_times.append(time)

        return np.array(
            [0.0],
            dtype=np.float64,
        )

    integrator.step(
        derivative=derivative,
        state=state,
        time=2.0,
        dt=0.4,
    )

    np.testing.assert_allclose(
        evaluated_times,
        [
            2.0,
            2.2,
            2.2,
            2.4,
        ],
    )


def test_rk4_does_not_mutate_input_state() -> None:
    integrator = RK4Integrator()

    state = np.array(
        [1.0, 2.0],
        dtype=np.float64,
    )

    original_state = state.copy()

    def derivative(
        time: float,
        state: np.ndarray,
    ) -> np.ndarray:
        return state

    integrator.step(
        derivative=derivative,
        state=state,
        time=0.0,
        dt=0.1,
    )

    np.testing.assert_array_equal(
        state,
        original_state,
    )


@pytest.mark.parametrize(
    "dt",
    [
        0.0,
        -0.1,
        float("inf"),
        float("nan"),
    ],
)
def test_rk4_rejects_invalid_dt(
    dt: float,
) -> None:
    integrator = RK4Integrator()

    state = np.array(
        [1.0],
        dtype=np.float64,
    )

    def derivative(
        time: float,
        state: np.ndarray,
    ) -> np.ndarray:
        return np.array(
            [0.0],
            dtype=np.float64,
        )

    with pytest.raises(ValueError):
        integrator.step(
            derivative=derivative,
            state=state,
            time=0.0,
            dt=dt,
        )


def test_rk4_rejects_wrong_derivative_shape() -> None:
    integrator = RK4Integrator()

    state = np.array(
        [1.0, 2.0],
        dtype=np.float64,
    )

    def derivative(
        time: float,
        state: np.ndarray,
    ) -> np.ndarray:
        return np.array(
            [1.0],
            dtype=np.float64,
        )

    with pytest.raises(ValueError):
        integrator.step(
            derivative=derivative,
            state=state,
            time=0.0,
            dt=0.1,
        )