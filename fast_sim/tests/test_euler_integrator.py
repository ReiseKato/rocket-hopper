import numpy as np
import pytest

from fast_sim.integration import EulerIntegrator


def test_euler_integrates_constant_derivative() -> None:
    integrator = EulerIntegrator()

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


def test_euler_passes_current_time_to_derivative() -> None:
    integrator = EulerIntegrator()

    state = np.array(
        [0.0],
        dtype=np.float64,
    )

    def derivative(
        time: float,
        state: np.ndarray,
    ) -> np.ndarray:
        return np.array(
            [time],
            dtype=np.float64,
        )

    next_state = integrator.step(
        derivative=derivative,
        state=state,
        time=2.0,
        dt=0.5,
    )

    np.testing.assert_allclose(
        next_state,
        np.array([1.0]),
    )


def test_euler_does_not_mutate_input_state() -> None:
    integrator = EulerIntegrator()

    state = np.array(
        [1.0, 2.0],
        dtype=np.float64,
    )

    original_state = state.copy()

    def derivative(
        time: float,
        state: np.ndarray,
    ) -> np.ndarray:
        return np.array(
            [1.0, 1.0],
            dtype=np.float64,
        )

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
def test_euler_rejects_invalid_dt(
    dt: float,
) -> None:
    integrator = EulerIntegrator()

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


def test_euler_rejects_wrong_derivative_shape() -> None:
    integrator = EulerIntegrator()

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