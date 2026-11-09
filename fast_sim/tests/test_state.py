import numpy as np
import pytest

from fast_sim.models import (
    HopperState2D,
    HopperStateDerivative2D,
)


def test_state_to_vector_uses_canonical_order() -> None:
    state = HopperState2D(
        px=1.0,
        pz=2.0,
        vx=3.0,
        vz=4.0,
        theta=5.0,
        omega=6.0,
    )

    np.testing.assert_allclose(
        state.to_vector(),
        np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]),
    )


def test_state_vector_round_trip() -> None:
    state = HopperState2D(
        px=1.0,
        pz=2.0,
        vx=3.0,
        vz=4.0,
        theta=5.0,
        omega=6.0,
    )

    reconstructed = HopperState2D.from_vector(
        state.to_vector()
    )

    assert reconstructed == state


def test_state_from_vector_rejects_wrong_shape() -> None:
    with pytest.raises(ValueError):
        HopperState2D.from_vector([1.0, 2.0])


def test_to_vector_does_not_mutate_state() -> None:
    state = HopperState2D(
        px=1.0,
        pz=2.0,
        vx=3.0,
        vz=4.0,
        theta=5.0,
        omega=6.0,
    )

    vector = state.to_vector()
    vector[0] = 999.0

    assert state.px == 1.0


def test_derivative_to_vector_uses_canonical_order() -> None:
    derivative = HopperStateDerivative2D(
        px_dot=1.0,
        pz_dot=2.0,
        vx_dot=3.0,
        vz_dot=4.0,
        theta_dot=5.0,
        omega_dot=6.0,
    )

    np.testing.assert_allclose(
        derivative.to_vector(),
        np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]),
    )