import pytest

from fast_sim.actuators import (
    LimitedThrustActuator,
    ThrustActuatorParameters,
)


def test_thrust_passes_through_within_limits() -> None:
    actuator = LimitedThrustActuator(
        ThrustActuatorParameters(
            min_thrust=0.0,
            max_thrust=20.0,
        )
    )

    assert actuator.apply(10.0) == pytest.approx(10.0)


def test_thrust_is_limited_at_maximum() -> None:
    actuator = LimitedThrustActuator(
        ThrustActuatorParameters(
            min_thrust=0.0,
            max_thrust=20.0,
        )
    )

    assert actuator.apply(30.0) == pytest.approx(20.0)


def test_thrust_is_limited_at_minimum() -> None:
    actuator = LimitedThrustActuator(
        ThrustActuatorParameters(
            min_thrust=5.0,
            max_thrust=20.0,
        )
    )

    assert actuator.apply(2.0) == pytest.approx(5.0)