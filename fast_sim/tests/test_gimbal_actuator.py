import math

import pytest

from fast_sim.actuators import (
    GimbalActuatorParameters,
    RateLimitedGimbalActuator,
)


def create_gimbal() -> RateLimitedGimbalActuator:
    return RateLimitedGimbalActuator(
        parameters=GimbalActuatorParameters(
            max_angle=math.radians(10.0),
            max_rate=math.radians(100.0),
        ),
    )


def test_gimbal_respects_rate_limit() -> None:
    gimbal = create_gimbal()

    angle = gimbal.step(
        command=math.radians(10.0),
        dt=0.01,
    )

    assert angle == pytest.approx(
        math.radians(1.0)
    )


def test_gimbal_accumulates_rate_limited_steps() -> None:
    gimbal = create_gimbal()

    for _ in range(3):
        gimbal.step(
            command=math.radians(10.0),
            dt=0.01,
        )

    assert gimbal.angle == pytest.approx(
        math.radians(3.0)
    )


def test_gimbal_stops_at_command() -> None:
    gimbal = create_gimbal()

    for _ in range(20):
        gimbal.step(
            command=math.radians(5.0),
            dt=0.01,
        )

    assert gimbal.angle == pytest.approx(
        math.radians(5.0)
    )


def test_gimbal_respects_angle_limit() -> None:
    gimbal = create_gimbal()

    for _ in range(50):
        gimbal.step(
            command=math.radians(30.0),
            dt=0.01,
        )

    assert gimbal.angle == pytest.approx(
        math.radians(10.0)
    )


def test_gimbal_moves_symmetrically_in_negative_direction() -> None:
    gimbal = create_gimbal()

    angle = gimbal.step(
        command=math.radians(-10.0),
        dt=0.01,
    )

    assert angle == pytest.approx(
        math.radians(-1.0)
    )