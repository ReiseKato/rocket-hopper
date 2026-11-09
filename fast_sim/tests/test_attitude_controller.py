import math

import pytest

from fast_sim.control import (
    AttitudeReference2D,
    PDAttitudeController2D,
    PDAttitudeControllerParameters,
)
from fast_sim.models import HopperState2D


def create_state(
    *,
    theta: float = 0.0,
    omega: float = 0.0,
) -> HopperState2D:
    return HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=theta,
        omega=omega,
    )


def create_controller() -> PDAttitudeController2D:
    return PDAttitudeController2D(
        PDAttitudeControllerParameters(
            proportional_gain=2.0,
            derivative_gain=0.5,
        )
    )


def test_positive_attitude_error_produces_positive_gimbal_command() -> None:
    controller = create_controller()

    command = controller.compute_gimbal_command(
        state=create_state(
            theta=math.radians(5.0)
        ),
        reference=AttitudeReference2D(
            theta=0.0
        ),
    )

    assert command > 0.0


def test_negative_attitude_error_produces_negative_gimbal_command() -> None:
    controller = create_controller()

    command = controller.compute_gimbal_command(
        state=create_state(
            theta=math.radians(-5.0)
        ),
        reference=AttitudeReference2D(
            theta=0.0
        ),
    )

    assert command < 0.0


def test_positive_angular_velocity_produces_damping_command() -> None:
    controller = create_controller()

    command = controller.compute_gimbal_command(
        state=create_state(
            theta=0.0,
            omega=math.radians(10.0),
        ),
        reference=AttitudeReference2D(
            theta=0.0,
            omega=0.0,
        ),
    )

    assert command > 0.0


def test_controller_returns_zero_at_reference() -> None:
    controller = create_controller()

    command = controller.compute_gimbal_command(
        state=create_state(
            theta=math.radians(3.0),
            omega=math.radians(-2.0),
        ),
        reference=AttitudeReference2D(
            theta=math.radians(3.0),
            omega=math.radians(-2.0),
        ),
    )

    assert command == pytest.approx(0.0)


def test_attitude_error_uses_shortest_wrapped_direction() -> None:
    controller = PDAttitudeController2D(
        PDAttitudeControllerParameters(
            proportional_gain=1.0,
            derivative_gain=0.0,
        )
    )

    command = controller.compute_gimbal_command(
        state=create_state(
            theta=math.radians(179.0)
        ),
        reference=AttitudeReference2D(
            theta=math.radians(-179.0)
        ),
    )

    assert command == pytest.approx(
        math.radians(-2.0)
    )


def test_controller_does_not_apply_actuator_angle_limits() -> None:
    controller = PDAttitudeController2D(
        PDAttitudeControllerParameters(
            proportional_gain=10.0,
            derivative_gain=0.0,
        )
    )

    command = controller.compute_gimbal_command(
        state=create_state(
            theta=math.radians(20.0)
        ),
        reference=AttitudeReference2D(
            theta=0.0
        ),
    )

    assert command > math.radians(10.0)
