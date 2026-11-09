import math

import pytest

from fast_sim.control import (
    HorizontalPositionReference2D,
    PDHorizontalPositionController2D,
    PDHorizontalPositionControllerParameters,
)
from fast_sim.models import HopperState2D


GRAVITY = 9.81


def create_state(
    *,
    px: float = 0.0,
    vx: float = 0.0,
) -> HopperState2D:
    return HopperState2D(
        px=px,
        pz=2.0,
        vx=vx,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )


def create_controller(
    *,
    proportional_gain: float = 1.0,
    derivative_gain: float = 1.0,
    max_attitude_angle: float = math.radians(10.0),
) -> PDHorizontalPositionController2D:
    return PDHorizontalPositionController2D(
        parameters=PDHorizontalPositionControllerParameters(
            proportional_gain=proportional_gain,
            derivative_gain=derivative_gain,
        ),
        gravity=GRAVITY,
        max_attitude_angle=max_attitude_angle,
    )


def test_controller_returns_level_attitude_at_reference() -> None:
    controller = create_controller()

    attitude_reference = (
        controller.compute_attitude_reference(
            state=create_state(),
            reference=HorizontalPositionReference2D(
                px=0.0,
                vx=0.0,
            ),
        )
    )

    assert attitude_reference.theta == pytest.approx(
        0.0
    )

    assert attitude_reference.omega == pytest.approx(
        0.0
    )


def test_positive_position_error_requests_positive_attitude() -> None:
    controller = create_controller()

    attitude_reference = (
        controller.compute_attitude_reference(
            state=create_state(
                px=0.0,
            ),
            reference=HorizontalPositionReference2D(
                px=1.0,
            ),
        )
    )

    assert attitude_reference.theta > 0.0


def test_rightward_velocity_requests_negative_attitude_for_braking() -> None:
    controller = create_controller(
        proportional_gain=0.0,
        derivative_gain=1.0,
    )

    attitude_reference = (
        controller.compute_attitude_reference(
            state=create_state(
                vx=1.0,
            ),
            reference=HorizontalPositionReference2D(
                px=0.0,
                vx=0.0,
            ),
        )
    )

    assert attitude_reference.theta < 0.0


def test_acceleration_mapping_uses_inverse_sine() -> None:
    controller = create_controller(
        proportional_gain=1.0,
        derivative_gain=0.0,
        max_attitude_angle=math.radians(45.0),
    )

    desired_acceleration = 0.5 * GRAVITY

    attitude_reference = (
        controller.compute_attitude_reference(
            state=create_state(),
            reference=HorizontalPositionReference2D(
                px=desired_acceleration,
            ),
        )
    )

    assert attitude_reference.theta == pytest.approx(
        math.radians(30.0)
    )


def test_positive_command_is_limited_by_max_attitude() -> None:
    max_angle = math.radians(10.0)

    controller = create_controller(
        proportional_gain=100.0,
        derivative_gain=0.0,
        max_attitude_angle=max_angle,
    )

    attitude_reference = (
        controller.compute_attitude_reference(
            state=create_state(),
            reference=HorizontalPositionReference2D(
                px=100.0,
            ),
        )
    )

    assert attitude_reference.theta == pytest.approx(
        max_angle
    )


def test_negative_command_is_limited_symmetrically() -> None:
    max_angle = math.radians(10.0)

    controller = create_controller(
        proportional_gain=100.0,
        derivative_gain=0.0,
        max_attitude_angle=max_angle,
    )

    attitude_reference = (
        controller.compute_attitude_reference(
            state=create_state(),
            reference=HorizontalPositionReference2D(
                px=-100.0,
            ),
        )
    )

    assert attitude_reference.theta == pytest.approx(
        -max_angle
    )


def test_max_acceleration_matches_attitude_limit() -> None:
    max_angle = math.radians(10.0)

    controller = create_controller(
        max_attitude_angle=max_angle,
    )

    assert (
        controller.max_horizontal_acceleration
        == pytest.approx(
            GRAVITY * math.sin(max_angle)
        )
    )


@pytest.mark.parametrize(
    "max_attitude_angle",
    [
        0.0,
        -0.1,
        math.pi / 2.0,
        math.pi,
    ],
)
def test_controller_rejects_invalid_attitude_limit(
    max_attitude_angle: float,
) -> None:
    with pytest.raises(ValueError):
        create_controller(
            max_attitude_angle=max_attitude_angle,
        )
