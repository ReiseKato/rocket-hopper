import pytest

from fast_sim.control import (
    AltitudeReference2D,
    PDAltitudeController2D,
    PDAltitudeControllerParameters,
)
from fast_sim.models import HopperState2D


HOVER_THRUST = 19.62


def create_state(
    *,
    pz: float = 2.0,
    vz: float = 0.0,
) -> HopperState2D:
    return HopperState2D(
        px=0.0,
        pz=pz,
        vx=0.0,
        vz=vz,
        theta=0.0,
        omega=0.0,
    )


def create_controller() -> PDAltitudeController2D:
    return PDAltitudeController2D(
        parameters=PDAltitudeControllerParameters(
            proportional_gain=8.0,
            derivative_gain=7.2,
        ),
        hover_thrust=HOVER_THRUST,
    )


def test_controller_returns_hover_thrust_at_reference() -> None:
    controller = create_controller()

    command = controller.compute_thrust_command(
        state=create_state(),
        reference=AltitudeReference2D(
            pz=2.0,
            vz=0.0,
        ),
    )

    assert command == pytest.approx(
        HOVER_THRUST
    )


def test_below_reference_increases_thrust() -> None:
    controller = create_controller()

    command = controller.compute_thrust_command(
        state=create_state(
            pz=1.5,
        ),
        reference=AltitudeReference2D(
            pz=2.0,
        ),
    )

    assert command > HOVER_THRUST


def test_above_reference_decreases_thrust() -> None:
    controller = create_controller()

    command = controller.compute_thrust_command(
        state=create_state(
            pz=2.5,
        ),
        reference=AltitudeReference2D(
            pz=2.0,
        ),
    )

    assert command < HOVER_THRUST


def test_downward_velocity_increases_thrust() -> None:
    controller = create_controller()

    command = controller.compute_thrust_command(
        state=create_state(
            vz=-0.5,
        ),
        reference=AltitudeReference2D(
            pz=2.0,
            vz=0.0,
        ),
    )

    assert command > HOVER_THRUST


def test_upward_velocity_decreases_thrust() -> None:
    controller = create_controller()

    command = controller.compute_thrust_command(
        state=create_state(
            vz=0.5,
        ),
        reference=AltitudeReference2D(
            pz=2.0,
            vz=0.0,
        ),
    )

    assert command < HOVER_THRUST


def test_controller_does_not_apply_upper_actuator_limit() -> None:
    controller = create_controller()

    command = controller.compute_thrust_command(
        state=create_state(
            pz=0.0,
        ),
        reference=AltitudeReference2D(
            pz=2.0,
        ),
    )

    assert command > 30.0


def test_controller_does_not_return_negative_thrust() -> None:
    controller = create_controller()

    command = controller.compute_thrust_command(
        state=create_state(
            pz=100.0,
        ),
        reference=AltitudeReference2D(
            pz=2.0,
        ),
    )

    assert command == pytest.approx(0.0)
