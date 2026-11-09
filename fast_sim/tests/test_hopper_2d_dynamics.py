"""Physics-focused tests for the 2D hopper dynamics."""

from math import radians

import pytest

from fast_sim import (
    Hopper2DDynamics,
    HopperActuation2D,
    HopperState2D,
    RigidBody2DParameters,
)


@pytest.fixture
def parameters() -> RigidBody2DParameters:
    return RigidBody2DParameters(
        mass=2.0,
        inertia_y=0.08,
        engine_offset=0.25,
        gravity=9.81,
    )


@pytest.fixture
def resting_state() -> HopperState2D:
    return HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )


def test_free_fall(parameters: RigidBody2DParameters, resting_state: HopperState2D) -> None:
    dynamics = Hopper2DDynamics(parameters)
    derivative = dynamics.derivative(
        resting_state,
        HopperActuation2D(thrust=0.0, gimbal_angle=0.0),
    )

    assert derivative.vx_dot == pytest.approx(0.0, abs=1e-12)
    assert derivative.vz_dot == pytest.approx(-parameters.gravity)
    assert derivative.omega_dot == pytest.approx(0.0, abs=1e-12)


def test_ideal_hover(parameters: RigidBody2DParameters, resting_state: HopperState2D) -> None:
    dynamics = Hopper2DDynamics(parameters)
    hover_thrust = parameters.mass * parameters.gravity

    derivative = dynamics.derivative(
        resting_state,
        HopperActuation2D(thrust=hover_thrust, gimbal_angle=0.0),
    )

    assert derivative.vx_dot == pytest.approx(0.0, abs=1e-12)
    assert derivative.vz_dot == pytest.approx(0.0, abs=1e-12)
    assert derivative.omega_dot == pytest.approx(0.0, abs=1e-12)


def test_tilted_vehicle_without_gimbal_has_no_gimbal_torque(
    parameters: RigidBody2DParameters,
) -> None:
    dynamics = Hopper2DDynamics(parameters)
    state = HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=radians(10.0),
        omega=0.0,
    )
    hover_thrust = parameters.mass * parameters.gravity

    derivative = dynamics.derivative(
        state,
        HopperActuation2D(thrust=hover_thrust, gimbal_angle=0.0),
    )

    assert derivative.vx_dot == pytest.approx(1.7034886, rel=1e-6)
    assert derivative.vz_dot == pytest.approx(-0.149035943, rel=1e-6)
    assert derivative.omega_dot == pytest.approx(0.0, abs=1e-12)


def test_positive_gimbal_accelerates_right_and_rotates_nose_left(
    parameters: RigidBody2DParameters,
    resting_state: HopperState2D,
) -> None:
    dynamics = Hopper2DDynamics(parameters)
    hover_thrust = parameters.mass * parameters.gravity

    derivative = dynamics.derivative(
        resting_state,
        HopperActuation2D(
            thrust=hover_thrust,
            gimbal_angle=radians(5.0),
        ),
    )

    assert derivative.vx_dot == pytest.approx(0.8549978, rel=1e-6)
    assert derivative.vz_dot == pytest.approx(-0.0373300, rel=1e-6)
    assert derivative.omega_dot == pytest.approx(-5.3437360, rel=1e-6)


def test_gimbal_response_is_symmetric_about_zero(
    parameters: RigidBody2DParameters,
    resting_state: HopperState2D,
) -> None:
    dynamics = Hopper2DDynamics(parameters)
    hover_thrust = parameters.mass * parameters.gravity
    angle = radians(5.0)

    positive = dynamics.derivative(
        resting_state,
        HopperActuation2D(thrust=hover_thrust, gimbal_angle=angle),
    )
    negative = dynamics.derivative(
        resting_state,
        HopperActuation2D(thrust=hover_thrust, gimbal_angle=-angle),
    )

    assert positive.vx_dot == pytest.approx(-negative.vx_dot)
    assert positive.vz_dot == pytest.approx(negative.vz_dot)
    assert positive.omega_dot == pytest.approx(-negative.omega_dot)


def test_vertical_world_thrust_can_coexist_with_nonzero_torque(
    parameters: RigidBody2DParameters,
) -> None:
    dynamics = Hopper2DDynamics(parameters)
    angle = radians(10.0)
    state = HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=angle,
        omega=0.0,
    )
    hover_thrust = parameters.mass * parameters.gravity

    derivative = dynamics.derivative(
        state,
        HopperActuation2D(thrust=hover_thrust, gimbal_angle=-angle),
    )

    assert derivative.vx_dot == pytest.approx(0.0, abs=1e-12)
    assert derivative.vz_dot == pytest.approx(0.0, abs=1e-12)
    assert derivative.omega_dot == pytest.approx(10.646803893, rel=1e-6)
