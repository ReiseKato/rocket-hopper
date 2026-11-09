import math

import pytest

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing import (
    SingleContactConstraint2D,
    SingleContactDynamics2D,
    SingleContactFoot2D,
    SingleContactState2D,
    TouchdownContact2D,
    TouchdownDetector2D,
)
from fast_sim.models import (
    HopperActuation2D,
    RigidBody2DParameters,
)


MASS = 2.0
INERTIA_Y = 0.08
ENGINE_OFFSET = 0.25
GRAVITY = 9.81

FOOT_SPACING = 0.6
CG_TO_FEET = 0.8


def create_dynamics() -> SingleContactDynamics2D:
    return SingleContactDynamics2D(
        RigidBody2DParameters(
            mass=MASS,
            inertia_y=INERTIA_Y,
            engine_offset=ENGINE_OFFSET,
            gravity=GRAVITY,
        ),
        LandingGearGeometry2D(
            foot_spacing=FOOT_SPACING,
            cg_to_feet=CG_TO_FEET,
        ),
    )


def test_inertia_about_contact_uses_parallel_axis_theorem() -> None:
    dynamics = create_dynamics()

    expected = (
        INERTIA_Y
        + MASS
        * (
            (FOOT_SPACING / 2.0) ** 2
            + CG_TO_FEET**2
        )
    )

    assert dynamics.inertia_about_contact() == pytest.approx(
        expected
    )


def test_gravity_rotates_right_contact_toward_second_foot() -> None:
    dynamics = create_dynamics()

    derivative = dynamics.derivative(
        SingleContactState2D(
            theta=0.0,
            omega=0.0,
        ),
        SingleContactConstraint2D(
            foot=SingleContactFoot2D.RIGHT,
            contact_x=0.3,
            ground_height=0.0,
        ),
        HopperActuation2D(
            thrust=0.0,
            gimbal_angle=0.0,
        ),
    )

    assert derivative.omega_dot < 0.0


def test_gravity_rotates_left_contact_toward_second_foot() -> None:
    dynamics = create_dynamics()

    derivative = dynamics.derivative(
        SingleContactState2D(
            theta=0.0,
            omega=0.0,
        ),
        SingleContactConstraint2D(
            foot=SingleContactFoot2D.LEFT,
            contact_x=-0.3,
            ground_height=0.0,
        ),
        HopperActuation2D(
            thrust=0.0,
            gimbal_angle=0.0,
        ),
    )

    assert derivative.omega_dot > 0.0


def test_hover_thrust_cancels_gravity_torque_when_upright() -> None:
    dynamics = create_dynamics()

    derivative = dynamics.derivative(
        SingleContactState2D(
            theta=0.0,
            omega=0.0,
        ),
        SingleContactConstraint2D(
            foot=SingleContactFoot2D.RIGHT,
            contact_x=0.3,
            ground_height=0.0,
        ),
        HopperActuation2D(
            thrust=MASS * GRAVITY,
            gimbal_angle=0.0,
        ),
    )

    assert derivative.omega_dot == pytest.approx(
        0.0,
        abs=1e-12,
    )


def test_reconstructed_contact_foot_remains_fixed() -> None:
    dynamics = create_dynamics()

    constraint = SingleContactConstraint2D(
        foot=SingleContactFoot2D.RIGHT,
        contact_x=1.25,
        ground_height=0.4,
    )

    reduced_state = SingleContactState2D(
        theta=math.radians(7.0),
        omega=math.radians(-12.0),
    )

    hopper_state = dynamics.to_hopper_state(
        reduced_state,
        constraint,
    )

    positions = dynamics.geometry.foot_positions_world(
        hopper_state
    )

    assert positions.right_x == pytest.approx(
        constraint.contact_x
    )
    assert positions.right_z == pytest.approx(
        constraint.ground_height
    )


def test_second_contact_is_detectable_from_reconstructed_state() -> None:
    dynamics = create_dynamics()

    constraint = SingleContactConstraint2D(
        foot=SingleContactFoot2D.RIGHT,
        contact_x=0.3,
        ground_height=0.0,
    )

    hopper_state = dynamics.to_hopper_state(
        SingleContactState2D(
            theta=0.0,
            omega=-0.1,
        ),
        constraint,
    )

    detection = TouchdownDetector2D(
        dynamics.geometry,
        ground_height=0.0,
    ).detect(
        hopper_state
    )

    assert detection.contact is TouchdownContact2D.BOTH
