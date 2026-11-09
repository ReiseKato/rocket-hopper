import math

import pytest

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.integration import RK4Integrator
from fast_sim.landing import (
    IntegratedSingleContactStepper2D,
    SingleContactConstraint2D,
    SingleContactDynamics2D,
    SingleContactFoot2D,
    SingleContactState2D,
)
from fast_sim.models import (
    HopperActuation2D,
    RigidBody2DParameters,
)


def test_integrated_single_contact_stepper_uses_real_rk4_interface() -> None:
    parameters = RigidBody2DParameters(
        mass=2.0,
        inertia_y=0.08,
        engine_offset=0.25,
        gravity=9.81,
    )

    geometry = LandingGearGeometry2D(
        foot_spacing=0.6,
        cg_to_feet=0.8,
    )

    dynamics = SingleContactDynamics2D(
        parameters,
        geometry,
    )

    stepper = IntegratedSingleContactStepper2D(
        dynamics=dynamics,
        integrator=RK4Integrator(),
    )

    initial_state = SingleContactState2D(
        theta=math.radians(5.0),
        omega=0.0,
    )

    constraint = SingleContactConstraint2D(
        foot=SingleContactFoot2D.RIGHT,
        contact_x=0.3,
        ground_height=0.0,
    )

    next_state = stepper.step(
        time=1.25,
        state=initial_state,
        constraint=constraint,
        actuation=HopperActuation2D(
            thrust=0.0,
            gimbal_angle=0.0,
        ),
        dt=0.01,
    )

    assert math.isfinite(next_state.theta)
    assert math.isfinite(next_state.omega)

    assert next_state.theta == pytest.approx(
        initial_state.theta,
        abs=1e-3,
    )

    # With the right foot fixed and zero thrust, gravity should start
    # rotating the vehicle toward the second-foot contact.
    assert next_state.omega < 0.0
