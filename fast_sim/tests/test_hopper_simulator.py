import math

import numpy as np
import pytest

from fast_sim.dynamics import Hopper2DDynamics
from fast_sim.integration import (
    DerivativeFunction,
    EulerIntegrator,
    Integrator,
    StateVector,
)
from fast_sim.models import (
    HopperActuation2D,
    HopperState2D,
    RigidBody2DParameters,
)
from fast_sim.simulation import HopperSimulator


def create_test_dynamics() -> Hopper2DDynamics:
    parameters = RigidBody2DParameters(
        mass=2.0,
        inertia_y=0.08,
        engine_offset=0.25,
    )

    return Hopper2DDynamics(parameters)


def create_initial_state() -> HopperState2D:
    return HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )


def test_hover_remains_stationary_after_euler_step() -> None:
    dynamics = create_test_dynamics()

    simulator = HopperSimulator(
        dynamics=dynamics,
        integrator=EulerIntegrator(),
        initial_state=create_initial_state(),
    )

    mass = dynamics.parameters.mass
    gravity = dynamics.parameters.gravity

    actuation = HopperActuation2D(
        thrust=mass * gravity,
        gimbal_angle=0.0,
    )

    next_state = simulator.step(
        actuation=actuation,
        dt=0.01,
    )

    np.testing.assert_allclose(
        next_state.to_vector(),
        create_initial_state().to_vector(),
    )

    assert simulator.time == pytest.approx(0.01)


def test_euler_step_matches_expected_gimbal_response() -> None:
    dynamics = create_test_dynamics()

    simulator = HopperSimulator(
        dynamics=dynamics,
        integrator=EulerIntegrator(),
        initial_state=create_initial_state(),
    )

    thrust = (
        dynamics.parameters.mass
        * dynamics.parameters.gravity
    )

    delta = math.radians(5.0)
    dt = 0.01

    actuation = HopperActuation2D(
        thrust=thrust,
        gimbal_angle=delta,
    )

    next_state = simulator.step(
        actuation=actuation,
        dt=dt,
    )

    expected_ax = (
        thrust
        / dynamics.parameters.mass
        * math.sin(delta)
    )

    expected_az = (
        thrust
        / dynamics.parameters.mass
        * math.cos(delta)
        - dynamics.parameters.gravity
    )

    expected_angular_acceleration = (
        -dynamics.parameters.engine_offset
        * thrust
        / dynamics.parameters.inertia_y
        * math.sin(delta)
    )

    assert next_state.px == pytest.approx(0.0)
    assert next_state.pz == pytest.approx(2.0)

    assert next_state.vx == pytest.approx(
        expected_ax * dt
    )

    assert next_state.vz == pytest.approx(
        expected_az * dt
    )

    assert next_state.theta == pytest.approx(0.0)

    assert next_state.omega == pytest.approx(
        expected_angular_acceleration * dt
    )


def test_simulator_works_with_another_integrator_implementation() -> None:
    class PassThroughIntegrator(Integrator):
        def step(
            self,
            derivative: DerivativeFunction,
            state: StateVector,
            time: float,
            dt: float,
        ) -> StateVector:
            return state.copy()

    initial_state = create_initial_state()

    simulator = HopperSimulator(
        dynamics=create_test_dynamics(),
        integrator=PassThroughIntegrator(),
        initial_state=initial_state,
    )

    actuation = HopperActuation2D(
        thrust=0.0,
        gimbal_angle=0.0,
    )

    next_state = simulator.step(
        actuation=actuation,
        dt=0.1,
    )

    assert next_state == initial_state
    assert simulator.state == initial_state
    assert simulator.time == pytest.approx(0.1)


def test_failed_integration_does_not_change_simulator_state() -> None:
    class FailingIntegrator(Integrator):
        def step(
            self,
            derivative: DerivativeFunction,
            state: StateVector,
            time: float,
            dt: float,
        ) -> StateVector:
            raise RuntimeError("integration failed")

    initial_state = create_initial_state()

    simulator = HopperSimulator(
        dynamics=create_test_dynamics(),
        integrator=FailingIntegrator(),
        initial_state=initial_state,
        initial_time=1.5,
    )

    actuation = HopperActuation2D(
        thrust=0.0,
        gimbal_angle=0.0,
    )

    with pytest.raises(
        RuntimeError,
        match="integration failed",
    ):
        simulator.step(
            actuation=actuation,
            dt=0.01,
        )

    assert simulator.state == initial_state
    assert simulator.time == pytest.approx(1.5)


@pytest.mark.parametrize(
    "initial_time",
    [
        float("inf"),
        float("-inf"),
        float("nan"),
    ],
)
def test_simulator_rejects_non_finite_initial_time(
    initial_time: float,
) -> None:
    with pytest.raises(ValueError):
        HopperSimulator(
            dynamics=create_test_dynamics(),
            integrator=EulerIntegrator(),
            initial_state=create_initial_state(),
            initial_time=initial_time,
        )