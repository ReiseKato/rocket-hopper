import math

import pytest

from fast_sim.dynamics import Hopper2DDynamics
from fast_sim.integration import (
    EulerIntegrator,
    RK4Integrator
)
from fast_sim.models import (
    HopperActuation2D,
    HopperState2D,
    RigidBody2DParameters,
)
from fast_sim.simulation import HopperSimulator


MASS = 2.0
INERTIA_Y = 0.08
ENGINE_OFFSET = 0.25
GRAVITY = 9.81


def create_simulator(
    initial_state: HopperState2D,
) -> HopperSimulator:
    parameters = RigidBody2DParameters(
        mass=MASS,
        inertia_y=INERTIA_Y,
        engine_offset=ENGINE_OFFSET,
        gravity=GRAVITY,
    )

    dynamics = Hopper2DDynamics(parameters)

    return HopperSimulator(
        dynamics=dynamics,
        integrator=EulerIntegrator(),
        initial_state=initial_state,
    )


def test_hover_remains_stationary_over_many_steps() -> None:
    initial_state = HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )

    simulator = create_simulator(initial_state)

    actuation = HopperActuation2D(
        thrust=MASS * GRAVITY,
        gimbal_angle=0.0,
    )

    dt = 0.01
    steps = 1000

    for _ in range(steps):
        simulator.step(
            actuation=actuation,
            dt=dt,
        )

    assert simulator.state.px == pytest.approx(0.0)
    assert simulator.state.pz == pytest.approx(2.0)
    assert simulator.state.vx == pytest.approx(0.0)
    assert simulator.state.vz == pytest.approx(0.0)
    assert simulator.state.theta == pytest.approx(0.0)
    assert simulator.state.omega == pytest.approx(0.0)

    assert simulator.time == pytest.approx(
        steps * dt
    )


def test_hover_remains_stationary_with_rk4() -> None:
    parameters = RigidBody2DParameters(
        mass=MASS,
        inertia_y=INERTIA_Y,
        engine_offset=ENGINE_OFFSET,
        gravity=GRAVITY,
    )

    dynamics = Hopper2DDynamics(parameters)

    initial_state = HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )

    simulator = HopperSimulator(
        dynamics=dynamics,
        integrator=RK4Integrator(),
        initial_state=initial_state,
    )

    actuation = HopperActuation2D(
        thrust=MASS * GRAVITY,
        gimbal_angle=0.0,
    )

    dt = 0.01

    for _ in range(1000):
        simulator.step(
            actuation=actuation,
            dt=dt,
        )

    assert simulator.state.px == pytest.approx(0.0)
    assert simulator.state.pz == pytest.approx(2.0)
    assert simulator.state.vx == pytest.approx(0.0)
    assert simulator.state.vz == pytest.approx(0.0)
    assert simulator.state.theta == pytest.approx(0.0)
    assert simulator.state.omega == pytest.approx(0.0)

    assert simulator.time == pytest.approx(10.0)


def test_free_fall_matches_discrete_euler_solution() -> None:
    initial_height = 10.0

    initial_state = HopperState2D(
        px=0.0,
        pz=initial_height,
        vx=0.0,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )

    simulator = create_simulator(initial_state)

    actuation = HopperActuation2D(
        thrust=0.0,
        gimbal_angle=0.0,
    )

    dt = 0.01
    steps = 100

    for _ in range(steps):
        simulator.step(
            actuation=actuation,
            dt=dt,
        )

    expected_vz = (
        -GRAVITY
        * steps
        * dt
    )

    expected_pz = (
        initial_height
        - 0.5
        * GRAVITY
        * dt**2
        * steps
        * (steps - 1)
    )

    assert simulator.state.vz == pytest.approx(
        expected_vz
    )

    assert simulator.state.pz == pytest.approx(
        expected_pz
    )

    assert simulator.state.px == pytest.approx(0.0)
    assert simulator.state.vx == pytest.approx(0.0)
    assert simulator.state.theta == pytest.approx(0.0)
    assert simulator.state.omega == pytest.approx(0.0)


def test_tilted_hopper_without_gimbal_does_not_rotate() -> None:
    initial_theta = math.radians(10.0)

    initial_state = HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=initial_theta,
        omega=0.0,
    )

    simulator = create_simulator(initial_state)

    actuation = HopperActuation2D(
        thrust=MASS * GRAVITY,
        gimbal_angle=0.0,
    )

    dt = 0.01
    steps = 100

    for _ in range(steps):
        simulator.step(
            actuation=actuation,
            dt=dt,
        )

    assert simulator.state.theta == pytest.approx(
        initial_theta
    )

    assert simulator.state.omega == pytest.approx(
        0.0
    )

    assert simulator.state.vx > 0.0
    assert simulator.state.px > 0.0

    assert simulator.state.vz < 0.0
    assert simulator.state.pz < 2.0


def test_first_gimbal_step_matches_hand_calculation() -> None:
    initial_state = HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )

    simulator = create_simulator(initial_state)

    actuation = HopperActuation2D(
        thrust=MASS * GRAVITY,
        gimbal_angle=math.radians(5.0),
    )

    state = simulator.step(
        actuation=actuation,
        dt=0.01,
    )

    assert state.px == pytest.approx(0.0)
    assert state.pz == pytest.approx(2.0)

    assert state.vx == pytest.approx(
        0.008549978363545267
    )

    assert state.vz == pytest.approx(
        -0.0003733001171997685
    )

    assert state.theta == pytest.approx(0.0)

    assert state.omega == pytest.approx(
        -0.053437364772157915
    )


def test_five_gimbal_steps_match_reference_trajectory() -> None:
    initial_state = HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )

    simulator = create_simulator(initial_state)

    actuation = HopperActuation2D(
        thrust=MASS * GRAVITY,
        gimbal_angle=math.radians(5.0),
    )

    dt = 0.01

    for _ in range(5):
        simulator.step(
            actuation=actuation,
            dt=dt,
        )

    state = simulator.state

    assert simulator.time == pytest.approx(0.05)

    assert state.px == pytest.approx(
        0.000852386574138519
    )

    assert state.pz == pytest.approx(
        1.9999628968975236
    )

    assert state.vx == pytest.approx(
        0.04222761053895607
    )

    assert state.vz == pytest.approx(
        -0.0018214536537752935
    )

    assert state.theta == pytest.approx(
        -0.005343736477215791
    )

    assert state.omega == pytest.approx(
        -0.26718682386078957
    )


def test_rk4_gimbal_response_has_expected_directions() -> None:
    parameters = RigidBody2DParameters(
        mass=MASS,
        inertia_y=INERTIA_Y,
        engine_offset=ENGINE_OFFSET,
        gravity=GRAVITY,
    )

    simulator = HopperSimulator(
        dynamics=Hopper2DDynamics(parameters),
        integrator=RK4Integrator(),
        initial_state=HopperState2D(
            px=0.0,
            pz=2.0,
            vx=0.0,
            vz=0.0,
            theta=0.0,
            omega=0.0,
        ),
    )

    actuation = HopperActuation2D(
        thrust=MASS * GRAVITY,
        gimbal_angle=math.radians(5.0),
    )

    simulator.step(
        actuation=actuation,
        dt=0.01,
    )

    state = simulator.state

    assert state.px > 0.0
    assert state.vx > 0.0

    assert state.pz < 2.0
    assert state.vz < 0.0

    assert state.theta < 0.0
    assert state.omega < 0.0


