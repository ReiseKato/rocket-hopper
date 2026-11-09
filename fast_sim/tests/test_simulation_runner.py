import math

import pytest

from fast_sim.actuators import (
    GimbalActuatorParameters,
    HopperActuatorSystem2D,
    LimitedThrustActuator,
    RateLimitedGimbalActuator,
    ThrustActuatorParameters,
)
from fast_sim.dynamics import Hopper2DDynamics
from fast_sim.integration import (
    EulerIntegrator,
    Integrator,
    RK4Integrator,
)
from fast_sim.models import (
    HopperActuationCommand2D,
    HopperState2D,
    RigidBody2DParameters,
)
from fast_sim.simulation import (
    HopperSimulator,
    SimulationRunner,
)
from fast_sim.system import HopperSystem2D


MASS = 2.0
INERTIA_Y = 0.08
ENGINE_OFFSET = 0.25
GRAVITY = 9.81

MAX_THRUST = 30.0
MAX_GIMBAL_ANGLE = math.radians(10.0)
MAX_GIMBAL_RATE = math.radians(100.0)


def create_system(
    integrator: Integrator,
) -> HopperSystem2D:
    parameters = RigidBody2DParameters(
        mass=MASS,
        inertia_y=INERTIA_Y,
        engine_offset=ENGINE_OFFSET,
        gravity=GRAVITY,
    )

    simulator = HopperSimulator(
        dynamics=Hopper2DDynamics(parameters),
        integrator=integrator,
        initial_state=HopperState2D(
            px=0.0,
            pz=2.0,
            vx=0.0,
            vz=0.0,
            theta=0.0,
            omega=0.0,
        ),
    )

    actuator_system = HopperActuatorSystem2D(
        thrust_actuator=LimitedThrustActuator(
            ThrustActuatorParameters(
                min_thrust=0.0,
                max_thrust=MAX_THRUST,
            ),
        ),
        gimbal_actuator=RateLimitedGimbalActuator(
            GimbalActuatorParameters(
                max_angle=MAX_GIMBAL_ANGLE,
                max_rate=MAX_GIMBAL_RATE,
            ),
        ),
    )

    return HopperSystem2D(
        actuator_system=actuator_system,
        simulator=simulator,
    )


@pytest.mark.parametrize(
    "integrator",
    [
        EulerIntegrator(),
        RK4Integrator(),
    ],
)
def test_runner_executes_hover_run(
    integrator: Integrator,
) -> None:
    system = create_system(integrator)
    runner = SimulationRunner(system)

    hover_command = HopperActuationCommand2D(
        thrust=MASS * GRAVITY,
        gimbal_angle=0.0,
    )

    result = runner.run(
        step_count=100,
        dt=0.01,
        command_provider=lambda time, state: hover_command,
    )

    assert result.step_count == 100

    assert len(result.times) == 101
    assert len(result.rigid_body_states) == 101
    assert len(result.commands) == 100
    assert len(result.physical_actuations) == 100

    assert result.duration == pytest.approx(1.0)

    final_state = result.final_rigid_body_state

    assert final_state.px == pytest.approx(0.0)
    assert final_state.pz == pytest.approx(2.0)
    assert final_state.vx == pytest.approx(0.0)
    assert final_state.vz == pytest.approx(0.0)
    assert final_state.theta == pytest.approx(0.0)
    assert final_state.omega == pytest.approx(0.0)


def test_runner_records_command_and_limited_actuation() -> None:
    system = create_system(RK4Integrator())
    runner = SimulationRunner(system)

    command = HopperActuationCommand2D(
        thrust=40.0,
        gimbal_angle=math.radians(10.0),
    )

    result = runner.run(
        step_count=1,
        dt=0.01,
        command_provider=lambda time, state: command,
    )

    assert result.commands[0].thrust == pytest.approx(40.0)
    assert result.commands[0].gimbal_angle == pytest.approx(
        math.radians(10.0)
    )

    assert result.physical_actuations[0].thrust == pytest.approx(
        MAX_THRUST
    )
    assert result.physical_actuations[0].gimbal_angle == pytest.approx(
        math.radians(1.0)
    )


def test_command_provider_receives_current_time_and_state() -> None:
    system = create_system(EulerIntegrator())
    runner = SimulationRunner(system)

    observations: list[tuple[float, float]] = []

    def command_provider(
        time: float,
        state: HopperState2D,
    ) -> HopperActuationCommand2D:
        observations.append(
            (time, state.vz)
        )

        return HopperActuationCommand2D(
            thrust=0.0,
            gimbal_angle=0.0,
        )

    runner.run(
        step_count=2,
        dt=0.1,
        command_provider=command_provider,
    )

    assert observations[0][0] == pytest.approx(0.0)
    assert observations[0][1] == pytest.approx(0.0)

    assert observations[1][0] == pytest.approx(0.1)
    assert observations[1][1] == pytest.approx(-0.981)


def test_runner_continues_from_current_system_state() -> None:
    system = create_system(EulerIntegrator())
    runner = SimulationRunner(system)

    def command_provider(
        time: float,
        state: HopperState2D,
    ) -> HopperActuationCommand2D:
        return HopperActuationCommand2D(
            thrust=0.0,
            gimbal_angle=0.0,
        )

    first_result = runner.run(
        step_count=2,
        dt=0.1,
        command_provider=command_provider,
    )

    second_result = runner.run(
        step_count=3,
        dt=0.1,
        command_provider=command_provider,
    )

    assert first_result.start_time == pytest.approx(0.0)
    assert first_result.end_time == pytest.approx(0.2)

    assert second_result.start_time == pytest.approx(0.2)
    assert second_result.end_time == pytest.approx(0.5)

    assert (
        second_result.initial_rigid_body_state
        == first_result.final_rigid_body_state
    )


def test_runner_preserves_actuator_state_between_runs() -> None:
    system = create_system(RK4Integrator())
    runner = SimulationRunner(system)

    command = HopperActuationCommand2D(
        thrust=MASS * GRAVITY,
        gimbal_angle=math.radians(10.0),
    )

    first_result = runner.run(
        step_count=2,
        dt=0.01,
        command_provider=lambda time, state: command,
    )

    second_result = runner.run(
        step_count=1,
        dt=0.01,
        command_provider=lambda time, state: command,
    )

    assert (
        first_result.physical_actuations[-1].gimbal_angle
        == pytest.approx(math.radians(2.0))
    )

    assert (
        second_result.physical_actuations[0].gimbal_angle
        == pytest.approx(math.radians(3.0))
    )


def test_runner_rejects_non_command_provider_result() -> None:
    system = create_system(EulerIntegrator())
    runner = SimulationRunner(system)

    with pytest.raises(
        TypeError,
        match="command_provider must return HopperActuationCommand2D",
    ):
        runner.run(
            step_count=1,
            dt=0.01,
            command_provider=lambda time, state: None,  # type: ignore[return-value]
        )


@pytest.mark.parametrize(
    "step_count",
    [
        0,
        -1,
        -100,
    ],
)
def test_runner_rejects_non_positive_step_count(
    step_count: int,
) -> None:
    runner = SimulationRunner(
        create_system(EulerIntegrator())
    )

    with pytest.raises(
        ValueError,
        match="step_count must be > 0",
    ):
        runner.run(
            step_count=step_count,
            dt=0.01,
            command_provider=lambda time, state: (
                HopperActuationCommand2D(
                    thrust=0.0,
                    gimbal_angle=0.0,
                )
            ),
        )


@pytest.mark.parametrize(
    "step_count",
    [
        1.5,
        float("inf"),
        float("-inf"),
        float("nan"),
        "10",
        None,
        True,
    ],
)
def test_runner_rejects_non_integer_step_count(
    step_count,
) -> None:
    runner = SimulationRunner(
        create_system(EulerIntegrator())
    )

    with pytest.raises(
        TypeError,
        match="step_count must be an integer",
    ):
        runner.run(
            step_count=step_count,
            dt=0.01,
            command_provider=lambda time, state: (
                HopperActuationCommand2D(
                    thrust=0.0,
                    gimbal_angle=0.0,
                )
            ),
        )


@pytest.mark.parametrize(
    "dt",
    [
        0.0,
        -0.01,
        float("inf"),
        float("-inf"),
        float("nan"),
    ],
)
def test_runner_rejects_invalid_dt(
    dt: float,
) -> None:
    runner = SimulationRunner(
        create_system(EulerIntegrator())
    )

    with pytest.raises(ValueError):
        runner.run(
            step_count=1,
            dt=dt,
            command_provider=lambda time, state: (
                HopperActuationCommand2D(
                    thrust=0.0,
                    gimbal_angle=0.0,
                )
            ),
        )
