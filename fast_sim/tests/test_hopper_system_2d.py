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
from fast_sim.integration import RK4Integrator
from fast_sim.models import (
    HopperActuationCommand2D,
    HopperState2D,
    RigidBody2DParameters,
)
from fast_sim.simulation import HopperSimulator
from fast_sim.system import HopperSystem2D


def create_system() -> HopperSystem2D:
    rigid_body_parameters = RigidBody2DParameters(
        mass=2.0,
        inertia_y=0.08,
        engine_offset=0.25,
    )

    simulator = HopperSimulator(
        dynamics=Hopper2DDynamics(
            rigid_body_parameters
        ),
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

    actuator_system = HopperActuatorSystem2D(
        thrust_actuator=LimitedThrustActuator(
            ThrustActuatorParameters(
                min_thrust=0.0,
                max_thrust=25.0,
            )
        ),
        gimbal_actuator=RateLimitedGimbalActuator(
            GimbalActuatorParameters(
                max_angle=math.radians(10.0),
                max_rate=math.radians(100.0),
            )
        ),
    )

    return HopperSystem2D(
        actuator_system=actuator_system,
        simulator=simulator,
    )


def test_system_applies_actuator_limits_before_dynamics() -> None:
    system = create_system()

    result = system.step(
        command=HopperActuationCommand2D(
            thrust=30.0,
            gimbal_angle=math.radians(10.0),
        ),
        dt=0.01,
    )

    assert result.actuation.thrust == pytest.approx(
        25.0
    )

    assert result.actuation.gimbal_angle == pytest.approx(
        math.radians(1.0)
    )

    assert system.time == pytest.approx(0.01)

    assert (
        result.rigid_body_state
        == system.rigid_body_state
    )