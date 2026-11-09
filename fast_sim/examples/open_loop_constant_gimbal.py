"""Run a simple open-loop hopper simulation and plot the result."""

from math import radians

import matplotlib.pyplot as plt

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
from fast_sim.simulation import (
    HopperSimulator,
    SimulationRunner,
)
from fast_sim.system import HopperSystem2D
from fast_sim.visualization import (
    plot_basic_time_series,
    plot_actuation_comparisons,
)


def main() -> None:
    parameters = RigidBody2DParameters(
        mass=2.0,
        inertia_y=0.08,
        engine_offset=0.25,
        gravity=9.81,
    )

    initial_state = HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )

    simulator = HopperSimulator(
        dynamics=Hopper2DDynamics(parameters),
        integrator=RK4Integrator(),
        initial_state=initial_state,
    )

    actuator_system = HopperActuatorSystem2D(
        thrust_actuator=LimitedThrustActuator(
            ThrustActuatorParameters(
                min_thrust=0.0,
                max_thrust=30.0,
            ),
        ),
        gimbal_actuator=RateLimitedGimbalActuator(
            GimbalActuatorParameters(
                max_angle=radians(10.0),
                max_rate=radians(100.0),
            ),
        ),
    )

    system = HopperSystem2D(
        actuator_system=actuator_system,
        simulator=simulator,
    )

    runner = SimulationRunner(system)

    constant_command = HopperActuationCommand2D(
        thrust=parameters.mass * parameters.gravity,
        gimbal_angle=radians(5.0),
    )

    result = runner.run(
        step_count=500,
        dt=0.01,
        command_provider=lambda time, state: constant_command,
    )

    plot_basic_time_series(result)
    plot_actuation_comparisons(result)

    plt.show()


if __name__ == "__main__":
    main()
