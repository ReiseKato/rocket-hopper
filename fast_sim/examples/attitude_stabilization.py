"""Stabilize a small initial attitude error with a PD controller."""

from math import degrees, radians

import matplotlib.pyplot as plt

from fast_sim.actuators import (
    GimbalActuatorParameters,
    HopperActuatorSystem2D,
    LimitedThrustActuator,
    RateLimitedGimbalActuator,
    ThrustActuatorParameters,
)
from fast_sim.control import (
    AttitudeReference2D,
    PDAttitudeController2D,
    design_pd_attitude_gains,
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
    plot_simulation_overview,
)


MASS = 2.0
INERTIA_Y = 0.08
ENGINE_OFFSET = 0.25
GRAVITY = 9.81

NATURAL_FREQUENCY = 4.0
DAMPING_RATIO = 0.8


def main() -> None:
    parameters = RigidBody2DParameters(
        mass=MASS,
        inertia_y=INERTIA_Y,
        engine_offset=ENGINE_OFFSET,
        gravity=GRAVITY,
    )

    hover_thrust = (
        parameters.mass
        * parameters.gravity
    )

    simulator = HopperSimulator(
        dynamics=Hopper2DDynamics(parameters),
        integrator=RK4Integrator(),
        initial_state=HopperState2D(
            px=0.0,
            pz=2.0,
            vx=0.0,
            vz=0.0,
            theta=radians(5.0),
            omega=0.0,
        ),
    )

    actuator_system = HopperActuatorSystem2D(
        thrust_actuator=LimitedThrustActuator(
            ThrustActuatorParameters(
                min_thrust=0.0,
                max_thrust=30.0,
            )
        ),
        gimbal_actuator=RateLimitedGimbalActuator(
            GimbalActuatorParameters(
                max_angle=radians(10.0),
                max_rate=radians(100.0),
            )
        ),
    )

    system = HopperSystem2D(
        actuator_system=actuator_system,
        simulator=simulator,
    )

    controller_parameters = design_pd_attitude_gains(
        inertia_y=parameters.inertia_y,
        engine_offset=parameters.engine_offset,
        nominal_thrust=hover_thrust,
        natural_frequency=NATURAL_FREQUENCY,
        damping_ratio=DAMPING_RATIO,
    )

    controller = PDAttitudeController2D(
        controller_parameters
    )

    reference = AttitudeReference2D(
        theta=0.0,
        omega=0.0,
    )

    def command_provider(
        time: float,
        state: HopperState2D,
    ) -> HopperActuationCommand2D:
        gimbal_command = (
            controller.compute_gimbal_command(
                state=state,
                reference=reference,
            )
        )

        return HopperActuationCommand2D(
            thrust=hover_thrust,
            gimbal_angle=gimbal_command,
        )

    runner = SimulationRunner(system)

    result = runner.run(
        step_count=300,
        dt=0.01,
        command_provider=command_provider,
    )

    print(
        "Kp:",
        controller_parameters.proportional_gain,
    )
    print(
        "Kd:",
        controller_parameters.derivative_gain,
    )
    print(
        "Final theta [deg]:",
        degrees(
            result.final_rigid_body_state.theta
        ),
    )
    print(
        "Final omega [deg/s]:",
        degrees(
            result.final_rigid_body_state.omega
        ),
    )
    print(
        "Final vx [m/s]:",
        result.final_rigid_body_state.vx,
    )
    print(
        "Final vz [m/s]:",
        result.final_rigid_body_state.vz,
    )

    plot_simulation_overview(result)
    

    plt.show()


if __name__ == "__main__":
    main()
