"""Stabilize attitude and altitude with two independent PD loops."""

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
    AltitudeReference2D,
    AttitudeReference2D,
    PDAltitudeController2D,
    PDAttitudeController2D,
    design_pd_altitude_gains,
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

ATTITUDE_NATURAL_FREQUENCY = 4.0
ATTITUDE_DAMPING_RATIO = 0.8

ALTITUDE_NATURAL_FREQUENCY = 2.0
ALTITUDE_DAMPING_RATIO = 0.9


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

    attitude_controller_parameters = (
        design_pd_attitude_gains(
            inertia_y=parameters.inertia_y,
            engine_offset=parameters.engine_offset,
            nominal_thrust=hover_thrust,
            natural_frequency=(
                ATTITUDE_NATURAL_FREQUENCY
            ),
            damping_ratio=(
                ATTITUDE_DAMPING_RATIO
            ),
        )
    )

    altitude_controller_parameters = (
        design_pd_altitude_gains(
            mass=parameters.mass,
            natural_frequency=(
                ALTITUDE_NATURAL_FREQUENCY
            ),
            damping_ratio=(
                ALTITUDE_DAMPING_RATIO
            ),
        )
    )

    attitude_controller = PDAttitudeController2D(
        attitude_controller_parameters
    )

    altitude_controller = PDAltitudeController2D(
        parameters=altitude_controller_parameters,
        hover_thrust=hover_thrust,
    )

    attitude_reference = AttitudeReference2D(
        theta=0.0,
        omega=0.0,
    )

    altitude_reference = AltitudeReference2D(
        pz=2.0,
        vz=0.0,
    )

    def command_provider(
        time: float,
        state: HopperState2D,
    ) -> HopperActuationCommand2D:
        thrust_command = (
            altitude_controller.compute_thrust_command(
                state=state,
                reference=altitude_reference,
            )
        )

        gimbal_command = (
            attitude_controller.compute_gimbal_command(
                state=state,
                reference=attitude_reference,
            )
        )

        return HopperActuationCommand2D(
            thrust=thrust_command,
            gimbal_angle=gimbal_command,
        )

    runner = SimulationRunner(system)

    result = runner.run(
        step_count=1000,
        dt=0.01,
        command_provider=command_provider,
    )

    final_state = result.final_rigid_body_state

    print(
        "Attitude Kp:",
        attitude_controller_parameters.proportional_gain,
    )
    print(
        "Attitude Kd:",
        attitude_controller_parameters.derivative_gain,
    )
    print(
        "Altitude Kp:",
        altitude_controller_parameters.proportional_gain,
    )
    print(
        "Altitude Kd:",
        altitude_controller_parameters.derivative_gain,
    )
    print(
        "Final px [m]:",
        final_state.px,
    )
    print(
        "Final pz [m]:",
        final_state.pz,
    )
    print(
        "Final vx [m/s]:",
        final_state.vx,
    )
    print(
        "Final vz [m/s]:",
        final_state.vz,
    )
    print(
        "Final theta [deg]:",
        degrees(final_state.theta),
    )
    print(
        "Final omega [deg/s]:",
        degrees(final_state.omega),
    )

    plot_simulation_overview(result)

    plt.show()


if __name__ == "__main__":
    main()
