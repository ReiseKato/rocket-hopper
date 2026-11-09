import math

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


MASS = 2.0
INERTIA_Y = 0.08
ENGINE_OFFSET = 0.25
GRAVITY = 9.81
HOVER_THRUST = MASS * GRAVITY


def create_system() -> HopperSystem2D:
    rigid_body_parameters = RigidBody2DParameters(
        mass=MASS,
        inertia_y=INERTIA_Y,
        engine_offset=ENGINE_OFFSET,
        gravity=GRAVITY,
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
            theta=math.radians(5.0),
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
                max_angle=math.radians(10.0),
                max_rate=math.radians(100.0),
            )
        ),
    )

    return HopperSystem2D(
        actuator_system=actuator_system,
        simulator=simulator,
    )


def test_attitude_and_altitude_controllers_stabilize_height() -> None:
    system = create_system()
    runner = SimulationRunner(system)

    attitude_controller = PDAttitudeController2D(
        design_pd_attitude_gains(
            inertia_y=INERTIA_Y,
            engine_offset=ENGINE_OFFSET,
            nominal_thrust=HOVER_THRUST,
            natural_frequency=4.0,
            damping_ratio=0.8,
        )
    )

    altitude_controller = PDAltitudeController2D(
        parameters=design_pd_altitude_gains(
            mass=MASS,
            natural_frequency=2.0,
            damping_ratio=0.9,
        ),
        hover_thrust=HOVER_THRUST,
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

    result = runner.run(
        step_count=500,
        dt=0.01,
        command_provider=command_provider,
    )

    final_state = result.final_rigid_body_state

    assert abs(
        final_state.pz - altitude_reference.pz
    ) < 1e-3

    assert abs(final_state.vz) < 1e-3

    assert abs(final_state.theta) < math.radians(
        0.01
    )

    assert abs(final_state.omega) < math.radians(
        0.05
    )

    # Horizontal motion remains intentionally uncontrolled.
    assert abs(final_state.vx) > 0.0
