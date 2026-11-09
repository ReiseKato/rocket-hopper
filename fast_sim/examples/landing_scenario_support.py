"""Shared assembly helpers for the three landing end-to-end examples."""

from collections.abc import Callable
from dataclasses import dataclass
from math import degrees, radians

from fast_sim.actuators import (
    GimbalActuatorParameters,
    HopperActuatorSystem2D,
    LimitedThrustActuator,
    RateLimitedGimbalActuator,
    ThrustActuatorParameters,
)
from fast_sim.control import (
    AltitudeReference2D,
    HorizontalPositionReference2D,
    PDAltitudeController2D,
    PDAttitudeController2D,
    PDHorizontalPositionController2D,
    design_pd_altitude_gains,
    design_pd_attitude_gains,
    design_pd_horizontal_position_gains,
)
from fast_sim.dynamics import Hopper2DDynamics
from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.integration import RK4Integrator
from fast_sim.landing import (
    FirstContactImpactTransition2D,
    IntegratedSingleContactStepper2D,
    LandingEvaluation2D,
    LandingEvaluator2D,
    LandingGuidanceParameters2D,
    LandingOrchestrator2D,
    LandingRunResult2D,
    LandingSuccessCriteria2D,
    PiecewiseLandingGuidance2D,
    SimultaneousContactImpactTransition2D,
    SimultaneousLandingEvaluation2D,
    SingleContactDynamics2D,
    TouchdownDetector2D,
)
from fast_sim.models import (
    HopperActuationCommand2D,
    HopperState2D,
    RigidBody2DParameters,
)
from fast_sim.simulation import (
    HopperSimulator,
    SimulationResult,
    SimulationRunner,
)
from fast_sim.system import HopperSystem2D


DT = 0.01

MASS = 2.0
INERTIA_Y = 0.08
ENGINE_OFFSET = 0.25
GRAVITY = 9.81

MAX_THRUST = 30.0
MAX_GIMBAL_ANGLE = radians(12.0)
MAX_GIMBAL_RATE = radians(120.0)

FOOT_SPACING = 0.6
CG_TO_FEET = 0.8
GROUND_HEIGHT = 0.0

ATTITUDE_NATURAL_FREQUENCY = 5.0
ATTITUDE_DAMPING_RATIO = 0.85

ALTITUDE_NATURAL_FREQUENCY = 2.0
ALTITUDE_DAMPING_RATIO = 0.9

HORIZONTAL_NATURAL_FREQUENCY = 0.9
HORIZONTAL_DAMPING_RATIO = 0.9
MAX_HORIZONTAL_ATTITUDE = radians(12.0)


@dataclass(frozen=True, slots=True)
class LandingScenarioComponents2D:
    """Real simulator components shared by one complete scenario run."""

    parameters: RigidBody2DParameters
    geometry: LandingGearGeometry2D

    actuator_system: HopperActuatorSystem2D
    system: HopperSystem2D

    horizontal_controller: (
        PDHorizontalPositionController2D
    )
    attitude_controller: PDAttitudeController2D
    altitude_controller: PDAltitudeController2D


def build_landing_scenario_components(
    initial_state: HopperState2D,
) -> LandingScenarioComponents2D:
    """Construct the real plant, actuators, controllers, and simulator."""
    parameters = RigidBody2DParameters(
        mass=MASS,
        inertia_y=INERTIA_Y,
        engine_offset=ENGINE_OFFSET,
        gravity=GRAVITY,
    )

    geometry = LandingGearGeometry2D(
        foot_spacing=FOOT_SPACING,
        cg_to_feet=CG_TO_FEET,
    )

    simulator = HopperSimulator(
        dynamics=Hopper2DDynamics(
            parameters
        ),
        integrator=RK4Integrator(),
        initial_state=initial_state,
    )

    actuator_system = HopperActuatorSystem2D(
        thrust_actuator=LimitedThrustActuator(
            ThrustActuatorParameters(
                min_thrust=0.0,
                max_thrust=MAX_THRUST,
            )
        ),
        gimbal_actuator=RateLimitedGimbalActuator(
            GimbalActuatorParameters(
                max_angle=MAX_GIMBAL_ANGLE,
                max_rate=MAX_GIMBAL_RATE,
            )
        ),
    )

    system = HopperSystem2D(
        actuator_system=actuator_system,
        simulator=simulator,
    )

    hover_thrust = (
        parameters.mass
        * parameters.gravity
    )

    attitude_controller = (
        PDAttitudeController2D(
            design_pd_attitude_gains(
                inertia_y=parameters.inertia_y,
                engine_offset=(
                    parameters.engine_offset
                ),
                nominal_thrust=hover_thrust,
                natural_frequency=(
                    ATTITUDE_NATURAL_FREQUENCY
                ),
                damping_ratio=(
                    ATTITUDE_DAMPING_RATIO
                ),
            )
        )
    )

    altitude_controller = (
        PDAltitudeController2D(
            parameters=design_pd_altitude_gains(
                mass=parameters.mass,
                natural_frequency=(
                    ALTITUDE_NATURAL_FREQUENCY
                ),
                damping_ratio=(
                    ALTITUDE_DAMPING_RATIO
                ),
            ),
            hover_thrust=hover_thrust,
        )
    )

    horizontal_controller = (
        PDHorizontalPositionController2D(
            parameters=(
                design_pd_horizontal_position_gains(
                    natural_frequency=(
                        HORIZONTAL_NATURAL_FREQUENCY
                    ),
                    damping_ratio=(
                        HORIZONTAL_DAMPING_RATIO
                    ),
                )
            ),
            gravity=parameters.gravity,
            max_attitude_angle=(
                MAX_HORIZONTAL_ATTITUDE
            ),
        )
    )

    return LandingScenarioComponents2D(
        parameters=parameters,
        geometry=geometry,
        actuator_system=actuator_system,
        system=system,
        horizontal_controller=(
            horizontal_controller
        ),
        attitude_controller=(
            attitude_controller
        ),
        altitude_controller=(
            altitude_controller
        ),
    )


def create_position_command_provider(
    components: LandingScenarioComponents2D,
    *,
    target_x: float,
    target_z: float,
):
    """Create a real cascaded position/altitude command provider."""
    horizontal_reference = (
        HorizontalPositionReference2D(
            px=target_x,
            vx=0.0,
        )
    )

    altitude_reference = (
        AltitudeReference2D(
            pz=target_z,
            vz=0.0,
        )
    )

    def command_provider(
        time: float,
        state: HopperState2D,
    ) -> HopperActuationCommand2D:
        attitude_reference = (
            components
            .horizontal_controller
            .compute_attitude_reference(
                state=state,
                reference=horizontal_reference,
            )
        )

        gimbal_command = (
            components
            .attitude_controller
            .compute_gimbal_command(
                state=state,
                reference=attitude_reference,
            )
        )

        thrust_command = (
            components
            .altitude_controller
            .compute_thrust_command(
                state=state,
                reference=altitude_reference,
            )
        )

        return HopperActuationCommand2D(
            thrust=thrust_command,
            gimbal_angle=gimbal_command,
        )

    return command_provider


def run_until_settled(
    components: LandingScenarioComponents2D,
    *,
    target_x: float,
    target_z: float,
    max_duration: float,
    check_interval: float = 0.25,
    required_settle_time: float = 0.5,
    max_position_error: float = 0.04,
    max_altitude_error: float = 0.04,
    max_horizontal_speed: float = 0.05,
    max_vertical_speed: float = 0.05,
    max_attitude_angle: float = radians(1.0),
    max_angular_speed: float = radians(3.0),
) -> SimulationResult:
    """Run until the vehicle remains settled around a target for a dwell."""
    command_provider = (
        create_position_command_provider(
            components,
            target_x=target_x,
            target_z=target_z,
        )
    )

    runner = SimulationRunner(
        components.system
    )

    check_step_count = round(
        check_interval / DT
    )

    required_stable_checks = round(
        required_settle_time
        / check_interval
    )

    maximum_checks = round(
        max_duration
        / check_interval
    )

    chunks: list[SimulationResult] = []
    stable_check_count = 0

    for _ in range(maximum_checks):
        chunk = runner.run(
            step_count=check_step_count,
            dt=DT,
            command_provider=command_provider,
        )

        chunks.append(chunk)

        state = (
            chunk.final_rigid_body_state
        )

        is_settled = (
            abs(state.px - target_x)
            <= max_position_error
            and abs(state.pz - target_z)
            <= max_altitude_error
            and abs(state.vx)
            <= max_horizontal_speed
            and abs(state.vz)
            <= max_vertical_speed
            and abs(state.theta)
            <= max_attitude_angle
            and abs(state.omega)
            <= max_angular_speed
        )

        if is_settled:
            stable_check_count += 1
        else:
            stable_check_count = 0

        if (
            stable_check_count
            >= required_stable_checks
        ):
            return concatenate_simulation_results(
                chunks
            )

    raise RuntimeError(
        "vehicle did not settle at the requested "
        "pre-landing position"
    )


def run_until_position_reached(
    components: LandingScenarioComponents2D,
    *,
    target_x: float,
    target_z: float,
    max_duration: float,
    position_tolerance: float,
    altitude_tolerance: float,
    check_interval: float = 0.10,
) -> SimulationResult:
    """Stop at the first positional reach without a velocity dwell.

    This intentionally differs from `run_until_settled`: the landing
    handoff happens as soon as the requested position region is first
    entered. Non-zero translational and angular motion is allowed.
    """
    command_provider = (
        create_position_command_provider(
            components,
            target_x=target_x,
            target_z=target_z,
        )
    )

    runner = SimulationRunner(
        components.system
    )

    check_step_count = round(
        check_interval / DT
    )

    maximum_checks = round(
        max_duration
        / check_interval
    )

    chunks: list[SimulationResult] = []

    for _ in range(maximum_checks):
        chunk = runner.run(
            step_count=check_step_count,
            dt=DT,
            command_provider=command_provider,
        )

        chunks.append(chunk)

        state = (
            chunk.final_rigid_body_state
        )

        if (
            abs(state.px - target_x)
            <= position_tolerance
            and abs(state.pz - target_z)
            <= altitude_tolerance
        ):
            return concatenate_simulation_results(
                chunks
            )

    raise RuntimeError(
        "vehicle did not reach the requested "
        "pre-landing position"
    )


def run_landing(
    components: LandingScenarioComponents2D,
    *,
    landing_target_x: float,
    approach_descent_speed: float,
    flare_height: float,
    touchdown_descent_speed: float,
    max_step_count: int,
) -> LandingRunResult2D:
    """Execute the real landing guidance, contact, and evaluation stack."""
    guidance = PiecewiseLandingGuidance2D(
        LandingGuidanceParameters2D(
            target_x=landing_target_x,
            ground_height=GROUND_HEIGHT,
            approach_descent_speed=(
                approach_descent_speed
            ),
            flare_height=flare_height,
            touchdown_descent_speed=(
                touchdown_descent_speed
            ),
        ),
        geometry=components.geometry,
        initial_state=(
            components.system.rigid_body_state
        ),
        start_time=components.system.time,
    )

    evaluator = LandingEvaluator2D(
        components.parameters,
        components.geometry,
        LandingSuccessCriteria2D(
            max_horizontal_position_error=0.20,
            max_first_contact_normal_speed=0.60,
            max_first_contact_tangential_speed=0.40,
            max_second_contact_normal_speed=0.60,
            max_second_contact_tangential_speed=0.40,
            max_attitude_angle=radians(10.0),
            max_angular_speed=radians(45.0),
            max_simultaneous_contact_normal_speed=0.60,
            max_simultaneous_contact_tangential_speed=0.40,
        ),
    )

    single_contact_dynamics = (
        SingleContactDynamics2D(
            components.parameters,
            components.geometry,
        )
    )

    orchestrator = LandingOrchestrator2D(
        system=components.system,
        # The same stateful actuator system continues after first contact.
        physical_actuation_provider=(
            components.actuator_system.step
        ),
        guidance=guidance,
        horizontal_controller=(
            components.horizontal_controller
        ),
        attitude_controller=(
            components.attitude_controller
        ),
        altitude_controller=(
            components.altitude_controller
        ),
        touchdown_detector=TouchdownDetector2D(
            components.geometry,
            ground_height=GROUND_HEIGHT,
        ),
        first_contact_impact=(
            FirstContactImpactTransition2D(
                components.parameters,
                components.geometry,
            )
        ),
        simultaneous_contact_impact=(
            SimultaneousContactImpactTransition2D(
                components.parameters,
                components.geometry,
                max_foot_height_difference=1e-6,
            )
        ),
        single_contact_stepper=(
            IntegratedSingleContactStepper2D(
                single_contact_dynamics,
                RK4Integrator(),
            )
        ),
        single_contact_dynamics=(
            single_contact_dynamics
        ),
        evaluator=evaluator,
    )

    return orchestrator.run(
        max_step_count=max_step_count,
        dt=DT,
    )


def concatenate_simulation_results(
    results: list[SimulationResult],
) -> SimulationResult:
    """Join consecutive SimulationResult chunks without duplicate samples."""
    if not results:
        raise ValueError(
            "at least one SimulationResult is required"
        )

    times = list(results[0].times)
    states = list(
        results[0].rigid_body_states
    )
    commands = list(
        results[0].commands
    )
    actuations = list(
        results[0].physical_actuations
    )

    for result in results[1:]:
        times.extend(
            result.times[1:]
        )
        states.extend(
            result.rigid_body_states[1:]
        )
        commands.extend(
            result.commands
        )
        actuations.extend(
            result.physical_actuations
        )

    return SimulationResult(
        times=tuple(times),
        rigid_body_states=tuple(states),
        commands=tuple(commands),
        physical_actuations=tuple(actuations),
    )


def print_landing_summary(
    *,
    scenario_name: str,
    landing_result: LandingRunResult2D,
    pre_landing_result: SimulationResult | None = None,
) -> None:
    """Print the scenario handoff and both impact metrics."""
    print()
    print(scenario_name)

    if pre_landing_result is not None:
        state = (
            pre_landing_result
            .final_rigid_body_state
        )

        print("Pre-landing handoff")
        print(
            "  time [s]:",
            pre_landing_result.end_time,
        )
        print("  px [m]:", state.px)
        print("  pz [m]:", state.pz)
        print("  vx [m/s]:", state.vx)
        print("  vz [m/s]:", state.vz)
        print(
            "  theta [deg]:",
            degrees(state.theta),
        )
        print(
            "  omega [deg/s]:",
            degrees(state.omega),
        )

    print("Landing")
    print(
        "  completed:",
        landing_result.completed,
    )
    print(
        "  successful:",
        landing_result.successful,
    )
    print(
        "  contact sequence:",
        (
            landing_result
            .contact_sequence
            .value
            if landing_result
            .contact_sequence
            is not None
            else "none"
        ),
    )

    evaluation = (
        landing_result.evaluation
    )

    if evaluation is None:
        print("  evaluation: none")
        return

    if isinstance(
        evaluation,
        LandingEvaluation2D,
    ):
        print("  first contact")
        print(
            "    foot:",
            evaluation
            .first_contact
            .foot
            .value,
        )
        print(
            "    normal speed [m/s]:",
            evaluation
            .first_contact
            .normal_speed,
        )
        print(
            "    tangential speed [m/s]:",
            evaluation
            .first_contact
            .tangential_speed,
        )
        print(
            "    dissipated energy [J]:",
            evaluation
            .first_contact
            .dissipated_energy,
        )

        print("  second contact")
        print(
            "    foot:",
            evaluation
            .second_contact
            .foot
            .value,
        )
        print(
            "    normal speed [m/s]:",
            evaluation
            .second_contact
            .normal_speed,
        )
        print(
            "    tangential speed [m/s]:",
            evaluation
            .second_contact
            .tangential_speed,
        )
        print(
            "    dissipated energy [J]:",
            evaluation
            .second_contact
            .dissipated_energy,
        )

    elif isinstance(
        evaluation,
        SimultaneousLandingEvaluation2D,
    ):
        print("  simultaneous contact")
        print(
            "    left normal speed [m/s]:",
            evaluation
            .contact
            .left_normal_speed,
        )
        print(
            "    left tangential speed [m/s]:",
            evaluation
            .contact
            .left_tangential_speed,
        )
        print(
            "    right normal speed [m/s]:",
            evaluation
            .contact
            .right_normal_speed,
        )
        print(
            "    right tangential speed [m/s]:",
            evaluation
            .contact
            .right_tangential_speed,
        )
        print(
            "    dissipated vehicle energy [J]:",
            evaluation
            .contact
            .dissipated_energy,
        )

    print(
        "  total dissipated impact energy [J]:",
        evaluation.total_dissipated_energy,
    )

    if evaluation.failed_criteria:
        print("  failed criteria:")
        for reason in (
            evaluation.failed_criteria
        ):
            print(
                "   -",
                reason.value,
            )
