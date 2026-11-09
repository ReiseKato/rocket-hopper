"""Orchestration of a complete 2D landing maneuver."""

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Protocol

import numpy as np

from fast_sim.control import (
    AltitudeController2D,
    AttitudeController2D,
    HorizontalPositionController2D,
)
from fast_sim.integration import Integrator
from fast_sim.landing.contact import (
    FirstContactImpactTransition2D,
    SimultaneousContactImpactTransition2D,
    SingleContactConstraint2D,
    SingleContactDynamics2D,
    SingleContactState2D,
)
from fast_sim.landing.evaluation import (
    LandingContactMetrics2D,
    LandingEvaluation2D,
    LandingEvaluator2D,
    SimultaneousLandingEvaluation2D,
)
from fast_sim.landing.guidance import (
    LandingGuidance2D,
    LandingGuidanceReference2D,
)
from fast_sim.landing.touchdown import (
    TouchdownContact2D,
    TouchdownDetection2D,
    TouchdownDetector2D,
)
from fast_sim.models import (
    HopperActuation2D,
    HopperActuationCommand2D,
    HopperState2D,
)
from fast_sim.system import HopperSystem2D


class LandingDynamicsMode2D(Enum):
    """Physical dynamics mode active during a landing run."""

    FREE_FLIGHT = "free_flight"
    SINGLE_CONTACT = "single_contact"
    COMPLETED = "completed"


class LandingContactSequence2D(Enum):
    """Geometric contact sequence that completed the landing."""

    SEQUENTIAL = "sequential"
    SIMULTANEOUS = "simultaneous"


class LandingTerminationReason2D(Enum):
    """Reason why a landing run ended."""

    COMPLETED = "completed"
    MAX_STEPS_REACHED = "max_steps_reached"


PhysicalActuationProvider2D = Callable[
    [HopperActuationCommand2D, float],
    HopperActuation2D,
]


class SingleContactStepper2D(Protocol):
    """Numerically advance the reduced single-contact state."""

    def step(
        self,
        *,
        time: float,
        state: SingleContactState2D,
        constraint: SingleContactConstraint2D,
        actuation: HopperActuation2D,
        dt: float,
    ) -> SingleContactState2D:
        """Return the reduced state after one time step."""


class IntegratedSingleContactStepper2D:
    """Connect `SingleContactDynamics2D` to the generic Integrator."""

    def __init__(
        self,
        dynamics: SingleContactDynamics2D,
        integrator: Integrator,
    ) -> None:
        self._dynamics = dynamics
        self._integrator = integrator

    @property
    def dynamics(self) -> SingleContactDynamics2D:
        """Return the constrained dynamics model."""
        return self._dynamics

    @property
    def integrator(self) -> Integrator:
        """Return the numerical integrator."""
        return self._integrator

    def step(
        self,
        *,
        time: float,
        state: SingleContactState2D,
        constraint: SingleContactConstraint2D,
        actuation: HopperActuation2D,
        dt: float,
    ) -> SingleContactState2D:
        """Advance theta and omega by one numerical step."""
        if not isfinite(time):
            raise ValueError("time must be finite")

        if not isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and > 0")

        vector = np.array(
            [state.theta, state.omega],
            dtype=float,
        )

        def derivative(
            derivative_time: float,
            derivative_vector: np.ndarray,
        ) -> np.ndarray:
            reduced_state = SingleContactState2D(
                theta=float(derivative_vector[0]),
                omega=float(derivative_vector[1]),
            )

            reduced_derivative = self._dynamics.derivative(
                reduced_state,
                constraint,
                actuation,
            )

            return np.array(
                [
                    reduced_derivative.theta_dot,
                    reduced_derivative.omega_dot,
                ],
                dtype=float,
            )

        next_vector = self._integrator.step(
            derivative,
            vector,
            time,
            dt,
        )

        return SingleContactState2D(
            theta=float(next_vector[0]),
            omega=float(next_vector[1]),
        )


LandingEvaluationResult2D = (
    LandingEvaluation2D
    | SimultaneousLandingEvaluation2D
)


@dataclass(frozen=True, slots=True)
class LandingRunResult2D:
    """Recorded result of one complete or truncated landing run."""

    times: tuple[float, ...]
    rigid_body_states: tuple[HopperState2D, ...]
    modes: tuple[LandingDynamicsMode2D, ...]

    guidance_references: tuple[
        LandingGuidanceReference2D,
        ...
    ]
    commands: tuple[HopperActuationCommand2D, ...]
    physical_actuations: tuple[HopperActuation2D, ...]

    first_contact: LandingContactMetrics2D | None
    evaluation: LandingEvaluationResult2D | None

    contact_sequence: LandingContactSequence2D | None
    termination_reason: LandingTerminationReason2D

    def __post_init__(self) -> None:
        sample_count = len(self.times)

        if sample_count == 0:
            raise ValueError(
                "landing result must contain at least one sample"
            )

        if len(self.rigid_body_states) != sample_count:
            raise ValueError(
                "rigid_body_states must match times"
            )

        if len(self.modes) != sample_count:
            raise ValueError(
                "modes must match times"
            )

        step_count = sample_count - 1

        if len(self.guidance_references) != step_count:
            raise ValueError(
                "guidance_references must match executed steps"
            )

        if len(self.commands) != step_count:
            raise ValueError(
                "commands must match executed steps"
            )

        if len(self.physical_actuations) != step_count:
            raise ValueError(
                "physical_actuations must match executed steps"
            )

        if (
            self.termination_reason
            is LandingTerminationReason2D.COMPLETED
        ):
            if self.evaluation is None:
                raise ValueError(
                    "completed landing must have an evaluation"
                )

            if self.contact_sequence is None:
                raise ValueError(
                    "completed landing must have a contact sequence"
                )

    @property
    def step_count(self) -> int:
        """Return the number of executed finite time steps."""
        return len(self.commands)

    @property
    def start_time(self) -> float:
        """Return the first recorded time."""
        return self.times[0]

    @property
    def end_time(self) -> float:
        """Return the final recorded time."""
        return self.times[-1]

    @property
    def duration(self) -> float:
        """Return the recorded run duration."""
        return self.end_time - self.start_time

    @property
    def initial_rigid_body_state(self) -> HopperState2D:
        """Return the initial rigid-body state."""
        return self.rigid_body_states[0]

    @property
    def final_rigid_body_state(self) -> HopperState2D:
        """Return the final rigid-body state."""
        return self.rigid_body_states[-1]

    @property
    def completed(self) -> bool:
        """Return whether both feet completed the landing."""
        return (
            self.termination_reason
            is LandingTerminationReason2D.COMPLETED
        )

    @property
    def successful(self) -> bool:
        """Return whether the completed landing passed its criteria."""
        return (
            self.evaluation is not None
            and self.evaluation.successful
        )


class LandingOrchestrator2D:
    """Coordinate guidance, control, plant modes, contact, and evaluation.

    The orchestrator is intentionally one-shot. After the first sequential
    contact, the free-flight `HopperSystem2D` no longer represents the
    constrained rigid-body state and is not resumed.

    `physical_actuation_provider` must advance the same actuator-model
    state that was used inside `HopperSystem2D` before first contact.
    """

    def __init__(
        self,
        *,
        system: HopperSystem2D,
        physical_actuation_provider: PhysicalActuationProvider2D,
        guidance: LandingGuidance2D,
        horizontal_controller: HorizontalPositionController2D,
        attitude_controller: AttitudeController2D,
        altitude_controller: AltitudeController2D,
        touchdown_detector: TouchdownDetector2D,
        first_contact_impact: FirstContactImpactTransition2D,
        simultaneous_contact_impact: (
            SimultaneousContactImpactTransition2D
        ),
        single_contact_stepper: SingleContactStepper2D,
        single_contact_dynamics: SingleContactDynamics2D,
        evaluator: LandingEvaluator2D,
    ) -> None:
        self._system = system
        self._physical_actuation_provider = (
            physical_actuation_provider
        )
        self._guidance = guidance
        self._horizontal_controller = horizontal_controller
        self._attitude_controller = attitude_controller
        self._altitude_controller = altitude_controller
        self._touchdown_detector = touchdown_detector
        self._first_contact_impact = first_contact_impact
        self._simultaneous_contact_impact = (
            simultaneous_contact_impact
        )
        self._single_contact_stepper = (
            single_contact_stepper
        )
        self._single_contact_dynamics = (
            single_contact_dynamics
        )
        self._evaluator = evaluator
        self._has_run = False

    def run(
        self,
        *,
        max_step_count: int,
        dt: float,
    ) -> LandingRunResult2D:
        """Run until the landing completes or the step limit is reached."""
        self._validate_run_arguments(
            max_step_count=max_step_count,
            dt=dt,
        )

        if self._has_run:
            raise RuntimeError(
                "LandingOrchestrator2D is one-shot; "
                "create a new instance for another run"
            )

        initial_state = self._system.rigid_body_state
        initial_detection = self._touchdown_detector.detect(
            initial_state
        )

        if initial_detection.is_touchdown:
            raise ValueError(
                "landing orchestration must start before touchdown"
            )

        self._has_run = True

        time = float(self._system.time)
        state = initial_state
        mode = LandingDynamicsMode2D.FREE_FLIGHT

        times = [time]
        states = [state]
        modes = [mode]

        guidance_references: list[
            LandingGuidanceReference2D
        ] = []
        commands: list[HopperActuationCommand2D] = []
        physical_actuations: list[HopperActuation2D] = []

        first_contact_metrics: (
            LandingContactMetrics2D | None
        ) = None

        contact_constraint: (
            SingleContactConstraint2D | None
        ) = None

        single_contact_state: (
            SingleContactState2D | None
        ) = None

        previous_free_flight_detection = initial_detection

        for _ in range(max_step_count):
            references = self._guidance.compute_references(
                time,
                state,
            )

            command = self._compute_command(
                state=state,
                references=references,
            )

            guidance_references.append(references)
            commands.append(command)

            if mode is LandingDynamicsMode2D.FREE_FLIGHT:
                previous_state = state
                previous_detection = (
                    previous_free_flight_detection
                )

                system_step_result = self._system.step(
                    command,
                    dt,
                )

                physical_actuation = (
                    system_step_result.actuation
                )

                physical_actuations.append(
                    physical_actuation
                )

                state = system_step_result.rigid_body_state
                time = float(self._system.time)

                detection = self._touchdown_detector.detect(
                    state
                )

                if (
                    detection.contact
                    is TouchdownContact2D.NONE
                ):
                    previous_free_flight_detection = (
                        detection
                    )

                    times.append(time)
                    states.append(state)
                    modes.append(mode)
                    continue

                if self._is_genuine_simultaneous_contact(
                    detection
                ):
                    impact = (
                        self._simultaneous_contact_impact
                        .apply(
                            state,
                            detection,
                        )
                    )

                    evaluation = (
                        self._evaluator
                        .evaluate_simultaneous_contact(
                            time=time,
                            pre_impact_state=state,
                            impact_result=impact,
                            target_x=(
                                references
                                .horizontal_position
                                .px
                            ),
                        )
                    )

                    state = impact.post_impact_state
                    mode = LandingDynamicsMode2D.COMPLETED

                    times.append(time)
                    states.append(state)
                    modes.append(mode)

                    return self._build_result(
                        times=times,
                        states=states,
                        modes=modes,
                        guidance_references=(
                            guidance_references
                        ),
                        commands=commands,
                        physical_actuations=(
                            physical_actuations
                        ),
                        first_contact=None,
                        evaluation=evaluation,
                        contact_sequence=(
                            LandingContactSequence2D
                            .SIMULTANEOUS
                        ),
                        termination_reason=(
                            LandingTerminationReason2D
                            .COMPLETED
                        ),
                    )

                single_detection = (
                    self._resolve_single_contact_detection(
                        previous_detection=previous_detection,
                        current_detection=detection,
                    )
                )

                impact = self._first_contact_impact.apply(
                    state,
                    single_detection,
                )

                first_contact_metrics = (
                    self._evaluator.evaluate_first_contact(
                        time=time,
                        pre_impact_state=state,
                        impact_result=impact,
                    )
                )

                contact_constraint = impact.constraint
                single_contact_state = (
                    impact.single_contact_state
                )

                state = impact.post_impact_state
                mode = (
                    LandingDynamicsMode2D.SINGLE_CONTACT
                )

                times.append(time)
                states.append(state)
                modes.append(mode)
                continue

            if mode is LandingDynamicsMode2D.SINGLE_CONTACT:
                assert first_contact_metrics is not None
                assert contact_constraint is not None
                assert single_contact_state is not None

                physical_actuation = (
                    self._physical_actuation_provider(
                        command,
                        dt,
                    )
                )

                physical_actuations.append(
                    physical_actuation
                )

                single_contact_state = (
                    self._single_contact_stepper.step(
                        time=time,
                        state=single_contact_state,
                        constraint=contact_constraint,
                        actuation=physical_actuation,
                        dt=dt,
                    )
                )

                time += dt

                state = (
                    self._single_contact_dynamics
                    .to_hopper_state(
                        single_contact_state,
                        contact_constraint,
                    )
                )

                if not self._second_foot_has_contact(
                    state=state,
                    constraint=contact_constraint,
                ):
                    times.append(time)
                    states.append(state)
                    modes.append(mode)
                    continue

                both_detection = (
                    self._constrained_both_detection(
                        state=state,
                        constraint=contact_constraint,
                    )
                )

                evaluation = (
                    self._evaluator.evaluate_second_contact(
                        time=time,
                        pre_impact_state=state,
                        detection=both_detection,
                        first_contact=first_contact_metrics,
                        target_x=(
                            references
                            .horizontal_position
                            .px
                        ),
                    )
                )

                state = self._completed_sequential_state(
                    constraint=contact_constraint
                )

                mode = LandingDynamicsMode2D.COMPLETED

                times.append(time)
                states.append(state)
                modes.append(mode)

                return self._build_result(
                    times=times,
                    states=states,
                    modes=modes,
                    guidance_references=(
                        guidance_references
                    ),
                    commands=commands,
                    physical_actuations=(
                        physical_actuations
                    ),
                    first_contact=first_contact_metrics,
                    evaluation=evaluation,
                    contact_sequence=(
                        LandingContactSequence2D.SEQUENTIAL
                    ),
                    termination_reason=(
                        LandingTerminationReason2D.COMPLETED
                    ),
                )

            raise RuntimeError(
                f"unsupported landing mode: {mode}"
            )

        return self._build_result(
            times=times,
            states=states,
            modes=modes,
            guidance_references=guidance_references,
            commands=commands,
            physical_actuations=physical_actuations,
            first_contact=first_contact_metrics,
            evaluation=None,
            contact_sequence=None,
            termination_reason=(
                LandingTerminationReason2D.MAX_STEPS_REACHED
            ),
        )

    def _compute_command(
        self,
        *,
        state: HopperState2D,
        references: LandingGuidanceReference2D,
    ) -> HopperActuationCommand2D:
        attitude_reference = (
            self._horizontal_controller
            .compute_attitude_reference(
                state=state,
                reference=references.horizontal_position,
            )
        )

        gimbal_command = (
            self._attitude_controller
            .compute_gimbal_command(
                state=state,
                reference=attitude_reference,
            )
        )

        thrust_command = (
            self._altitude_controller
            .compute_thrust_command(
                state=state,
                reference=references.altitude,
            )
        )

        return HopperActuationCommand2D(
            thrust=thrust_command,
            gimbal_angle=gimbal_command,
        )

    def _is_genuine_simultaneous_contact(
        self,
        detection: TouchdownDetection2D,
    ) -> bool:
        if (
            detection.contact
            is not TouchdownContact2D.BOTH
        ):
            return False

        positions = detection.foot_positions

        return (
            abs(
                positions.left_z
                - positions.right_z
            )
            <= (
                self._simultaneous_contact_impact
                .max_foot_height_difference
            )
        )

    def _resolve_single_contact_detection(
        self,
        *,
        previous_detection: TouchdownDetection2D,
        current_detection: TouchdownDetection2D,
    ) -> TouchdownDetection2D:
        """Resolve a discrete non-simultaneous BOTH detection.

        If exactly one foot is already reported, it is used directly.
        If both feet crossed the ground within one free-flight step, the
        earlier crossing is estimated from the previous and current
        clearances. The impact itself is still applied at the current
        discrete state; exact event localization is a later improvement.
        """
        if (
            current_detection.contact
            is TouchdownContact2D.LEFT
        ):
            return current_detection

        if (
            current_detection.contact
            is TouchdownContact2D.RIGHT
        ):
            return current_detection

        if (
            current_detection.contact
            is not TouchdownContact2D.BOTH
        ):
            raise ValueError(
                "single-contact resolution requires touchdown"
            )

        left_fraction = self._crossing_fraction(
            previous_detection.left_clearance,
            current_detection.left_clearance,
        )

        right_fraction = self._crossing_fraction(
            previous_detection.right_clearance,
            current_detection.right_clearance,
        )

        if left_fraction < right_fraction:
            contact = TouchdownContact2D.LEFT
        elif right_fraction < left_fraction:
            contact = TouchdownContact2D.RIGHT
        else:
            # The genuinely simultaneous case should already have been
            # handled. For equal numerical crossing fractions with
            # unequal foot heights, use the lower foot as the robust
            # discrete fallback.
            if (
                current_detection.left_clearance
                <= current_detection.right_clearance
            ):
                contact = TouchdownContact2D.LEFT
            else:
                contact = TouchdownContact2D.RIGHT

        return TouchdownDetection2D(
            contact=contact,
            foot_positions=current_detection.foot_positions,
            ground_height=current_detection.ground_height,
            left_clearance=current_detection.left_clearance,
            right_clearance=current_detection.right_clearance,
        )

    @staticmethod
    def _crossing_fraction(
        previous_clearance: float,
        current_clearance: float,
    ) -> float:
        """Estimate the within-step ground crossing fraction."""
        if previous_clearance <= 0.0:
            return 0.0

        if current_clearance > 0.0:
            return float("inf")

        denominator = (
            previous_clearance
            - current_clearance
        )

        if denominator <= 0.0:
            return float("inf")

        return previous_clearance / denominator

    def _second_foot_has_contact(
        self,
        *,
        state: HopperState2D,
        constraint: SingleContactConstraint2D,
    ) -> bool:
        positions = (
            self._single_contact_dynamics
            .geometry
            .foot_positions_world(state)
        )

        ground = constraint.ground_height

        if constraint.foot.value == "left":
            return positions.right_z <= ground

        return positions.left_z <= ground

    def _constrained_both_detection(
        self,
        *,
        state: HopperState2D,
        constraint: SingleContactConstraint2D,
    ) -> TouchdownDetection2D:
        positions = (
            self._single_contact_dynamics
            .geometry
            .foot_positions_world(state)
        )

        ground = constraint.ground_height

        return TouchdownDetection2D(
            contact=TouchdownContact2D.BOTH,
            foot_positions=positions,
            ground_height=ground,
            left_clearance=positions.left_z - ground,
            right_clearance=positions.right_z - ground,
        )

    def _completed_sequential_state(
        self,
        *,
        constraint: SingleContactConstraint2D,
    ) -> HopperState2D:
        """Project the final sequential landing to a stationary two-foot pose."""
        return (
            self._single_contact_dynamics
            .to_hopper_state(
                SingleContactState2D(
                    theta=0.0,
                    omega=0.0,
                ),
                constraint,
            )
        )

    @staticmethod
    def _build_result(
        *,
        times: list[float],
        states: list[HopperState2D],
        modes: list[LandingDynamicsMode2D],
        guidance_references: list[
            LandingGuidanceReference2D
        ],
        commands: list[HopperActuationCommand2D],
        physical_actuations: list[HopperActuation2D],
        first_contact: LandingContactMetrics2D | None,
        evaluation: LandingEvaluationResult2D | None,
        contact_sequence: LandingContactSequence2D | None,
        termination_reason: LandingTerminationReason2D,
    ) -> LandingRunResult2D:
        return LandingRunResult2D(
            times=tuple(times),
            rigid_body_states=tuple(states),
            modes=tuple(modes),
            guidance_references=tuple(
                guidance_references
            ),
            commands=tuple(commands),
            physical_actuations=tuple(
                physical_actuations
            ),
            first_contact=first_contact,
            evaluation=evaluation,
            contact_sequence=contact_sequence,
            termination_reason=termination_reason,
        )

    @staticmethod
    def _validate_run_arguments(
        *,
        max_step_count: int,
        dt: float,
    ) -> None:
        if (
            not isinstance(max_step_count, int)
            or isinstance(max_step_count, bool)
        ):
            raise TypeError(
                "max_step_count must be an integer"
            )

        if max_step_count <= 0:
            raise ValueError(
                "max_step_count must be > 0"
            )

        if not isfinite(dt) or dt <= 0.0:
            raise ValueError(
                "dt must be finite and > 0"
            )
