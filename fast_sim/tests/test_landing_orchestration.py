import math

import pytest

from fast_sim.control import (
    AltitudeReference2D,
    AttitudeReference2D,
    HorizontalPositionReference2D,
)
from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing import (
    FirstContactImpactTransition2D,
    LandingContactSequence2D,
    LandingDynamicsMode2D,
    LandingEvaluator2D,
    LandingGuidanceReference2D,
    LandingOrchestrator2D,
    LandingPhase2D,
    LandingSuccessCriteria2D,
    LandingTerminationReason2D,
    SimultaneousContactImpactTransition2D,
    SingleContactDynamics2D,
    SingleContactState2D,
    TouchdownDetector2D,
)
from fast_sim.models import (
    HopperActuation2D,
    HopperState2D,
    RigidBody2DParameters,
)


MASS = 2.0
INERTIA_Y = 0.08
ENGINE_OFFSET = 0.25
GRAVITY = 9.81
FOOT_SPACING = 0.6
CG_TO_FEET = 0.8


class FakeGuidance:
    def compute_references(self, time, state):
        return LandingGuidanceReference2D(
            phase=LandingPhase2D.FINAL,
            horizontal_position=HorizontalPositionReference2D(
                px=0.0,
                vx=0.0,
            ),
            altitude=AltitudeReference2D(
                pz=CG_TO_FEET,
                vz=-0.1,
            ),
        )


class FakeHorizontalController:
    def compute_attitude_reference(self, state, reference):
        return AttitudeReference2D(
            theta=0.0,
            omega=0.0,
        )


class FakeAttitudeController:
    def compute_gimbal_command(self, state, reference):
        return 0.0


class FakeAltitudeController:
    def compute_thrust_command(self, state, reference):
        return MASS * GRAVITY


class FakeSystemStepResult:
    def __init__(self, state, actuation):
        self.rigid_body_state = state
        self.actuation = actuation


class FakeSystem:
    def __init__(self, states):
        self._states = list(states)
        self._index = 0
        self.time = 0.0
        self.rigid_body_state = self._states[0]

    def step(self, command, dt):
        self._index += 1
        self.time += dt
        self.rigid_body_state = self._states[self._index]

        return FakeSystemStepResult(
            self.rigid_body_state,
            HopperActuation2D(
                thrust=command.thrust,
                gimbal_angle=command.gimbal_angle,
            ),
        )


class FakeSingleContactStepper:
    def __init__(self, states):
        self._states = iter(states)

    def step(
        self,
        *,
        time,
        state,
        constraint,
        actuation,
        dt,
    ):
        return next(self._states)


def create_parameters():
    return RigidBody2DParameters(
        mass=MASS,
        inertia_y=INERTIA_Y,
        engine_offset=ENGINE_OFFSET,
        gravity=GRAVITY,
    )


def create_geometry():
    return LandingGearGeometry2D(
        foot_spacing=FOOT_SPACING,
        cg_to_feet=CG_TO_FEET,
    )


def create_evaluator():
    return LandingEvaluator2D(
        create_parameters(),
        create_geometry(),
        LandingSuccessCriteria2D(
            max_horizontal_position_error=1.0,
            max_first_contact_normal_speed=2.0,
            max_first_contact_tangential_speed=2.0,
            max_second_contact_normal_speed=2.0,
            max_second_contact_tangential_speed=2.0,
            max_attitude_angle=math.radians(20.0),
            max_angular_speed=math.radians(100.0),
            max_simultaneous_contact_normal_speed=2.0,
            max_simultaneous_contact_tangential_speed=2.0,
        ),
    )


def create_orchestrator(system, single_contact_stepper):
    geometry = create_geometry()

    return LandingOrchestrator2D(
        system=system,
        physical_actuation_provider=(
            lambda command, dt: HopperActuation2D(
                thrust=command.thrust,
                gimbal_angle=command.gimbal_angle,
            )
        ),
        guidance=FakeGuidance(),
        horizontal_controller=FakeHorizontalController(),
        attitude_controller=FakeAttitudeController(),
        altitude_controller=FakeAltitudeController(),
        touchdown_detector=TouchdownDetector2D(
            geometry,
            ground_height=0.0,
        ),
        first_contact_impact=FirstContactImpactTransition2D(
            create_parameters(),
            geometry,
        ),
        simultaneous_contact_impact=(
            SimultaneousContactImpactTransition2D(
                create_parameters(),
                geometry,
                max_foot_height_difference=1e-6,
            )
        ),
        single_contact_stepper=single_contact_stepper,
        single_contact_dynamics=SingleContactDynamics2D(
            create_parameters(),
            geometry,
        ),
        evaluator=create_evaluator(),
    )


def test_sequential_landing_reaches_completed_evaluation() -> None:
    theta = math.radians(10.0)

    right_touchdown_pz = (
        CG_TO_FEET * math.cos(theta)
        + 0.5 * FOOT_SPACING * math.sin(theta)
    )

    system = FakeSystem(
        [
            HopperState2D(
                px=0.0,
                pz=1.2,
                vx=0.0,
                vz=-0.2,
                theta=theta,
                omega=0.0,
            ),
            HopperState2D(
                px=0.0,
                pz=right_touchdown_pz,
                vx=0.0,
                vz=-0.2,
                theta=theta,
                omega=0.0,
            ),
        ]
    )

    orchestrator = create_orchestrator(
        system,
        FakeSingleContactStepper(
            [
                SingleContactState2D(
                    theta=math.radians(5.0),
                    omega=-0.1,
                ),
                SingleContactState2D(
                    theta=0.0,
                    omega=-0.1,
                ),
            ]
        ),
    )

    result = orchestrator.run(
        max_step_count=3,
        dt=0.01,
    )

    assert result.completed
    assert result.successful

    assert (
        result.contact_sequence
        is LandingContactSequence2D.SEQUENTIAL
    )

    assert (
        result.modes[-1]
        is LandingDynamicsMode2D.COMPLETED
    )

    assert result.first_contact is not None
    assert result.evaluation is not None


def test_genuine_simultaneous_landing_completes_directly() -> None:
    system = FakeSystem(
        [
            HopperState2D(
                px=0.0,
                pz=0.9,
                vx=0.0,
                vz=-0.1,
                theta=0.0,
                omega=0.0,
            ),
            HopperState2D(
                px=0.0,
                pz=CG_TO_FEET,
                vx=0.0,
                vz=-0.1,
                theta=0.0,
                omega=0.0,
            ),
        ]
    )

    result = create_orchestrator(
        system,
        FakeSingleContactStepper([]),
    ).run(
        max_step_count=1,
        dt=0.01,
    )

    assert result.completed
    assert result.successful

    assert (
        result.contact_sequence
        is LandingContactSequence2D.SIMULTANEOUS
    )

    assert result.first_contact is None
    assert result.evaluation is not None


def test_non_simultaneous_both_detection_is_resolved_as_single_contact() -> None:
    theta = math.radians(5.0)

    system = FakeSystem(
        [
            HopperState2D(
                px=0.0,
                pz=0.9,
                vx=0.0,
                vz=-0.3,
                theta=theta,
                omega=0.0,
            ),
            # Both feet are below ground, but their heights differ.
            HopperState2D(
                px=0.0,
                pz=0.70,
                vx=0.0,
                vz=-0.3,
                theta=theta,
                omega=0.0,
            ),
        ]
    )

    result = create_orchestrator(
        system,
        FakeSingleContactStepper([]),
    ).run(
        max_step_count=1,
        dt=0.01,
    )

    assert not result.completed
    assert result.first_contact is not None

    assert (
        result.modes[-1]
        is LandingDynamicsMode2D.SINGLE_CONTACT
    )

    assert result.contact_sequence is None


def test_max_steps_reached_without_touchdown() -> None:
    system = FakeSystem(
        [
            HopperState2D(
                px=0.0,
                pz=2.0,
                vx=0.0,
                vz=0.0,
                theta=0.0,
                omega=0.0,
            ),
            HopperState2D(
                px=0.0,
                pz=1.9,
                vx=0.0,
                vz=-0.1,
                theta=0.0,
                omega=0.0,
            ),
        ]
    )

    result = create_orchestrator(
        system,
        FakeSingleContactStepper([]),
    ).run(
        max_step_count=1,
        dt=0.01,
    )

    assert not result.completed
    assert not result.successful

    assert (
        result.termination_reason
        is LandingTerminationReason2D.MAX_STEPS_REACHED
    )


def test_result_keeps_n_plus_one_state_semantics() -> None:
    system = FakeSystem(
        [
            HopperState2D(
                px=0.0,
                pz=2.0,
                vx=0.0,
                vz=0.0,
                theta=0.0,
                omega=0.0,
            ),
            HopperState2D(
                px=0.0,
                pz=1.9,
                vx=0.0,
                vz=-0.1,
                theta=0.0,
                omega=0.0,
            ),
        ]
    )

    result = create_orchestrator(
        system,
        FakeSingleContactStepper([]),
    ).run(
        max_step_count=1,
        dt=0.01,
    )

    assert result.step_count == 1

    assert len(result.times) == 2
    assert len(result.rigid_body_states) == 2
    assert len(result.modes) == 2

    assert len(result.guidance_references) == 1
    assert len(result.commands) == 1
    assert len(result.physical_actuations) == 1


def test_orchestrator_is_one_shot() -> None:
    system = FakeSystem(
        [
            HopperState2D(
                px=0.0,
                pz=2.0,
                vx=0.0,
                vz=0.0,
                theta=0.0,
                omega=0.0,
            ),
            HopperState2D(
                px=0.0,
                pz=1.9,
                vx=0.0,
                vz=-0.1,
                theta=0.0,
                omega=0.0,
            ),
        ]
    )

    orchestrator = create_orchestrator(
        system,
        FakeSingleContactStepper([]),
    )

    orchestrator.run(
        max_step_count=1,
        dt=0.01,
    )

    with pytest.raises(RuntimeError):
        orchestrator.run(
            max_step_count=1,
            dt=0.01,
        )
