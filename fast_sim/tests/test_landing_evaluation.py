import math

import pytest

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing.contact import (
    FirstContactImpactResult2D,
    SingleContactConstraint2D,
    SingleContactFoot2D,
    SingleContactState2D,
)
from fast_sim.landing.evaluation import (
    LandingContactMetrics2D,
    LandingEvaluator2D,
    LandingFailureReason2D,
    LandingSuccessCriteria2D,
)
from fast_sim.landing.touchdown import (
    TouchdownContact2D,
    TouchdownDetector2D,
)
from fast_sim.models import (
    HopperState2D,
    RigidBody2DParameters,
)


MASS = 2.0
INERTIA_Y = 0.08
ENGINE_OFFSET = 0.25
GRAVITY = 9.81

FOOT_SPACING = 0.6
CG_TO_FEET = 0.8


def create_parameters() -> RigidBody2DParameters:
    return RigidBody2DParameters(
        mass=MASS,
        inertia_y=INERTIA_Y,
        engine_offset=ENGINE_OFFSET,
        gravity=GRAVITY,
    )


def create_geometry() -> LandingGearGeometry2D:
    return LandingGearGeometry2D(
        foot_spacing=FOOT_SPACING,
        cg_to_feet=CG_TO_FEET,
    )


def create_criteria(
    *,
    max_horizontal_position_error: float = 0.1,
    max_first_contact_normal_speed: float = 0.3,
    max_first_contact_tangential_speed: float = 0.2,
    max_second_contact_normal_speed: float = 0.3,
    max_second_contact_tangential_speed: float = 0.2,
    max_attitude_angle: float = math.radians(5.0),
    max_angular_speed: float = math.radians(20.0),
) -> LandingSuccessCriteria2D:
    return LandingSuccessCriteria2D(
        max_horizontal_position_error=(
            max_horizontal_position_error
        ),
        max_first_contact_normal_speed=(
            max_first_contact_normal_speed
        ),
        max_first_contact_tangential_speed=(
            max_first_contact_tangential_speed
        ),
        max_second_contact_normal_speed=(
            max_second_contact_normal_speed
        ),
        max_second_contact_tangential_speed=(
            max_second_contact_tangential_speed
        ),
        max_attitude_angle=max_attitude_angle,
        max_angular_speed=max_angular_speed,
    )


def create_evaluator(
    criteria: LandingSuccessCriteria2D | None = None,
) -> LandingEvaluator2D:
    return LandingEvaluator2D(
        parameters=create_parameters(),
        geometry=create_geometry(),
        criteria=criteria or create_criteria(),
    )


def create_first_impact_result() -> FirstContactImpactResult2D:
    post_state = HopperState2D(
        px=0.0,
        pz=0.8,
        vx=-0.08,
        vz=-0.03,
        theta=math.radians(2.0),
        omega=-0.1,
    )

    return FirstContactImpactResult2D(
        constraint=SingleContactConstraint2D(
            foot=SingleContactFoot2D.RIGHT,
            contact_x=0.3,
            ground_height=0.0,
        ),
        single_contact_state=SingleContactState2D(
            theta=post_state.theta,
            omega=post_state.omega,
        ),
        post_impact_state=post_state,
        contact_velocity_before_x=0.05,
        contact_velocity_before_z=-0.15,
        angular_momentum_about_contact_before=-0.2,
        angular_momentum_about_contact_after=-0.2,
        kinetic_energy_before=0.20,
        kinetic_energy_after=0.12,
        dissipated_energy=0.08,
    )


def create_first_contact_metrics() -> LandingContactMetrics2D:
    return LandingContactMetrics2D(
        foot=SingleContactFoot2D.RIGHT,
        time=1.0,
        normal_speed=0.15,
        tangential_speed=0.05,
        vehicle_theta=math.radians(2.0),
        vehicle_omega=-0.1,
        cg_vx=0.02,
        cg_vz=-0.12,
        kinetic_energy_before=0.20,
        kinetic_energy_after=0.12,
        dissipated_energy=0.08,
    )


def create_second_contact_state(
    *,
    px: float = 0.0,
    omega: float = -0.2,
) -> HopperState2D:
    # Physically consistent state for rotation around an upright right
    # contact point. At theta=0 the right foot is fixed and the left
    # foot reaches the ground with a downward velocity.
    q_x = -FOOT_SPACING / 2.0
    q_z = CG_TO_FEET

    vx = omega * q_z
    vz = -omega * q_x

    return HopperState2D(
        px=px,
        pz=CG_TO_FEET,
        vx=vx,
        vz=vz,
        theta=0.0,
        omega=omega,
    )


def test_first_contact_metrics_use_pre_impact_foot_velocity() -> None:
    evaluator = create_evaluator()

    pre_impact_state = HopperState2D(
        px=0.0,
        pz=0.9,
        vx=0.02,
        vz=-0.12,
        theta=math.radians(2.0),
        omega=-0.1,
    )

    metrics = evaluator.evaluate_first_contact(
        time=1.0,
        pre_impact_state=pre_impact_state,
        impact_result=create_first_impact_result(),
    )

    assert metrics.foot is SingleContactFoot2D.RIGHT
    assert metrics.normal_speed == pytest.approx(0.15)
    assert metrics.tangential_speed == pytest.approx(0.05)

    assert metrics.kinetic_energy_before == pytest.approx(0.20)
    assert metrics.kinetic_energy_after == pytest.approx(0.12)
    assert metrics.dissipated_energy == pytest.approx(0.08)


def test_second_contact_identifies_opposite_foot() -> None:
    evaluator = create_evaluator()
    state = create_second_contact_state()

    detection = TouchdownDetector2D(
        create_geometry(),
        ground_height=0.0,
    ).detect(state)

    assert detection.contact is TouchdownContact2D.BOTH

    evaluation = evaluator.evaluate_second_contact(
        time=1.2,
        pre_impact_state=state,
        detection=detection,
        first_contact=create_first_contact_metrics(),
        target_x=0.0,
    )

    assert (
        evaluation.second_contact.foot
        is SingleContactFoot2D.LEFT
    )


def test_second_contact_uses_actual_second_foot_velocity() -> None:
    evaluator = create_evaluator()
    state = create_second_contact_state(
        omega=-0.2
    )

    detection = TouchdownDetector2D(
        create_geometry(),
        ground_height=0.0,
    ).detect(state)

    evaluation = evaluator.evaluate_second_contact(
        time=1.2,
        pre_impact_state=state,
        detection=detection,
        first_contact=create_first_contact_metrics(),
        target_x=0.0,
    )

    # For the chosen constrained state the left foot has no tangential
    # speed and moves downward at 0.12 m/s.
    assert (
        evaluation.second_contact.normal_speed
        == pytest.approx(0.12)
    )

    assert (
        evaluation.second_contact.tangential_speed
        == pytest.approx(0.0)
    )


def test_successful_completed_landing_has_no_failures() -> None:
    evaluator = create_evaluator()
    state = create_second_contact_state()

    detection = TouchdownDetector2D(
        create_geometry(),
        ground_height=0.0,
    ).detect(state)

    evaluation = evaluator.evaluate_second_contact(
        time=1.2,
        pre_impact_state=state,
        detection=detection,
        first_contact=create_first_contact_metrics(),
        target_x=0.0,
    )

    assert evaluation.completed
    assert evaluation.successful
    assert evaluation.failed_criteria == ()

    assert evaluation.horizontal_position_error == pytest.approx(
        0.0
    )

    assert evaluation.time_between_contacts == pytest.approx(
        0.2
    )


def test_second_contact_energy_is_fully_dissipated() -> None:
    evaluator = create_evaluator()
    state = create_second_contact_state()

    detection = TouchdownDetector2D(
        create_geometry(),
        ground_height=0.0,
    ).detect(state)

    evaluation = evaluator.evaluate_second_contact(
        time=1.2,
        pre_impact_state=state,
        detection=detection,
        first_contact=create_first_contact_metrics(),
        target_x=0.0,
    )

    expected_energy = (
        0.5
        * MASS
        * (
            state.vx**2
            + state.vz**2
        )
        + 0.5
        * INERTIA_Y
        * state.omega**2
    )

    assert (
        evaluation.second_contact.kinetic_energy_before
        == pytest.approx(expected_energy)
    )

    assert (
        evaluation.second_contact.kinetic_energy_after
        == pytest.approx(0.0)
    )

    assert (
        evaluation.second_contact.dissipated_energy
        == pytest.approx(expected_energy)
    )

    assert evaluation.total_dissipated_energy == pytest.approx(
        0.08 + expected_energy
    )


def test_failed_criteria_report_all_relevant_reasons() -> None:
    criteria = create_criteria(
        max_horizontal_position_error=0.01,
        max_first_contact_normal_speed=0.10,
        max_first_contact_tangential_speed=0.01,
        max_second_contact_normal_speed=0.10,
        max_second_contact_tangential_speed=0.01,
        max_attitude_angle=math.radians(1.0),
        max_angular_speed=math.radians(5.0),
    )

    evaluator = create_evaluator(criteria)

    state = HopperState2D(
        px=0.05,
        pz=0.70,
        vx=0.20,
        vz=-0.20,
        theta=math.radians(3.0),
        omega=math.radians(-10.0),
    )

    detection = TouchdownDetector2D(
        create_geometry(),
        ground_height=0.0,
    ).detect(state)

    assert detection.contact is TouchdownContact2D.BOTH

    evaluation = evaluator.evaluate_second_contact(
        time=1.2,
        pre_impact_state=state,
        detection=detection,
        first_contact=create_first_contact_metrics(),
        target_x=0.0,
    )

    assert not evaluation.successful

    assert evaluation.failed_criteria == (
        LandingFailureReason2D.HORIZONTAL_POSITION_ERROR,
        LandingFailureReason2D.FIRST_CONTACT_NORMAL_SPEED,
        LandingFailureReason2D.FIRST_CONTACT_TANGENTIAL_SPEED,
        LandingFailureReason2D.SECOND_CONTACT_NORMAL_SPEED,
        LandingFailureReason2D.SECOND_CONTACT_TANGENTIAL_SPEED,
        LandingFailureReason2D.ATTITUDE_ANGLE,
        LandingFailureReason2D.ANGULAR_SPEED,
    )


def test_second_contact_requires_both_feet_on_ground() -> None:
    evaluator = create_evaluator()

    state = HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )

    detection = TouchdownDetector2D(
        create_geometry(),
        ground_height=0.0,
    ).detect(state)

    assert detection.contact is TouchdownContact2D.NONE

    with pytest.raises(ValueError):
        evaluator.evaluate_second_contact(
            time=1.2,
            pre_impact_state=state,
            detection=detection,
            first_contact=create_first_contact_metrics(),
            target_x=0.0,
        )


def test_second_contact_cannot_precede_first_contact() -> None:
    evaluator = create_evaluator()
    state = create_second_contact_state()

    detection = TouchdownDetector2D(
        create_geometry(),
        ground_height=0.0,
    ).detect(state)

    with pytest.raises(ValueError):
        evaluator.evaluate_second_contact(
            time=0.9,
            pre_impact_state=state,
            detection=detection,
            first_contact=create_first_contact_metrics(),
            target_x=0.0,
        )


@pytest.mark.parametrize(
    "field_name",
    [
        "max_horizontal_position_error",
        "max_first_contact_normal_speed",
        "max_first_contact_tangential_speed",
        "max_second_contact_normal_speed",
        "max_second_contact_tangential_speed",
        "max_attitude_angle",
        "max_angular_speed",
    ],
)
def test_success_criteria_reject_negative_limits(
    field_name: str,
) -> None:
    arguments = {
        "max_horizontal_position_error": 0.1,
        "max_first_contact_normal_speed": 0.3,
        "max_first_contact_tangential_speed": 0.2,
        "max_second_contact_normal_speed": 0.3,
        "max_second_contact_tangential_speed": 0.2,
        "max_attitude_angle": math.radians(5.0),
        "max_angular_speed": math.radians(20.0),
    }

    arguments[field_name] = -0.1

    with pytest.raises(ValueError):
        LandingSuccessCriteria2D(
            **arguments
        )
