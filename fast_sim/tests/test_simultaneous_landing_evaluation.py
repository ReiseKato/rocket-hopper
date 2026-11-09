import math

import pytest

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing import (
    LandingEvaluator2D,
    LandingFailureReason2D,
    LandingSuccessCriteria2D,
    SimultaneousContactImpactTransition2D,
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
CG_TO_FEET = 0.8


def create_geometry() -> LandingGearGeometry2D:
    return LandingGearGeometry2D(
        foot_spacing=0.6,
        cg_to_feet=CG_TO_FEET,
    )


def create_parameters() -> RigidBody2DParameters:
    return RigidBody2DParameters(
        mass=MASS,
        inertia_y=INERTIA_Y,
        engine_offset=ENGINE_OFFSET,
        gravity=GRAVITY,
    )


def create_criteria(**overrides) -> LandingSuccessCriteria2D:
    values = {
        "max_horizontal_position_error": 0.1,
        "max_first_contact_normal_speed": 0.3,
        "max_first_contact_tangential_speed": 0.2,
        "max_second_contact_normal_speed": 0.25,
        "max_second_contact_tangential_speed": 0.15,
        "max_attitude_angle": math.radians(5.0),
        "max_angular_speed": math.radians(20.0),
    }
    values.update(overrides)
    return LandingSuccessCriteria2D(**values)


def test_simultaneous_landing_can_be_successful() -> None:
    geometry = create_geometry()
    parameters = create_parameters()

    state = HopperState2D(
        px=0.02,
        pz=CG_TO_FEET,
        vx=0.03,
        vz=-0.10,
        theta=0.0,
        omega=math.radians(2.0),
    )

    detection = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    ).detect(state)

    impact = SimultaneousContactImpactTransition2D(
        parameters,
        geometry,
    ).apply(
        state,
        detection,
    )

    evaluation = LandingEvaluator2D(
        parameters,
        geometry,
        create_criteria(),
    ).evaluate_simultaneous_contact(
        time=2.0,
        pre_impact_state=state,
        impact_result=impact,
        target_x=0.0,
    )

    assert evaluation.completed
    assert evaluation.successful
    assert evaluation.failed_criteria == ()
    assert evaluation.total_dissipated_energy == pytest.approx(
        impact.dissipated_energy
    )


def test_dedicated_simultaneous_limits_override_default_limits() -> None:
    geometry = create_geometry()
    parameters = create_parameters()

    state = HopperState2D(
        px=0.0,
        pz=CG_TO_FEET,
        vx=0.0,
        vz=-0.20,
        theta=0.0,
        omega=0.0,
    )

    detection = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    ).detect(state)

    impact = SimultaneousContactImpactTransition2D(
        parameters,
        geometry,
    ).apply(
        state,
        detection,
    )

    evaluation = LandingEvaluator2D(
        parameters,
        geometry,
        create_criteria(
            max_simultaneous_contact_normal_speed=0.1,
        ),
    ).evaluate_simultaneous_contact(
        time=2.0,
        pre_impact_state=state,
        impact_result=impact,
        target_x=0.0,
    )

    assert not evaluation.successful

    assert (
        LandingFailureReason2D.SIMULTANEOUS_LEFT_NORMAL_SPEED
        in evaluation.failed_criteria
    )
    assert (
        LandingFailureReason2D.SIMULTANEOUS_RIGHT_NORMAL_SPEED
        in evaluation.failed_criteria
    )


def test_default_simultaneous_limit_uses_stricter_sequential_limit() -> None:
    criteria = create_criteria()

    assert (
        criteria.effective_simultaneous_normal_speed_limit
        == pytest.approx(0.25)
    )

    assert (
        criteria.effective_simultaneous_tangential_speed_limit
        == pytest.approx(0.15)
    )
