import math

import pytest

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing import (
    FirstContactImpactTransition2D,
    SingleContactFoot2D,
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


def test_right_first_contact_conserves_angular_momentum_about_contact() -> None:
    geometry = create_geometry()
    detector = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    )

    theta = math.radians(10.0)

    pz = (
        CG_TO_FEET * math.cos(theta)
        + 0.5 * FOOT_SPACING * math.sin(theta)
    )

    state = HopperState2D(
        px=0.2,
        pz=pz,
        vx=0.25,
        vz=-0.4,
        theta=theta,
        omega=math.radians(8.0),
    )

    detection = detector.detect(state)

    assert detection.contact is TouchdownContact2D.RIGHT

    transition = FirstContactImpactTransition2D(
        create_parameters(),
        geometry,
    )

    result = transition.apply(
        state,
        detection,
    )

    assert (
        result.constraint.foot
        is SingleContactFoot2D.RIGHT
    )

    assert (
        result.angular_momentum_about_contact_after
        == pytest.approx(
            result.angular_momentum_about_contact_before
        )
    )


def test_contacting_foot_is_stationary_after_impact() -> None:
    geometry = create_geometry()
    detector = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    )

    theta = math.radians(10.0)

    pz = (
        CG_TO_FEET * math.cos(theta)
        + 0.5 * FOOT_SPACING * math.sin(theta)
    )

    state = HopperState2D(
        px=0.0,
        pz=pz,
        vx=0.1,
        vz=-0.5,
        theta=theta,
        omega=math.radians(5.0),
    )

    result = FirstContactImpactTransition2D(
        create_parameters(),
        geometry,
    ).apply(
        state,
        detector.detect(state),
    )

    post = result.post_impact_state
    positions = geometry.foot_positions_world(
        post
    )

    assert positions.right_x == pytest.approx(
        result.constraint.contact_x
    )
    assert positions.right_z == pytest.approx(
        result.constraint.ground_height
    )

    body_x = FOOT_SPACING / 2.0
    body_z = -CG_TO_FEET

    r_x = (
        body_x * math.cos(post.theta)
        + body_z * math.sin(post.theta)
    )
    r_z = (
        -body_x * math.sin(post.theta)
        + body_z * math.cos(post.theta)
    )

    foot_vx = (
        post.vx
        + post.omega * r_z
    )
    foot_vz = (
        post.vz
        - post.omega * r_x
    )

    assert foot_vx == pytest.approx(0.0)
    assert foot_vz == pytest.approx(0.0)


def test_first_impact_dissipates_or_preserves_kinetic_energy() -> None:
    geometry = create_geometry()
    detector = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    )

    theta = math.radians(8.0)

    pz = (
        CG_TO_FEET * math.cos(theta)
        + 0.5 * FOOT_SPACING * math.sin(theta)
    )

    state = HopperState2D(
        px=0.0,
        pz=pz,
        vx=0.3,
        vz=-0.6,
        theta=theta,
        omega=math.radians(12.0),
    )

    result = FirstContactImpactTransition2D(
        create_parameters(),
        geometry,
    ).apply(
        state,
        detector.detect(state),
    )

    assert (
        result.kinetic_energy_after
        <= result.kinetic_energy_before
        + 1e-12
    )

    assert result.dissipated_energy >= -1e-12


def test_impact_rejects_no_contact() -> None:
    geometry = create_geometry()

    state = HopperState2D(
        px=0.0,
        pz=2.0,
        vx=0.0,
        vz=-0.2,
        theta=0.0,
        omega=0.0,
    )

    detector = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    )

    detection = detector.detect(state)

    assert detection.contact is TouchdownContact2D.NONE

    with pytest.raises(ValueError):
        FirstContactImpactTransition2D(
            create_parameters(),
            geometry,
        ).apply(
            state,
            detection,
        )


def test_impact_rejects_simultaneous_two_foot_contact() -> None:
    geometry = create_geometry()

    state = HopperState2D(
        px=0.0,
        pz=CG_TO_FEET,
        vx=0.0,
        vz=-0.2,
        theta=0.0,
        omega=0.0,
    )

    detector = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    )

    detection = detector.detect(state)

    assert detection.contact is TouchdownContact2D.BOTH

    with pytest.raises(ValueError):
        FirstContactImpactTransition2D(
            create_parameters(),
            geometry,
        ).apply(
            state,
            detection,
        )
