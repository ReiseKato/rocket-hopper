import math

import pytest

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing import (
    SimultaneousContactImpactTransition2D,
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


def test_simultaneous_impact_stops_all_rigid_body_motion() -> None:
    geometry = create_geometry()

    state = HopperState2D(
        px=0.1,
        pz=CG_TO_FEET,
        vx=0.12,
        vz=-0.20,
        theta=0.0,
        omega=math.radians(3.0),
    )

    detection = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    ).detect(state)

    assert detection.contact is TouchdownContact2D.BOTH

    result = SimultaneousContactImpactTransition2D(
        create_parameters(),
        geometry,
    ).apply(
        state,
        detection,
    )

    assert result.post_impact_state.vx == pytest.approx(0.0)
    assert result.post_impact_state.vz == pytest.approx(0.0)
    assert result.post_impact_state.omega == pytest.approx(0.0)


def test_simultaneous_impact_dissipates_all_kinetic_energy() -> None:
    geometry = create_geometry()

    state = HopperState2D(
        px=0.0,
        pz=CG_TO_FEET,
        vx=0.10,
        vz=-0.25,
        theta=0.0,
        omega=math.radians(4.0),
    )

    detection = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    ).detect(state)

    result = SimultaneousContactImpactTransition2D(
        create_parameters(),
        geometry,
    ).apply(
        state,
        detection,
    )

    expected = (
        0.5 * MASS * (
            state.vx**2
            + state.vz**2
        )
        + 0.5 * INERTIA_Y * state.omega**2
    )

    assert result.kinetic_energy_before == pytest.approx(
        expected
    )
    assert result.kinetic_energy_after == pytest.approx(
        0.0
    )
    assert result.dissipated_energy == pytest.approx(
        expected
    )


def test_simultaneous_impact_records_both_foot_velocities() -> None:
    geometry = create_geometry()

    state = HopperState2D(
        px=0.0,
        pz=CG_TO_FEET,
        vx=0.0,
        vz=-0.2,
        theta=0.0,
        omega=math.radians(10.0),
    )

    detection = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    ).detect(state)

    result = SimultaneousContactImpactTransition2D(
        create_parameters(),
        geometry,
    ).apply(
        state,
        detection,
    )

    assert (
        result.left_contact_velocity_before_z
        != pytest.approx(
            result.right_contact_velocity_before_z
        )
    )


def test_both_detection_with_different_foot_heights_is_rejected() -> None:
    geometry = create_geometry()

    # Both feet penetrate the ground after a coarse discrete step,
    # but the non-zero attitude means the contacts were not simultaneous.
    state = HopperState2D(
        px=0.0,
        pz=0.70,
        vx=0.0,
        vz=-0.5,
        theta=math.radians(5.0),
        omega=0.0,
    )

    detection = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    ).detect(state)

    assert detection.contact is TouchdownContact2D.BOTH

    with pytest.raises(
        ValueError,
        match="not geometrically simultaneous",
    ):
        SimultaneousContactImpactTransition2D(
            create_parameters(),
            geometry,
            max_foot_height_difference=1e-6,
        ).apply(
            state,
            detection,
        )


def test_simultaneous_impact_rejects_single_foot_contact() -> None:
    geometry = create_geometry()
    theta = math.radians(10.0)

    state = HopperState2D(
        px=0.0,
        pz=(
            CG_TO_FEET * math.cos(theta)
            + 0.3 * math.sin(theta)
        ),
        vx=0.0,
        vz=-0.2,
        theta=theta,
        omega=0.0,
    )

    detection = TouchdownDetector2D(
        geometry,
        ground_height=0.0,
    ).detect(state)

    assert detection.contact is TouchdownContact2D.RIGHT

    with pytest.raises(ValueError):
        SimultaneousContactImpactTransition2D(
            create_parameters(),
            geometry,
        ).apply(
            state,
            detection,
        )
