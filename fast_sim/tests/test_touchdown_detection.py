import math

import pytest

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing import (
    TouchdownContact2D,
    TouchdownDetector2D,
)
from fast_sim.models import HopperState2D


FOOT_SPACING = 0.6
CG_TO_FEET = 0.8


def create_state(
    *,
    px: float = 0.0,
    pz: float = 2.0,
    theta: float = 0.0,
) -> HopperState2D:
    return HopperState2D(
        px=px,
        pz=pz,
        vx=0.0,
        vz=0.0,
        theta=theta,
        omega=0.0,
    )


def create_detector(
    *,
    ground_height: float = 0.0,
) -> TouchdownDetector2D:
    return TouchdownDetector2D(
        LandingGearGeometry2D(
            foot_spacing=FOOT_SPACING,
            cg_to_feet=CG_TO_FEET,
        ),
        ground_height=ground_height,
    )


def test_no_touchdown_when_both_feet_are_above_ground() -> None:
    detector = create_detector()

    detection = detector.detect(
        create_state(
            pz=1.0,
        )
    )

    assert detection.contact is TouchdownContact2D.NONE
    assert not detection.is_touchdown
    assert not detection.left_in_contact
    assert not detection.right_in_contact

    assert detection.left_clearance == pytest.approx(
        0.2
    )
    assert detection.right_clearance == pytest.approx(
        0.2
    )


def test_upright_touchdown_occurs_when_cg_height_equals_cg_to_feet() -> None:
    detector = create_detector()

    detection = detector.detect(
        create_state(
            pz=CG_TO_FEET,
        )
    )

    assert detection.contact is TouchdownContact2D.BOTH
    assert detection.is_touchdown
    assert detection.left_in_contact
    assert detection.right_in_contact

    assert detection.left_clearance == pytest.approx(
        0.0
    )
    assert detection.right_clearance == pytest.approx(
        0.0
    )


def test_positive_attitude_touches_right_foot_first() -> None:
    detector = create_detector()
    theta = math.radians(10.0)

    right_touchdown_cg_height = (
        CG_TO_FEET * math.cos(theta)
        + 0.5 * FOOT_SPACING * math.sin(theta)
    )

    detection = detector.detect(
        create_state(
            pz=right_touchdown_cg_height,
            theta=theta,
        )
    )

    assert detection.contact is TouchdownContact2D.RIGHT
    assert detection.is_touchdown

    assert not detection.left_in_contact
    assert detection.right_in_contact

    assert detection.right_clearance == pytest.approx(
        0.0
    )
    assert detection.left_clearance > 0.0


def test_negative_attitude_touches_left_foot_first() -> None:
    detector = create_detector()
    theta = math.radians(-10.0)

    left_touchdown_cg_height = (
        CG_TO_FEET * math.cos(theta)
        - 0.5 * FOOT_SPACING * math.sin(theta)
    )

    detection = detector.detect(
        create_state(
            pz=left_touchdown_cg_height,
            theta=theta,
        )
    )

    assert detection.contact is TouchdownContact2D.LEFT
    assert detection.is_touchdown

    assert detection.left_in_contact
    assert not detection.right_in_contact

    assert detection.left_clearance == pytest.approx(
        0.0
    )
    assert detection.right_clearance > 0.0


def test_both_feet_are_in_contact_below_ground() -> None:
    detector = create_detector()

    detection = detector.detect(
        create_state(
            pz=0.7,
        )
    )

    assert detection.contact is TouchdownContact2D.BOTH
    assert detection.left_clearance < 0.0
    assert detection.right_clearance < 0.0

    assert detection.minimum_clearance == pytest.approx(
        -0.1
    )


def test_ground_height_is_respected() -> None:
    detector = create_detector(
        ground_height=1.5,
    )

    detection = detector.detect(
        create_state(
            pz=1.5 + CG_TO_FEET,
        )
    )

    assert detection.contact is TouchdownContact2D.BOTH
    assert detection.left_clearance == pytest.approx(
        0.0
    )
    assert detection.right_clearance == pytest.approx(
        0.0
    )


def test_horizontal_translation_does_not_change_touchdown() -> None:
    detector = create_detector()

    reference = detector.detect(
        create_state(
            px=0.0,
            pz=CG_TO_FEET,
        )
    )

    translated = detector.detect(
        create_state(
            px=100.0,
            pz=CG_TO_FEET,
        )
    )

    assert reference.contact is TouchdownContact2D.BOTH
    assert translated.contact is TouchdownContact2D.BOTH

    assert translated.left_clearance == pytest.approx(
        reference.left_clearance
    )
    assert translated.right_clearance == pytest.approx(
        reference.right_clearance
    )


def test_detection_exposes_geometry_positions() -> None:
    detector = create_detector()

    state = create_state(
        px=2.0,
        pz=1.0,
        theta=math.radians(5.0),
    )

    detection = detector.detect(state)

    expected = detector.geometry.foot_positions_world(
        state
    )

    assert detection.foot_positions == expected


def test_detector_rejects_non_finite_ground_height() -> None:
    geometry = LandingGearGeometry2D(
        foot_spacing=FOOT_SPACING,
        cg_to_feet=CG_TO_FEET,
    )

    with pytest.raises(ValueError):
        TouchdownDetector2D(
            geometry,
            ground_height=float("nan"),
        )
