import pytest

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing import (
    LandingGuidanceParameters2D,
    LandingPhase2D,
    PiecewiseLandingGuidance2D,
)
from fast_sim.models import HopperState2D


GROUND_HEIGHT = 0.0
CG_TO_FEET = 0.8
INITIAL_ALTITUDE = 2.0

APPROACH_SPEED = 0.5
FLARE_HEIGHT = 0.5
TOUCHDOWN_SPEED = 0.1


def create_state(
    *,
    pz: float = INITIAL_ALTITUDE,
) -> HopperState2D:
    return HopperState2D(
        px=0.0,
        pz=pz,
        vx=0.0,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )


def create_guidance(
    *,
    target_x: float = 1.0,
    initial_altitude: float = INITIAL_ALTITUDE,
    start_time: float = 0.0,
) -> PiecewiseLandingGuidance2D:
    return PiecewiseLandingGuidance2D(
        LandingGuidanceParameters2D(
            target_x=target_x,
            ground_height=GROUND_HEIGHT,
            approach_descent_speed=APPROACH_SPEED,
            flare_height=FLARE_HEIGHT,
            touchdown_descent_speed=TOUCHDOWN_SPEED,
        ),
        geometry=LandingGearGeometry2D(
            foot_spacing=0.6,
            cg_to_feet=CG_TO_FEET,
        ),
        initial_state=create_state(
            pz=initial_altitude
        ),
        start_time=start_time,
    )


def test_touchdown_altitude_uses_ground_and_landing_geometry() -> None:
    guidance = create_guidance()

    assert guidance.touchdown_altitude == pytest.approx(
        GROUND_HEIGHT + CG_TO_FEET
    )


def test_horizontal_reference_remains_at_landing_target() -> None:
    guidance = create_guidance(
        target_x=1.25
    )

    for time in (
        0.0,
        guidance.flare_start_time,
        guidance.final_reference_time,
    ):
        output = guidance.compute_references(
            time,
            create_state(),
        )

        assert (
            output.horizontal_position.px
            == pytest.approx(1.25)
        )
        assert (
            output.horizontal_position.vx
            == pytest.approx(0.0)
        )


def test_approach_reference_descends_at_constant_speed() -> None:
    guidance = create_guidance()

    time = 0.5

    output = guidance.compute_references(
        time,
        create_state(),
    )

    assert output.phase is LandingPhase2D.APPROACH

    assert output.altitude.pz == pytest.approx(
        INITIAL_ALTITUDE
        - APPROACH_SPEED * time
    )

    assert output.altitude.vz == pytest.approx(
        -APPROACH_SPEED
    )


def test_flare_starts_continuously_from_approach() -> None:
    guidance = create_guidance()

    output = guidance.compute_references(
        guidance.flare_start_time,
        create_state(),
    )

    assert output.phase is LandingPhase2D.FLARE

    assert output.altitude.pz == pytest.approx(
        guidance.flare_start_altitude
    )

    assert output.altitude.vz == pytest.approx(
        -APPROACH_SPEED
    )


def test_flare_reduces_descent_speed() -> None:
    guidance = create_guidance()

    midpoint_time = (
        guidance.flare_start_time
        + 0.5 * guidance.flare_duration
    )

    output = guidance.compute_references(
        midpoint_time,
        create_state(),
    )

    assert output.phase is LandingPhase2D.FLARE

    assert (
        -APPROACH_SPEED
        < output.altitude.vz
        < -TOUCHDOWN_SPEED
    )


def test_final_reference_uses_touchdown_altitude_and_speed() -> None:
    guidance = create_guidance()

    output = guidance.compute_references(
        guidance.final_reference_time,
        create_state(),
    )

    assert output.phase is LandingPhase2D.FINAL

    assert output.altitude.pz == pytest.approx(
        guidance.touchdown_altitude
    )

    assert output.altitude.vz == pytest.approx(
        -TOUCHDOWN_SPEED
    )


def test_flare_profile_remains_above_touchdown_altitude() -> None:
    guidance = create_guidance()

    for fraction in (
        0.0,
        0.25,
        0.5,
        0.75,
        0.999,
    ):
        time = (
            guidance.flare_start_time
            + fraction * guidance.flare_duration
        )

        output = guidance.compute_references(
            time,
            create_state(),
        )

        assert (
            output.altitude.pz
            >= guidance.touchdown_altitude
        )


def test_guidance_can_start_inside_nominal_flare_region() -> None:
    initial_altitude = 1.1

    guidance = create_guidance(
        initial_altitude=initial_altitude
    )

    assert guidance.approach_duration == pytest.approx(
        0.0
    )

    assert guidance.flare_start_altitude == pytest.approx(
        initial_altitude
    )

    output = guidance.compute_references(
        0.0,
        create_state(
            pz=initial_altitude
        ),
    )

    assert output.phase is LandingPhase2D.FLARE
    assert output.altitude.pz == pytest.approx(
        initial_altitude
    )
    assert output.altitude.vz == pytest.approx(
        -APPROACH_SPEED
    )


def test_guidance_rejects_initial_altitude_at_touchdown() -> None:
    with pytest.raises(ValueError):
        create_guidance(
            initial_altitude=CG_TO_FEET
        )


def test_guidance_rejects_time_before_start() -> None:
    guidance = create_guidance(
        start_time=5.0
    )

    with pytest.raises(ValueError):
        guidance.compute_references(
            4.99,
            create_state(),
        )


@pytest.mark.parametrize(
    ("field_name", "value"),
    [
        ("approach_descent_speed", 0.0),
        ("flare_height", 0.0),
        ("touchdown_descent_speed", -0.1),
    ],
)
def test_parameters_reject_invalid_values(
    field_name: str,
    value: float,
) -> None:
    arguments = {
        "target_x": 1.0,
        "ground_height": 0.0,
        "approach_descent_speed": 0.5,
        "flare_height": 0.5,
        "touchdown_descent_speed": 0.1,
    }

    arguments[field_name] = value

    with pytest.raises(ValueError):
        LandingGuidanceParameters2D(
            **arguments
        )


def test_touchdown_speed_cannot_exceed_approach_speed() -> None:
    with pytest.raises(ValueError):
        LandingGuidanceParameters2D(
            target_x=1.0,
            ground_height=0.0,
            approach_descent_speed=0.5,
            flare_height=0.5,
            touchdown_descent_speed=0.6,
        )
