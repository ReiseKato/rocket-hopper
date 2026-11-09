import math

import pytest

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.models import HopperState2D


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


def test_upright_foot_positions_are_symmetric() -> None:
    geometry = LandingGearGeometry2D(
        foot_spacing=0.6,
        cg_to_feet=0.8,
    )

    positions = geometry.foot_positions_world(
        create_state()
    )

    assert positions.left_x == pytest.approx(
        -0.3
    )
    assert positions.right_x == pytest.approx(
        0.3
    )

    assert positions.left_z == pytest.approx(
        1.2
    )
    assert positions.right_z == pytest.approx(
        1.2
    )


def test_world_translation_moves_both_feet() -> None:
    geometry = LandingGearGeometry2D(
        foot_spacing=0.6,
        cg_to_feet=0.8,
    )

    positions = geometry.foot_positions_world(
        create_state(
            px=3.0,
            pz=4.0,
        )
    )

    assert positions.left_x == pytest.approx(
        2.7
    )
    assert positions.right_x == pytest.approx(
        3.3
    )

    assert positions.left_z == pytest.approx(
        3.2
    )
    assert positions.right_z == pytest.approx(
        3.2
    )


def test_positive_attitude_places_right_foot_lower() -> None:
    geometry = LandingGearGeometry2D(
        foot_spacing=0.6,
        cg_to_feet=0.8,
    )

    positions = geometry.foot_positions_world(
        create_state(
            theta=math.radians(10.0),
        )
    )

    assert positions.right_z < positions.left_z


def test_positive_attitude_matches_expected_world_positions() -> None:
    foot_spacing = 0.6
    cg_to_feet = 0.8
    theta = math.radians(10.0)

    geometry = LandingGearGeometry2D(
        foot_spacing=foot_spacing,
        cg_to_feet=cg_to_feet,
    )

    positions = geometry.foot_positions_world(
        create_state(
            px=1.0,
            pz=2.0,
            theta=theta,
        )
    )

    half_spacing = foot_spacing / 2.0

    expected_left_x = (
        1.0
        - half_spacing * math.cos(theta)
        - cg_to_feet * math.sin(theta)
    )
    expected_left_z = (
        2.0
        + half_spacing * math.sin(theta)
        - cg_to_feet * math.cos(theta)
    )

    expected_right_x = (
        1.0
        + half_spacing * math.cos(theta)
        - cg_to_feet * math.sin(theta)
    )
    expected_right_z = (
        2.0
        - half_spacing * math.sin(theta)
        - cg_to_feet * math.cos(theta)
    )

    assert positions.left_x == pytest.approx(
        expected_left_x
    )
    assert positions.left_z == pytest.approx(
        expected_left_z
    )

    assert positions.right_x == pytest.approx(
        expected_right_x
    )
    assert positions.right_z == pytest.approx(
        expected_right_z
    )


def test_lowest_z_returns_lower_foot_height() -> None:
    geometry = LandingGearGeometry2D(
        foot_spacing=0.6,
        cg_to_feet=0.8,
    )

    positions = geometry.foot_positions_world(
        create_state(
            theta=math.radians(10.0),
        )
    )

    assert positions.lowest_z == pytest.approx(
        positions.right_z
    )


@pytest.mark.parametrize(
    ("field_name", "value"),
    [
        ("foot_spacing", 0.0),
        ("foot_spacing", -0.1),
        ("cg_to_feet", 0.0),
        ("cg_to_feet", -0.1),
    ],
)
def test_geometry_rejects_non_positive_dimensions(
    field_name: str,
    value: float,
) -> None:
    arguments = {
        "foot_spacing": 0.6,
        "cg_to_feet": 0.8,
    }

    arguments[field_name] = value

    with pytest.raises(ValueError):
        LandingGearGeometry2D(
            **arguments
        )
