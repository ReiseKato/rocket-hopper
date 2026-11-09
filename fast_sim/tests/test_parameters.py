"""Tests for rigid-body parameter validation."""

import pytest

from fast_sim import RigidBody2DParameters


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("mass", 0.0),
        ("mass", -1.0),
        ("inertia_y", 0.0),
        ("inertia_y", -1.0),
        ("engine_offset", -0.1),
        # ("gravity", 0.0),
        ("gravity", -9.81),
    ],
)
def test_invalid_parameters_are_rejected(field: str, value: float) -> None:
    kwargs = {
        "mass": 2.0,
        "inertia_y": 0.08,
        "engine_offset": 0.25,
        "gravity": 9.81,
    }
    kwargs[field] = value

    with pytest.raises(ValueError):
        RigidBody2DParameters(**kwargs)
