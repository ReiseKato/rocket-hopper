import pytest

from fast_sim.control import (
    design_pd_horizontal_position_gains,
)


def test_horizontal_position_design_matches_expected_gains() -> None:
    parameters = (
        design_pd_horizontal_position_gains(
            natural_frequency=0.8,
            damping_ratio=0.9,
        )
    )

    assert parameters.proportional_gain == pytest.approx(
        0.64
    )

    assert parameters.derivative_gain == pytest.approx(
        1.44
    )


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("natural_frequency", 0.0),
        ("damping_ratio", 0.0),
    ],
)
def test_horizontal_position_design_rejects_non_positive_inputs(
    name: str,
    value: float,
) -> None:
    arguments = {
        "natural_frequency": 0.8,
        "damping_ratio": 0.9,
    }

    arguments[name] = value

    with pytest.raises(ValueError):
        design_pd_horizontal_position_gains(
            **arguments
        )
