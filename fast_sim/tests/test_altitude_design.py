import pytest

from fast_sim.control import (
    design_pd_altitude_gains,
)


def test_second_order_altitude_design_matches_expected_gains() -> None:
    parameters = design_pd_altitude_gains(
        mass=2.0,
        natural_frequency=2.0,
        damping_ratio=0.9,
    )

    assert parameters.proportional_gain == pytest.approx(
        8.0
    )

    assert parameters.derivative_gain == pytest.approx(
        7.2
    )


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("mass", 0.0),
        ("natural_frequency", 0.0),
        ("damping_ratio", 0.0),
    ],
)
def test_altitude_design_rejects_non_positive_inputs(
    name: str,
    value: float,
) -> None:
    arguments = {
        "mass": 2.0,
        "natural_frequency": 2.0,
        "damping_ratio": 0.9,
    }

    arguments[name] = value

    with pytest.raises(ValueError):
        design_pd_altitude_gains(**arguments)
