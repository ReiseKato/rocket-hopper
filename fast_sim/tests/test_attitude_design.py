import pytest

from fast_sim.control import (
    design_pd_attitude_gains,
)


def test_second_order_design_matches_expected_gains() -> None:
    parameters = design_pd_attitude_gains(
        inertia_y=0.08,
        engine_offset=0.25,
        nominal_thrust=19.62,
        natural_frequency=4.0,
        damping_ratio=0.8,
    )

    assert parameters.proportional_gain == pytest.approx(
        0.2609582059123344
    )

    assert parameters.derivative_gain == pytest.approx(
        0.10438328236493374
    )


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("inertia_y", 0.0),
        ("engine_offset", 0.0),
        ("nominal_thrust", 0.0),
        ("natural_frequency", 0.0),
        ("damping_ratio", 0.0),
    ],
)
def test_second_order_design_rejects_non_positive_inputs(
    name: str,
    value: float,
) -> None:
    arguments = {
        "inertia_y": 0.08,
        "engine_offset": 0.25,
        "nominal_thrust": 19.62,
        "natural_frequency": 4.0,
        "damping_ratio": 0.8,
    }

    arguments[name] = value

    with pytest.raises(ValueError):
        design_pd_attitude_gains(**arguments)
