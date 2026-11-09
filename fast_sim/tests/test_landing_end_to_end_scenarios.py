import importlib.util
import sys
from math import radians
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


EXAMPLES_DIRECTORY = (
    Path(__file__)
    .resolve()
    .parents[1]
    / "examples"
)

sys.path.insert(
    0,
    str(EXAMPLES_DIRECTORY),
)


def load_example_module(
    filename: str,
):
    path = (
        EXAMPLES_DIRECTORY
        / filename
    )

    spec = (
        importlib.util
        .spec_from_file_location(
            path.stem,
            path,
        )
    )

    if (
        spec is None
        or spec.loader is None
    ):
        raise RuntimeError(
            f"cannot import {path}"
        )

    module = (
        importlib.util
        .module_from_spec(spec)
    )

    spec.loader.exec_module(
        module
    )

    return module


def test_scenario_1_stabilizes_then_lands_vertically() -> None:
    module = load_example_module(
        "landing_scenario_1_stabilized_vertical.py"
    )

    pre_result, landing_result, figure = (
        module.run_scenario(
            show_plot=False
        )
    )

    handoff = (
        pre_result.final_rigid_body_state
    )

    assert abs(
        handoff.px
        - module.STAGING_X
    ) <= 0.04

    assert abs(
        handoff.pz
        - module.STAGING_Z
    ) <= 0.04

    assert abs(handoff.vx) <= 0.05
    assert abs(handoff.vz) <= 0.05
    assert abs(handoff.theta) <= radians(1.0)

    assert landing_result.completed
    assert landing_result.successful

    assert abs(
        landing_result
        .final_rigid_body_state
        .px
        - module.LANDING_X
    ) <= 0.20

    assert len(figure.axes) == 8

    plt.close(figure)


def test_scenario_2_hands_off_without_settling_and_lands_farther_in_x() -> None:
    module = load_example_module(
        "landing_scenario_2_climb_then_offset_landing.py"
    )

    pre_result, landing_result, figure = (
        module.run_scenario(
            show_plot=False
        )
    )

    handoff = (
        pre_result.final_rigid_body_state
    )

    assert (
        module.LANDING_X
        > module.TRANSFER_X
    )

    assert (
        abs(handoff.vx) > 0.05
        or abs(handoff.vz) > 0.01
        or abs(handoff.omega) > radians(1.0)
    )

    assert landing_result.completed
    assert landing_result.successful

    assert abs(
        landing_result
        .final_rigid_body_state
        .px
        - module.LANDING_X
    ) <= 0.20

    assert len(figure.axes) == 8

    plt.close(figure)


def test_scenario_3_reverses_leftward_crossflight_and_lands_right() -> None:
    module = load_example_module(
        "landing_scenario_3_crossflight_reversal.py"
    )

    landing_result, figure = (
        module.run_scenario(
            show_plot=False
        )
    )

    initial_state = (
        landing_result
        .initial_rigid_body_state
    )

    assert initial_state.vx < 0.0
    assert initial_state.theta <= radians(-10.0)

    assert (
        module.LANDING_X
        > initial_state.px
    )

    assert landing_result.completed
    assert landing_result.successful

    assert abs(
        landing_result
        .final_rigid_body_state
        .px
        - module.LANDING_X
    ) <= 0.20

    assert len(figure.axes) == 8

    plt.close(figure)
