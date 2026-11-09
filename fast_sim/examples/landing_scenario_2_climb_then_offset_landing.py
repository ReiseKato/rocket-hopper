"""Scenario 2: climb to a point, then immediately land farther in x."""

import matplotlib.pyplot as plt

from fast_sim.models import HopperState2D
from fast_sim.visualization.landing_scenarios import (
    plot_landing_scenario_overview,
)

from landing_scenario_support import (
    build_landing_scenario_components,
    print_landing_summary,
    run_landing,
    run_until_position_reached,
)


STARTING_X = 0.0
STARTING_Z = 1.15

TRANSFER_X = 3.8
TRANSFER_Z = 10.0

LANDING_X = 5.5


def run_scenario(
    *,
    show_plot: bool = True,
):
    """Run a climb/transfer followed by a non-vertical landing.

    The handoff to landing guidance occurs on the first positional reach.
    There is deliberately no settling or velocity dwell at the transfer
    point.
    """
    components = (
        build_landing_scenario_components(
            HopperState2D(
                px=STARTING_X,
                pz=STARTING_Z,
                vx=0.0,
                vz=0.0,
                theta=0.0,
                omega=0.0,
            )
        )
    )

    pre_landing_result = (
        run_until_position_reached(
            components,
            target_x=TRANSFER_X,
            target_z=TRANSFER_Z,
            max_duration=60.0,
            position_tolerance=0.12,
            altitude_tolerance=0.12,
        )
    )

    landing_result = run_landing(
        components,
        landing_target_x=LANDING_X,
        approach_descent_speed=0.28,
        flare_height=0.60,
        touchdown_descent_speed=0.06,
        max_step_count=250000,
    )

    print_landing_summary(
        scenario_name=(
            "Scenario 2 — climb then offset landing"
        ),
        pre_landing_result=(
            pre_landing_result
        ),
        landing_result=landing_result,
    )

    figure = plot_landing_scenario_overview(
        landing_result,
        geometry=components.geometry,
        landing_target_x=LANDING_X,
        title=(
            "Scenario 2 — climb then offset landing"
        ),
        pre_landing_result=(
            pre_landing_result
        ),
        pre_landing_phase_label=(
            "Climb / transfer — no settling"
        ),
        pre_landing_target_x=TRANSFER_X,
        pre_landing_target_z=TRANSFER_Z,
    )

    if show_plot:
        plt.show()

    return (
        pre_landing_result,
        landing_result,
        figure,
    )


def main() -> None:
    run_scenario(
        show_plot=True
    )


if __name__ == "__main__":
    main()
