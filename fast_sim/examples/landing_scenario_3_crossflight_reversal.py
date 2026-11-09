"""Scenario 3: reverse a leftward cross-flight and land to the right."""

from math import radians

import matplotlib.pyplot as plt

from fast_sim.models import HopperState2D
from fast_sim.visualization.landing_scenarios import (
    plot_landing_scenario_overview,
)

from landing_scenario_support import (
    build_landing_scenario_components,
    print_landing_summary,
    run_landing,
)


LANDING_X = 1.5


def run_scenario(
    *,
    show_plot: bool = True,
):
    """Land to the right from a leftward, nose-left initial cross-flight."""
    components = (
        build_landing_scenario_components(
            HopperState2D(
                px=0.0,
                pz=3.2,
                vx=-1.0,
                vz=0.0,
                theta=radians(-12.0),
                omega=0.0,
            )
        )
    )

    landing_result = run_landing(
        components,
        landing_target_x=LANDING_X,
        approach_descent_speed=0.20,
        flare_height=0.70,
        touchdown_descent_speed=0.06,
        max_step_count=3500,
    )

    print_landing_summary(
        scenario_name=(
            "Scenario 3 — cross-flight reversal landing"
        ),
        landing_result=landing_result,
    )

    figure = plot_landing_scenario_overview(
        landing_result,
        geometry=components.geometry,
        landing_target_x=LANDING_X,
        title=(
            "Scenario 3 — cross-flight reversal landing"
        ),
        pre_landing_result=None,
        pre_landing_phase_label=None,
    )

    if show_plot:
        plt.show()

    return (
        landing_result,
        figure,
    )


def main() -> None:
    run_scenario(
        show_plot=True
    )


if __name__ == "__main__":
    main()
