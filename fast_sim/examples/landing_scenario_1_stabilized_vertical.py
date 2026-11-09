"""Scenario 1: stabilize several meters above the pad, then land vertically."""

import matplotlib.pyplot as plt

from fast_sim.models import HopperState2D
from fast_sim.visualization.landing_scenarios import (
    plot_landing_scenario_overview,
)

from landing_scenario_support import (
    build_landing_scenario_components,
    print_landing_summary,
    run_landing,
    run_until_settled,
)


STARTING_X = -1.0
STARTING_Z = 3.5

STAGING_X = 5.5
STAGING_Z = 15.5

LANDING_X = STAGING_X


def run_scenario(
    *,
    show_plot: bool = True,
):
    """Run the stabilized vertical landing scenario."""
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

    pre_landing_result = run_until_settled(
        components,
        target_x=STAGING_X,
        target_z=STAGING_Z,
        max_duration=50.0,
    )

    landing_result = run_landing(
        components,
        landing_target_x=LANDING_X,
        approach_descent_speed=0.40,
        flare_height=0.60,
        touchdown_descent_speed=0.07,
        max_step_count=160000,
    )

    print_landing_summary(
        scenario_name=(
            "Scenario 1 — stabilized vertical landing"
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
            "Scenario 1 — stabilized vertical landing"
        ),
        pre_landing_result=(
            pre_landing_result
        ),
        pre_landing_phase_label=(
            "Stabilize above landing pad"
        ),
        pre_landing_target_x=STAGING_X,
        pre_landing_target_z=STAGING_Z,
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
