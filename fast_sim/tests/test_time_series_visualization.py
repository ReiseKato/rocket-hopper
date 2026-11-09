import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from fast_sim.models import (
    HopperActuation2D,
    HopperActuationCommand2D,
    HopperState2D,
)
from fast_sim.simulation import SimulationResult
from fast_sim.visualization import (
    plot_actuation_comparisons,
    plot_basic_time_series,
    plot_simulation_overview,
)


def create_state(
    px: float,
    pz: float,
    vx: float = 0.0,
    vz: float = 0.0,
) -> HopperState2D:
    return HopperState2D(
        px=px,
        pz=pz,
        vx=vx,
        vz=vz,
        theta=0.0,
        omega=0.0,
    )


def create_result() -> SimulationResult:
    return SimulationResult(
        times=(0.0, 0.1, 0.2),
        rigid_body_states=(
            create_state(
                0.0,
                2.0,
                vx=0.0,
                vz=0.0,
            ),
            create_state(
                0.1,
                1.9,
                vx=1.0,
                vz=-1.0,
            ),
            create_state(
                0.2,
                1.8,
                vx=1.0,
                vz=-1.0,
            ),
        ),
        commands=(
            HopperActuationCommand2D(
                thrust=25.0,
                gimbal_angle=0.2,
            ),
            HopperActuationCommand2D(
                thrust=25.0,
                gimbal_angle=0.2,
            ),
        ),
        physical_actuations=(
            HopperActuation2D(
                thrust=20.0,
                gimbal_angle=0.05,
            ),
            HopperActuation2D(
                thrust=20.0,
                gimbal_angle=0.10,
            ),
        ),
    )


def test_plot_basic_time_series_returns_expected_figures() -> None:
    figures = plot_basic_time_series(
        create_result()
    )

    assert set(figures.keys()) == {
        "px",
        "pz",
        "vx",
        "vz",
        "theta",
        "omega",
        "thrust",
        "gimbal_angle",
    }

    for figure in figures.values():
        assert figure is not None

    plt.close("all")


def test_plot_actuation_comparisons_returns_expected_figures() -> None:
    figures = plot_actuation_comparisons(
        create_result()
    )

    assert set(figures.keys()) == {
        "thrust_comparison",
        "gimbal_angle_comparison",
    }

    for figure in figures.values():
        assert figure is not None

    plt.close("all")


def test_gimbal_comparison_contains_command_and_physical_series() -> None:
    figures = plot_actuation_comparisons(
        create_result()
    )

    axis = figures[
        "gimbal_angle_comparison"
    ].axes[0]

    legend = axis.get_legend()

    assert legend is not None

    labels = [
        text.get_text()
        for text in legend.get_texts()
    ]

    assert labels == [
        "Command",
        "Physical",
    ]

    plt.close("all")


def test_simulation_overview_contains_eight_subplots() -> None:
    figure = plot_simulation_overview(
        create_result()
    )

    assert len(figure.axes) == 8

    titles = [
        axis.get_title()
        for axis in figure.axes
    ]

    assert titles == [
        "Horizontal Position",
        "Vertical Position",
        "Horizontal Velocity",
        "Vertical Velocity",
        "Vehicle Angle",
        "Angular Velocity",
        "Commanded vs. Physical Thrust",
        "Commanded vs. Physical Gimbal Angle",
    ]

    plt.close("all")


def test_simulation_overview_contains_actuation_legends() -> None:
    figure = plot_simulation_overview(
        create_result()
    )

    for axis in figure.axes[-2:]:
        legend = axis.get_legend()

        assert legend is not None

        labels = [
            text.get_text()
            for text in legend.get_texts()
        ]

        assert labels == [
            "Command",
            "Physical",
        ]

    plt.close("all")
