"""Visualizations for complete 2D landing scenarios."""

from math import degrees

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing import (
    LandingDynamicsMode2D,
    LandingEvaluation2D,
    LandingPhase2D,
    LandingRunResult2D,
    SimultaneousLandingEvaluation2D,
)
from fast_sim.simulation import SimulationResult


def plot_landing_scenario_overview(
    landing_result: LandingRunResult2D,
    *,
    geometry: LandingGearGeometry2D,
    landing_target_x: float,
    title: str,
    ground_height: float = 0.0,
    pre_landing_result: SimulationResult | None = None,
    pre_landing_phase_label: str | None = None,
    pre_landing_target_x: float | None = None,
    pre_landing_target_z: float | None = None,
) -> Figure:
    """Plot trajectory, phases, states, contact speeds, and impact energy.

    `pre_landing_result` is optional. It is intended for scenario setup
    phases such as stabilization or a climb/transfer maneuver. It is not
    a mission-state abstraction.
    """
    figure, axes = plt.subplots(
        4,
        2,
        figsize=(14.0, 15.0),
        constrained_layout=True,
    )

    _plot_trajectory(
        axes[0, 0],
        landing_result,
        geometry=geometry,
        landing_target_x=landing_target_x,
        ground_height=ground_height,
        pre_landing_result=pre_landing_result,
        pre_landing_phase_label=pre_landing_phase_label,
        pre_landing_target_x=pre_landing_target_x,
        pre_landing_target_z=pre_landing_target_z,
    )

    _plot_phase_timeline(
        axes[0, 1],
        landing_result,
        pre_landing_result=pre_landing_result,
        pre_landing_phase_label=pre_landing_phase_label,
    )

    times, states = _combined_state_history(
        pre_landing_result,
        landing_result,
    )

    landing_start_time = landing_result.start_time

    contact_times = _contact_times(
        landing_result
    )

    _plot_state_series(
        axes[1, 0],
        times,
        [state.px for state in states],
        ylabel="px [m]",
        title="Horizontal position",
        landing_start_time=landing_start_time,
        contact_times=contact_times,
    )

    _plot_state_series(
        axes[1, 1],
        times,
        [state.pz for state in states],
        ylabel="pz [m]",
        title="Vertical position",
        landing_start_time=landing_start_time,
        contact_times=contact_times,
    )

    _plot_state_series(
        axes[2, 0],
        times,
        [state.vx for state in states],
        ylabel="vx [m/s]",
        title="Horizontal velocity",
        landing_start_time=landing_start_time,
        contact_times=contact_times,
    )

    _plot_state_series(
        axes[2, 1],
        times,
        [degrees(state.theta) for state in states],
        ylabel="theta [deg]",
        title="Vehicle attitude",
        landing_start_time=landing_start_time,
        contact_times=contact_times,
    )

    _plot_impact_speeds(
        axes[3, 0],
        landing_result,
    )

    _plot_impact_energies(
        axes[3, 1],
        landing_result,
    )

    sequence = (
        landing_result.contact_sequence.value
        if landing_result.contact_sequence is not None
        else "not completed"
    )

    status = (
        "successful"
        if landing_result.successful
        else "not successful"
    )

    figure.suptitle(
        f"{title}\n"
        f"contact: {sequence} — {status}"
    )

    return figure


def _plot_trajectory(
    axis,
    landing_result: LandingRunResult2D,
    *,
    geometry: LandingGearGeometry2D,
    landing_target_x: float,
    ground_height: float,
    pre_landing_result: SimulationResult | None,
    pre_landing_phase_label: str | None,
    pre_landing_target_x: float | None,
    pre_landing_target_z: float | None,
) -> None:
    if pre_landing_result is not None:
        pre_states = pre_landing_result.rigid_body_states

        axis.plot(
            [state.px for state in pre_states],
            [state.pz for state in pre_states],
            label=(
                pre_landing_phase_label
                or "Pre-landing"
            ),
        )

    landing_states = landing_result.rigid_body_states

    axis.plot(
        [state.px for state in landing_states],
        [state.pz for state in landing_states],
        label="Landing",
    )

    if (
        pre_landing_target_x is not None
        and pre_landing_target_z is not None
    ):
        axis.scatter(
            [pre_landing_target_x],
            [pre_landing_target_z],
            marker="x",
            label="Pre-landing target",
        )

    axis.axhline(
        ground_height,
        linestyle="--",
        label="Ground",
    )

    axis.axvline(
        landing_target_x,
        linestyle=":",
        label="Landing target x",
    )

    if landing_result.first_contact is not None:
        first_state = _state_at_time(
            landing_result,
            landing_result.first_contact.time,
        )

        feet = geometry.foot_positions_world(
            first_state
        )

        if (
            landing_result.first_contact.foot.value
            == "left"
        ):
            contact_x = feet.left_x
            contact_z = feet.left_z
        else:
            contact_x = feet.right_x
            contact_z = feet.right_z

        axis.scatter(
            [contact_x],
            [contact_z],
            marker="o",
            label="First contact",
        )

    if landing_result.completed:
        final_feet = geometry.foot_positions_world(
            landing_result.final_rigid_body_state
        )

        axis.scatter(
            [
                final_feet.left_x,
                final_feet.right_x,
            ],
            [
                final_feet.left_z,
                final_feet.right_z,
            ],
            marker="s",
            label="Completed contact",
        )

    axis.set_title("CG trajectory and ground contacts")
    axis.set_xlabel("px [m]")
    axis.set_ylabel("pz [m]")
    axis.grid(True)
    axis.legend()


def _plot_phase_timeline(
    axis,
    landing_result: LandingRunResult2D,
    *,
    pre_landing_result: SimulationResult | None,
    pre_landing_phase_label: str | None,
) -> None:
    tick_values: list[float] = []
    tick_labels: list[str] = []

    if pre_landing_result is not None:
        pre_times = np.asarray(
            pre_landing_result.times,
            dtype=float,
        )

        axis.step(
            pre_times,
            np.zeros_like(pre_times),
            where="post",
        )

        tick_values.append(0.0)
        tick_labels.append(
            pre_landing_phase_label
            or "Pre-landing"
        )

    phase_values = {
        LandingPhase2D.APPROACH: 1.0,
        LandingPhase2D.FLARE: 2.0,
        LandingPhase2D.FINAL: 3.0,
    }

    landing_times = np.asarray(
        landing_result.times,
        dtype=float,
    )

    landing_values: list[float] = []

    for index, mode in enumerate(
        landing_result.modes
    ):
        if mode is LandingDynamicsMode2D.COMPLETED:
            landing_values.append(5.0)
            continue

        if mode is LandingDynamicsMode2D.SINGLE_CONTACT:
            landing_values.append(4.0)
            continue

        if landing_result.guidance_references:
            reference_index = min(
                index,
                len(
                    landing_result.guidance_references
                )
                - 1,
            )

            reference = (
                landing_result
                .guidance_references[
                    reference_index
                ]
            )

            if reference is None:
                landing_values.append(1.0)
            else:
                landing_values.append(
                    phase_values[
                        reference.phase
                    ]
                )
        else:
            landing_values.append(1.0)

    axis.step(
        landing_times,
        np.asarray(
            landing_values,
            dtype=float,
        ),
        where="post",
    )

    tick_values.extend(
        [1.0, 2.0, 3.0, 4.0, 5.0]
    )

    tick_labels.extend(
        [
            "Landing / Approach",
            "Landing / Flare",
            "Landing / Final",
            "Single contact",
            "Completed",
        ]
    )

    first_time, final_time = _contact_times(
        landing_result
    )

    if first_time is not None:
        axis.axvline(
            first_time,
            linestyle=":",
            label="First contact",
        )

    if final_time is not None:
        axis.axvline(
            final_time,
            linestyle="--",
            label="Landing completed",
        )

    axis.set_yticks(
        tick_values,
        tick_labels,
    )

    axis.set_title(
        "Scenario and landing phases"
    )
    axis.set_xlabel("time [s]")
    axis.grid(True)

    if first_time is not None:
        axis.legend()


def _plot_state_series(
    axis,
    times: np.ndarray,
    values,
    *,
    ylabel: str,
    title: str,
    landing_start_time: float,
    contact_times: tuple[
        float | None,
        float | None,
    ],
) -> None:
    axis.plot(
        times,
        np.asarray(values, dtype=float),
    )

    if (
        times.size > 0
        and landing_start_time > times[0]
    ):
        axis.axvline(
            landing_start_time,
            linestyle="--",
            label="Landing guidance starts",
        )

    first_time, final_time = contact_times

    if first_time is not None:
        axis.axvline(
            first_time,
            linestyle=":",
            label="First contact",
        )

    if final_time is not None:
        axis.axvline(
            final_time,
            linestyle="-.",
            label="Completed",
        )

    axis.set_title(title)
    axis.set_xlabel("time [s]")
    axis.set_ylabel(ylabel)
    axis.grid(True)

    if (
        landing_start_time > times[0]
        or first_time is not None
    ):
        axis.legend()


def _plot_impact_speeds(
    axis,
    landing_result: LandingRunResult2D,
) -> None:
    evaluation = landing_result.evaluation

    if evaluation is None:
        axis.text(
            0.5,
            0.5,
            "No completed landing evaluation",
            ha="center",
            va="center",
        )
        axis.set_title("Foot impact speeds")
        return

    if isinstance(
        evaluation,
        LandingEvaluation2D,
    ):
        labels = [
            (
                "First / "
                f"{evaluation.first_contact.foot.value}"
            ),
            (
                "Second / "
                f"{evaluation.second_contact.foot.value}"
            ),
        ]

        normal = [
            evaluation.first_contact.normal_speed,
            evaluation.second_contact.normal_speed,
        ]

        tangential = [
            evaluation.first_contact.tangential_speed,
            evaluation.second_contact.tangential_speed,
        ]

    elif isinstance(
        evaluation,
        SimultaneousLandingEvaluation2D,
    ):
        labels = ["Left", "Right"]

        normal = [
            evaluation.contact.left_normal_speed,
            evaluation.contact.right_normal_speed,
        ]

        tangential = [
            evaluation.contact.left_tangential_speed,
            evaluation.contact.right_tangential_speed,
        ]

    else:
        raise TypeError(
            "unsupported landing evaluation type"
        )

    x = np.arange(
        len(labels),
        dtype=float,
    )

    width = 0.36

    axis.bar(
        x - width / 2.0,
        normal,
        width,
        label="Normal",
    )

    axis.bar(
        x + width / 2.0,
        tangential,
        width,
        label="Tangential",
    )

    axis.set_xticks(x, labels)
    axis.set_ylabel("speed [m/s]")
    axis.set_title(
        "Impact metric 1: foot speeds"
    )
    axis.grid(True, axis="y")
    axis.legend()


def _plot_impact_energies(
    axis,
    landing_result: LandingRunResult2D,
) -> None:
    evaluation = landing_result.evaluation

    if evaluation is None:
        axis.text(
            0.5,
            0.5,
            "No completed landing evaluation",
            ha="center",
            va="center",
        )
        axis.set_title("Impact energy")
        return

    if isinstance(
        evaluation,
        LandingEvaluation2D,
    ):
        labels = ["First", "Second"]

        values = [
            evaluation.first_contact.dissipated_energy,
            evaluation.second_contact.dissipated_energy,
        ]

    elif isinstance(
        evaluation,
        SimultaneousLandingEvaluation2D,
    ):
        labels = ["Simultaneous"]

        values = [
            evaluation.contact.dissipated_energy,
        ]

    else:
        raise TypeError(
            "unsupported landing evaluation type"
        )

    x = np.arange(
        len(labels),
        dtype=float,
    )

    axis.bar(
        x,
        values,
    )

    axis.set_xticks(
        x,
        labels,
    )

    axis.set_ylabel(
        "dissipated energy [J]"
    )

    axis.set_title(
        "Impact metric 2: idealized energy"
    )

    axis.grid(
        True,
        axis="y",
    )


def _combined_state_history(
    pre_landing_result: SimulationResult | None,
    landing_result: LandingRunResult2D,
) -> tuple[np.ndarray, tuple]:
    if pre_landing_result is None:
        return (
            np.asarray(
                landing_result.times,
                dtype=float,
            ),
            tuple(
                landing_result.rigid_body_states
            ),
        )

    times = (
        tuple(pre_landing_result.times)
        + tuple(
            landing_result.times[1:]
        )
    )

    states = (
        tuple(
            pre_landing_result
            .rigid_body_states
        )
        + tuple(
            landing_result
            .rigid_body_states[1:]
        )
    )

    return (
        np.asarray(
            times,
            dtype=float,
        ),
        states,
    )


def _contact_times(
    landing_result: LandingRunResult2D,
) -> tuple[
    float | None,
    float | None,
]:
    evaluation = landing_result.evaluation

    if isinstance(
        evaluation,
        LandingEvaluation2D,
    ):
        return (
            evaluation.first_contact.time,
            evaluation.second_contact.time,
        )

    if isinstance(
        evaluation,
        SimultaneousLandingEvaluation2D,
    ):
        return (
            evaluation.contact.time,
            evaluation.contact.time,
        )

    if landing_result.first_contact is not None:
        return (
            landing_result.first_contact.time,
            None,
        )

    return None, None


def _state_at_time(
    landing_result: LandingRunResult2D,
    time: float,
):
    index = int(
        np.argmin(
            np.abs(
                np.asarray(
                    landing_result.times,
                    dtype=float,
                )
                - time
            )
        )
    )

    return (
        landing_result
        .rigid_body_states[index]
    )
