"""Time-series visualizations for simulation results."""

from math import degrees
from typing import TypeAlias

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from fast_sim.simulation import SimulationResult


FigureMap: TypeAlias = dict[str, Figure]


def plot_basic_time_series(
    result: SimulationResult,
) -> FigureMap:
    """Create the default set of individual simulation plots."""
    return {
        "px": plot_px_time_series(result),
        "pz": plot_pz_time_series(result),
        "vx": plot_vx_time_series(result),
        "vz": plot_vz_time_series(result),
        "theta": plot_theta_time_series(result),
        "omega": plot_omega_time_series(result),
        "thrust": plot_thrust_time_series(result),
        "gimbal_angle": plot_gimbal_angle_time_series(result),
    }


def plot_actuation_comparisons(
    result: SimulationResult,
) -> FigureMap:
    """Create command-versus-physical actuation comparison plots."""
    return {
        "thrust_comparison": plot_thrust_command_comparison(
            result
        ),
        "gimbal_angle_comparison": (
            plot_gimbal_angle_command_comparison(
                result
            )
        ),
    }


def plot_simulation_overview(
    result: SimulationResult,
) -> Figure:
    """Create one overview figure containing the key simulation signals.

    The overview contains:
        - horizontal and vertical position,
        - horizontal and vertical velocity,
        - attitude and angular velocity,
        - commanded versus physical thrust,
        - commanded versus physical gimbal angle.
    """
    figure, axes = plt.subplots(
        nrows=4,
        ncols=2,
        sharex=True,
        figsize=(14.0, 12.0),
        constrained_layout=True,
    )

    times = _state_times(result)

    _plot_state_on_axis(
        axis=axes[0, 0],
        times=times,
        values=_state_values(
            result,
            attribute="px",
        ),
        title="Horizontal Position",
        ylabel="px [m]",
    )

    _plot_state_on_axis(
        axis=axes[0, 1],
        times=times,
        values=_state_values(
            result,
            attribute="pz",
        ),
        title="Vertical Position",
        ylabel="pz [m]",
    )

    _plot_state_on_axis(
        axis=axes[1, 0],
        times=times,
        values=_state_values(
            result,
            attribute="vx",
        ),
        title="Horizontal Velocity",
        ylabel="vx [m/s]",
    )

    _plot_state_on_axis(
        axis=axes[1, 1],
        times=times,
        values=_state_values(
            result,
            attribute="vz",
        ),
        title="Vertical Velocity",
        ylabel="vz [m/s]",
    )

    _plot_state_on_axis(
        axis=axes[2, 0],
        times=times,
        values=_angle_state_values(
            result,
            attribute="theta",
        ),
        title="Vehicle Angle",
        ylabel="theta [deg]",
    )

    _plot_state_on_axis(
        axis=axes[2, 1],
        times=times,
        values=_angle_state_values(
            result,
            attribute="omega",
        ),
        title="Angular Velocity",
        ylabel="omega [deg/s]",
    )

    _plot_actuation_comparison_on_axis(
        axis=axes[3, 0],
        times=_actuation_times(result),
        command_values=np.array(
            [
                command.thrust
                for command in result.commands
            ],
            dtype=np.float64,
        ),
        physical_values=np.array(
            [
                actuation.thrust
                for actuation
                in result.physical_actuations
            ],
            dtype=np.float64,
        ),
        title="Commanded vs. Physical Thrust",
        ylabel="T [N]",
    )

    _plot_actuation_comparison_on_axis(
        axis=axes[3, 1],
        times=_actuation_times(result),
        command_values=np.array(
            [
                degrees(command.gimbal_angle)
                for command in result.commands
            ],
            dtype=np.float64,
        ),
        physical_values=np.array(
            [
                degrees(actuation.gimbal_angle)
                for actuation
                in result.physical_actuations
            ],
            dtype=np.float64,
        ),
        title="Commanded vs. Physical Gimbal Angle",
        ylabel="delta [deg]",
    )

    axes[3, 0].set_xlabel("time [s]")
    axes[3, 1].set_xlabel("time [s]")

    figure.suptitle(
        "Simulation Overview"
    )

    return figure


def plot_px_time_series(
    result: SimulationResult,
) -> Figure:
    """Plot horizontal position px over time."""
    return _create_state_plot(
        times=_state_times(result),
        values=_state_values(
            result,
            attribute="px",
        ),
        title="Horizontal Position",
        ylabel="px [m]",
    )


def plot_pz_time_series(
    result: SimulationResult,
) -> Figure:
    """Plot vertical position pz over time."""
    return _create_state_plot(
        times=_state_times(result),
        values=_state_values(
            result,
            attribute="pz",
        ),
        title="Vertical Position",
        ylabel="pz [m]",
    )


def plot_vx_time_series(
    result: SimulationResult,
) -> Figure:
    """Plot horizontal velocity vx over time."""
    return _create_state_plot(
        times=_state_times(result),
        values=_state_values(
            result,
            attribute="vx",
        ),
        title="Horizontal Velocity",
        ylabel="vx [m/s]",
    )


def plot_vz_time_series(
    result: SimulationResult,
) -> Figure:
    """Plot vertical velocity vz over time."""
    return _create_state_plot(
        times=_state_times(result),
        values=_state_values(
            result,
            attribute="vz",
        ),
        title="Vertical Velocity",
        ylabel="vz [m/s]",
    )


def plot_theta_time_series(
    result: SimulationResult,
) -> Figure:
    """Plot vehicle angle theta over time in degrees."""
    return _create_state_plot(
        times=_state_times(result),
        values=_angle_state_values(
            result,
            attribute="theta",
        ),
        title="Vehicle Angle",
        ylabel="theta [deg]",
    )


def plot_omega_time_series(
    result: SimulationResult,
) -> Figure:
    """Plot angular velocity omega over time in degrees per second."""
    return _create_state_plot(
        times=_state_times(result),
        values=_angle_state_values(
            result,
            attribute="omega",
        ),
        title="Angular Velocity",
        ylabel="omega [deg/s]",
    )


def plot_thrust_time_series(
    result: SimulationResult,
) -> Figure:
    """Plot physical thrust over time."""
    values = np.array(
        [
            actuation.thrust
            for actuation in result.physical_actuations
        ],
        dtype=np.float64,
    )

    return _create_actuation_plot(
        times=_actuation_times(result),
        values=values,
        title="Thrust",
        ylabel="T [N]",
    )


def plot_gimbal_angle_time_series(
    result: SimulationResult,
) -> Figure:
    """Plot physical gimbal angle over time in degrees."""
    values = np.array(
        [
            degrees(actuation.gimbal_angle)
            for actuation in result.physical_actuations
        ],
        dtype=np.float64,
    )

    return _create_actuation_plot(
        times=_actuation_times(result),
        values=values,
        title="Gimbal Angle",
        ylabel="delta [deg]",
    )


def plot_thrust_command_comparison(
    result: SimulationResult,
) -> Figure:
    """Compare commanded and physical thrust over time."""
    command_values = np.array(
        [
            command.thrust
            for command in result.commands
        ],
        dtype=np.float64,
    )

    physical_values = np.array(
        [
            actuation.thrust
            for actuation in result.physical_actuations
        ],
        dtype=np.float64,
    )

    return _create_actuation_comparison_plot(
        times=_actuation_times(result),
        command_values=command_values,
        physical_values=physical_values,
        title="Commanded vs. Physical Thrust",
        ylabel="T [N]",
    )


def plot_gimbal_angle_command_comparison(
    result: SimulationResult,
) -> Figure:
    """Compare commanded and physical gimbal angle over time."""
    command_values = np.array(
        [
            degrees(command.gimbal_angle)
            for command in result.commands
        ],
        dtype=np.float64,
    )

    physical_values = np.array(
        [
            degrees(actuation.gimbal_angle)
            for actuation in result.physical_actuations
        ],
        dtype=np.float64,
    )

    return _create_actuation_comparison_plot(
        times=_actuation_times(result),
        command_values=command_values,
        physical_values=physical_values,
        title="Commanded vs. Physical Gimbal Angle",
        ylabel="delta [deg]",
    )


def _state_times(
    result: SimulationResult,
) -> np.ndarray:
    return np.asarray(
        result.times,
        dtype=np.float64,
    )


def _actuation_times(
    result: SimulationResult,
) -> np.ndarray:
    return np.asarray(
        result.times,
        dtype=np.float64,
    )


def _state_values(
    result: SimulationResult,
    *,
    attribute: str,
) -> np.ndarray:
    return np.array(
        [
            getattr(state, attribute)
            for state in result.rigid_body_states
        ],
        dtype=np.float64,
    )


def _angle_state_values(
    result: SimulationResult,
    *,
    attribute: str,
) -> np.ndarray:
    return np.array(
        [
            degrees(
                getattr(state, attribute)
            )
            for state in result.rigid_body_states
        ],
        dtype=np.float64,
    )


def _create_state_plot(
    times: np.ndarray,
    values: np.ndarray,
    title: str,
    ylabel: str,
) -> Figure:
    """Create a standard line plot for state trajectories."""
    figure, axis = plt.subplots()

    _plot_state_on_axis(
        axis=axis,
        times=times,
        values=values,
        title=title,
        ylabel=ylabel,
    )

    axis.set_xlabel("time [s]")
    figure.tight_layout()

    return figure


def _create_actuation_plot(
    times: np.ndarray,
    values: np.ndarray,
    title: str,
    ylabel: str,
) -> Figure:
    """Create a staircase plot for interval-wise physical actuation."""
    figure, axis = plt.subplots()

    axis.stairs(
        values,
        times,
        baseline=None,
    )

    axis.set_title(title)
    axis.set_xlabel("time [s]")
    axis.set_ylabel(ylabel)
    axis.grid(True)

    figure.tight_layout()

    return figure


def _create_actuation_comparison_plot(
    times: np.ndarray,
    command_values: np.ndarray,
    physical_values: np.ndarray,
    title: str,
    ylabel: str,
) -> Figure:
    """Create a comparison plot for command and physical actuation."""
    figure, axis = plt.subplots()

    _plot_actuation_comparison_on_axis(
        axis=axis,
        times=times,
        command_values=command_values,
        physical_values=physical_values,
        title=title,
        ylabel=ylabel,
    )

    axis.set_xlabel("time [s]")
    figure.tight_layout()

    return figure


def _plot_state_on_axis(
    *,
    axis: Axes,
    times: np.ndarray,
    values: np.ndarray,
    title: str,
    ylabel: str,
) -> None:
    """Plot a state trajectory on an existing axis."""
    axis.plot(
        times,
        values,
    )

    axis.set_title(title)
    axis.set_ylabel(ylabel)
    axis.grid(True)


def _plot_actuation_comparison_on_axis(
    *,
    axis: Axes,
    times: np.ndarray,
    command_values: np.ndarray,
    physical_values: np.ndarray,
    title: str,
    ylabel: str,
) -> None:
    """Plot command and physical actuation on an existing axis."""
    axis.stairs(
        command_values,
        times,
        baseline=None,
        label="Command",
    )

    axis.stairs(
        physical_values,
        times,
        baseline=None,
        label="Physical",
    )

    axis.set_title(title)
    axis.set_ylabel(ylabel)
    axis.grid(True)
    axis.legend()
