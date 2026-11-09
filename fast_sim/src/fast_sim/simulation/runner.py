"""Execution of multi-step hopper simulation runs."""

from collections.abc import Callable
from math import isfinite
from typing import TypeAlias

from fast_sim.models import (
    HopperActuationCommand2D,
    HopperState2D,
)
from fast_sim.simulation.result import SimulationResult
from fast_sim.system import HopperSystem2D


CommandProvider: TypeAlias = Callable[
    [float, HopperState2D],
    HopperActuationCommand2D,
]


class SimulationRunner:
    """Execute repeated steps of a HopperSystem2D."""

    def __init__(
        self,
        system: HopperSystem2D,
    ) -> None:
        self._system = system

    @property
    def system(self) -> HopperSystem2D:
        """Return the hopper system used by this runner."""
        return self._system

    def run(
        self,
        step_count: int,
        dt: float,
        command_provider: CommandProvider,
    ) -> SimulationResult:
        """Run the hopper system for a fixed number of steps.

        The command provider is evaluated once before every system step
        using the current simulation time and current rigid-body state.

        Args:
            step_count:
                Positive number of system steps to execute.
            dt:
                Positive system step size in seconds.
            command_provider:
                Function returning the requested actuation command for
                the next system step.

        Returns:
            Immutable result containing time, rigid-body state,
            command, and physical actuation histories.
        """
        self._validate_run_parameters(
            step_count=step_count,
            dt=dt,
        )

        times = [self._system.time]
        rigid_body_states = [
            self._system.rigid_body_state
        ]

        commands: list[HopperActuationCommand2D] = []
        physical_actuations = []

        for _ in range(step_count):
            command = command_provider(
                self._system.time,
                self._system.rigid_body_state,
            )

            if not isinstance(
                command,
                HopperActuationCommand2D,
            ):
                raise TypeError(
                    "command_provider must return "
                    "HopperActuationCommand2D"
                )

            step_result = self._system.step(
                command=command,
                dt=dt,
            )

            commands.append(command)
            physical_actuations.append(
                step_result.actuation
            )

            times.append(self._system.time)
            rigid_body_states.append(
                step_result.rigid_body_state
            )

        return SimulationResult(
            times=tuple(times),
            rigid_body_states=tuple(rigid_body_states),
            commands=tuple(commands),
            physical_actuations=tuple(
                physical_actuations
            ),
        )

    @staticmethod
    def _validate_run_parameters(
        step_count: int,
        dt: float,
    ) -> None:
        if isinstance(step_count, bool) or not isinstance(
            step_count,
            int,
        ):
            raise TypeError(
                "step_count must be an integer"
            )

        if step_count <= 0:
            raise ValueError(
                "step_count must be > 0"
            )

        if not isfinite(dt):
            raise ValueError(
                "dt must be finite"
            )

        if dt <= 0.0:
            raise ValueError(
                "dt must be > 0"
            )