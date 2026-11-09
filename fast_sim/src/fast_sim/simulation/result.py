"""Results produced by simulation runs."""

from dataclasses import dataclass
from math import isfinite

from fast_sim.models import (
    HopperActuation2D,
    HopperActuationCommand2D,
    HopperState2D,
)


@dataclass(frozen=True, slots=True)
class SimulationResult:
    """Immutable result of a completed simulation run.

    For N completed system steps:
        len(times) == N + 1
        len(rigid_body_states) == N + 1
        len(commands) == N
        len(physical_actuations) == N

    commands[k] is the command requested at times[k].

    physical_actuations[k] is the actuation produced by the actuator
    system and applied during the interval
    [times[k], times[k + 1]).
    """

    times: tuple[float, ...]
    rigid_body_states: tuple[HopperState2D, ...]
    commands: tuple[HopperActuationCommand2D, ...]
    physical_actuations: tuple[HopperActuation2D, ...]

    def __post_init__(self) -> None:
        if not self.times:
            raise ValueError("times must not be empty")

        if len(self.rigid_body_states) != len(self.times):
            raise ValueError(
                "rigid_body_states and times must have the same length"
            )

        expected_step_count = len(self.times) - 1

        if len(self.commands) != expected_step_count:
            raise ValueError(
                "commands must contain exactly one entry "
                "per system step"
            )

        if len(self.physical_actuations) != expected_step_count:
            raise ValueError(
                "physical_actuations must contain exactly one entry "
                "per system step"
            )

        for time in self.times:
            if not isfinite(time):
                raise ValueError("all times must be finite")

        for previous, current in zip(
            self.times,
            self.times[1:],
        ):
            if current <= previous:
                raise ValueError(
                    "times must be strictly increasing"
                )

    @property
    def step_count(self) -> int:
        """Return the number of completed system steps."""
        return len(self.commands)

    @property
    def start_time(self) -> float:
        """Return the simulation time at the beginning of the run."""
        return self.times[0]

    @property
    def end_time(self) -> float:
        """Return the simulation time at the end of the run."""
        return self.times[-1]

    @property
    def duration(self) -> float:
        """Return the simulated duration in seconds."""
        return self.end_time - self.start_time

    @property
    def initial_rigid_body_state(self) -> HopperState2D:
        """Return the rigid-body state at the beginning of the run."""
        return self.rigid_body_states[0]

    @property
    def final_rigid_body_state(self) -> HopperState2D:
        """Return the rigid-body state at the end of the run."""
        return self.rigid_body_states[-1]