"""Results produced by individual hopper system steps."""

from dataclasses import dataclass

from fast_sim.models import (
    HopperActuation2D,
    HopperState2D,
)


@dataclass(frozen=True, slots=True)
class HopperSystemStepResult2D:
    """Result of one complete 2D hopper system step."""

    rigid_body_state: HopperState2D
    actuation: HopperActuation2D