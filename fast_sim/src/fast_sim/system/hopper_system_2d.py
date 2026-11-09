"""Composition of the physical 2D hopper system."""

from fast_sim.actuators import HopperActuatorSystem2D
from fast_sim.models import (
    HopperActuationCommand2D,
    HopperState2D,
)
from fast_sim.simulation import HopperSimulator
from fast_sim.system.result import (
    HopperSystemStepResult2D,
)


class HopperSystem2D:
    """Coordinate actuators and rigid-body simulation."""

    def __init__(
        self,
        actuator_system: HopperActuatorSystem2D,
        simulator: HopperSimulator,
    ) -> None:
        self._actuator_system = actuator_system
        self._simulator = simulator

    @property
    def time(self) -> float:
        """Return the current simulation time."""
        return self._simulator.time

    @property
    def rigid_body_state(self) -> HopperState2D:
        """Return the current rigid-body state."""
        return self._simulator.state

    def step(
        self,
        command: HopperActuationCommand2D,
        dt: float,
    ) -> HopperSystemStepResult2D:
        """Advance the complete hopper system by one step."""
        actuation = self._actuator_system.step(
            command=command,
            dt=dt,
        )

        next_state = self._simulator.step(
            actuation=actuation,
            dt=dt,
        )

        return HopperSystemStepResult2D(
            rigid_body_state=next_state,
            actuation=actuation,
        )