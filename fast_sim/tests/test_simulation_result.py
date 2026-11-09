import pytest

from fast_sim.models import (
    HopperActuation2D,
    HopperActuationCommand2D,
    HopperState2D,
)
from fast_sim.simulation import SimulationResult


def create_state(px: float) -> HopperState2D:
    return HopperState2D(
        px=px,
        pz=0.0,
        vx=0.0,
        vz=0.0,
        theta=0.0,
        omega=0.0,
    )


def create_command(thrust: float) -> HopperActuationCommand2D:
    return HopperActuationCommand2D(
        thrust=thrust,
        gimbal_angle=0.0,
    )


def create_actuation(thrust: float) -> HopperActuation2D:
    return HopperActuation2D(
        thrust=thrust,
        gimbal_angle=0.0,
    )


def test_result_exposes_run_metadata() -> None:
    result = SimulationResult(
        times=(1.0, 1.1, 1.2),
        rigid_body_states=(
            create_state(0.0),
            create_state(1.0),
            create_state(2.0),
        ),
        commands=(
            create_command(10.0),
            create_command(20.0),
        ),
        physical_actuations=(
            create_actuation(9.0),
            create_actuation(18.0),
        ),
    )

    assert result.step_count == 2
    assert result.start_time == pytest.approx(1.0)
    assert result.end_time == pytest.approx(1.2)
    assert result.duration == pytest.approx(0.2)

    assert result.initial_rigid_body_state.px == pytest.approx(0.0)
    assert result.final_rigid_body_state.px == pytest.approx(2.0)


def test_result_rejects_mismatched_rigid_body_state_count() -> None:
    with pytest.raises(ValueError):
        SimulationResult(
            times=(0.0, 0.1),
            rigid_body_states=(
                create_state(0.0),
            ),
            commands=(
                create_command(10.0),
            ),
            physical_actuations=(
                create_actuation(10.0),
            ),
        )


def test_result_rejects_mismatched_command_count() -> None:
    with pytest.raises(ValueError):
        SimulationResult(
            times=(0.0, 0.1),
            rigid_body_states=(
                create_state(0.0),
                create_state(1.0),
            ),
            commands=(),
            physical_actuations=(
                create_actuation(10.0),
            ),
        )


def test_result_rejects_mismatched_physical_actuation_count() -> None:
    with pytest.raises(ValueError):
        SimulationResult(
            times=(0.0, 0.1),
            rigid_body_states=(
                create_state(0.0),
                create_state(1.0),
            ),
            commands=(
                create_command(10.0),
            ),
            physical_actuations=(),
        )


def test_result_rejects_non_increasing_times() -> None:
    with pytest.raises(ValueError):
        SimulationResult(
            times=(0.0, 0.0),
            rigid_body_states=(
                create_state(0.0),
                create_state(1.0),
            ),
            commands=(
                create_command(10.0),
            ),
            physical_actuations=(
                create_actuation(10.0),
            ),
        )


@pytest.mark.parametrize(
    "times",
    [
        (0.0, float("inf")),
        (0.0, float("-inf")),
        (0.0, float("nan")),
    ],
)
def test_result_rejects_non_finite_times(
    times: tuple[float, float],
) -> None:
    with pytest.raises(ValueError):
        SimulationResult(
            times=times,
            rigid_body_states=(
                create_state(0.0),
                create_state(1.0),
            ),
            commands=(
                create_command(10.0),
            ),
            physical_actuations=(
                create_actuation(10.0),
            ),
        )
