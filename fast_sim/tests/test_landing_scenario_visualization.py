import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing import (
    LandingContactMetrics2D,
    LandingContactSequence2D,
    LandingDynamicsMode2D,
    LandingEvaluation2D,
    LandingRunResult2D,
    LandingTerminationReason2D,
    SimultaneousLandingContactMetrics2D,
    SimultaneousLandingEvaluation2D,
    SingleContactFoot2D,
)
from fast_sim.models import (
    HopperActuation2D,
    HopperActuationCommand2D,
    HopperState2D,
)
from fast_sim.visualization.landing_scenarios import (
    plot_landing_scenario_overview,
)


def state(
    *,
    px: float,
    pz: float,
) -> HopperState2D:
    return HopperState2D(
        px=px,
        pz=pz,
        vx=0.0,
        vz=-0.1,
        theta=0.0,
        omega=0.0,
    )


def command() -> HopperActuationCommand2D:
    return HopperActuationCommand2D(
        thrust=19.62,
        gimbal_angle=0.0,
    )


def actuation() -> HopperActuation2D:
    return HopperActuation2D(
        thrust=19.62,
        gimbal_angle=0.0,
    )


def test_visualization_supports_simultaneous_contact() -> None:
    contact = (
        SimultaneousLandingContactMetrics2D(
            time=1.0,
            left_normal_speed=0.10,
            left_tangential_speed=0.01,
            right_normal_speed=0.10,
            right_tangential_speed=0.01,
            vehicle_theta=0.0,
            vehicle_omega=0.0,
            cg_vx=0.0,
            cg_vz=-0.10,
            kinetic_energy_before=0.02,
            kinetic_energy_after=0.0,
            dissipated_energy=0.02,
        )
    )

    evaluation = (
        SimultaneousLandingEvaluation2D(
            completed=True,
            successful=True,
            contact=contact,
            horizontal_position_error=0.0,
            total_dissipated_energy=0.02,
            failed_criteria=(),
        )
    )

    landing = LandingRunResult2D(
        times=(0.0, 1.0),
        rigid_body_states=(
            state(
                px=0.0,
                pz=2.0,
            ),
            state(
                px=0.0,
                pz=0.8,
            ),
        ),
        modes=(
            LandingDynamicsMode2D.FREE_FLIGHT,
            LandingDynamicsMode2D.COMPLETED,
        ),
        guidance_references=(None,),  # type: ignore[arg-type]
        commands=(command(),),
        physical_actuations=(actuation(),),
        first_contact=None,
        evaluation=evaluation,
        contact_sequence=(
            LandingContactSequence2D.SIMULTANEOUS
        ),
        termination_reason=(
            LandingTerminationReason2D.COMPLETED
        ),
    )

    figure = plot_landing_scenario_overview(
        landing,
        geometry=LandingGearGeometry2D(
            foot_spacing=0.6,
            cg_to_feet=0.8,
        ),
        landing_target_x=0.0,
        title="Test",
    )

    assert len(figure.axes) == 8

    plt.close(figure)
