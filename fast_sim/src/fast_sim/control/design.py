"""Design helpers for 2D controllers."""

from math import isfinite

from fast_sim.control.parameters import (
    PDAltitudeControllerParameters,
    PDAttitudeControllerParameters,
    PDHorizontalPositionControllerParameters,
)


def design_pd_attitude_gains(
    *,
    inertia_y: float,
    engine_offset: float,
    nominal_thrust: float,
    natural_frequency: float,
    damping_ratio: float,
) -> PDAttitudeControllerParameters:
    """Design attitude PD gains from a small-angle target model."""
    values = {
        "inertia_y": inertia_y,
        "engine_offset": engine_offset,
        "nominal_thrust": nominal_thrust,
        "natural_frequency": natural_frequency,
        "damping_ratio": damping_ratio,
    }

    for name, value in values.items():
        if not isfinite(value):
            raise ValueError(
                f"{name} must be finite"
            )

    if inertia_y <= 0.0:
        raise ValueError("inertia_y must be > 0")

    if engine_offset <= 0.0:
        raise ValueError(
            "engine_offset must be > 0"
        )

    if nominal_thrust <= 0.0:
        raise ValueError(
            "nominal_thrust must be > 0"
        )

    if natural_frequency <= 0.0:
        raise ValueError(
            "natural_frequency must be > 0"
        )

    if damping_ratio <= 0.0:
        raise ValueError(
            "damping_ratio must be > 0"
        )

    control_authority = (
        engine_offset
        * nominal_thrust
        / inertia_y
    )

    proportional_gain = (
        natural_frequency**2
        / control_authority
    )

    derivative_gain = (
        2.0
        * damping_ratio
        * natural_frequency
        / control_authority
    )

    return PDAttitudeControllerParameters(
        proportional_gain=proportional_gain,
        derivative_gain=derivative_gain,
    )


def design_pd_altitude_gains(
    *,
    mass: float,
    natural_frequency: float,
    damping_ratio: float,
) -> PDAltitudeControllerParameters:
    """Design altitude PD gains from an upright-hover target model."""
    values = {
        "mass": mass,
        "natural_frequency": natural_frequency,
        "damping_ratio": damping_ratio,
    }

    for name, value in values.items():
        if not isfinite(value):
            raise ValueError(
                f"{name} must be finite"
            )

    if mass <= 0.0:
        raise ValueError("mass must be > 0")

    if natural_frequency <= 0.0:
        raise ValueError(
            "natural_frequency must be > 0"
        )

    if damping_ratio <= 0.0:
        raise ValueError(
            "damping_ratio must be > 0"
        )

    proportional_gain = (
        mass
        * natural_frequency**2
    )

    derivative_gain = (
        2.0
        * damping_ratio
        * natural_frequency
        * mass
    )

    return PDAltitudeControllerParameters(
        proportional_gain=proportional_gain,
        derivative_gain=derivative_gain,
    )


def design_pd_horizontal_position_gains(
    *,
    natural_frequency: float,
    damping_ratio: float,
) -> PDHorizontalPositionControllerParameters:
    """Design outer-loop horizontal position PD gains.

    Assuming the inner attitude loop follows the requested attitude
    sufficiently fast and

        p_x_ddot = a_x_cmd,

    the desired second-order error dynamics give

        Kp = natural_frequency**2
        Kd = 2 * damping_ratio * natural_frequency.
    """
    values = {
        "natural_frequency": natural_frequency,
        "damping_ratio": damping_ratio,
    }

    for name, value in values.items():
        if not isfinite(value):
            raise ValueError(
                f"{name} must be finite"
            )

    if natural_frequency <= 0.0:
        raise ValueError(
            "natural_frequency must be > 0"
        )

    if damping_ratio <= 0.0:
        raise ValueError(
            "damping_ratio must be > 0"
        )

    return PDHorizontalPositionControllerParameters(
        proportional_gain=natural_frequency**2,
        derivative_gain=(
            2.0
            * damping_ratio
            * natural_frequency
        ),
    )
