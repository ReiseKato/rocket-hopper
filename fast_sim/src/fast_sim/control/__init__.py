"""Control components for the fast simulator."""

from fast_sim.control.altitude import PDAltitudeController2D
from fast_sim.control.attitude import PDAttitudeController2D
from fast_sim.control.base import (
    AltitudeController2D,
    AttitudeController2D,
    HorizontalPositionController2D,
)
from fast_sim.control.design import (
    design_pd_altitude_gains,
    design_pd_attitude_gains,
    design_pd_horizontal_position_gains,
)
from fast_sim.control.horizontal_position import (
    PDHorizontalPositionController2D,
)
from fast_sim.control.parameters import (
    PDAltitudeControllerParameters,
    PDAttitudeControllerParameters,
    PDHorizontalPositionControllerParameters,
)
from fast_sim.control.reference import (
    AltitudeReference2D,
    AttitudeReference2D,
    HorizontalPositionReference2D,
)


__all__ = [
    "AltitudeController2D",
    "AltitudeReference2D",
    "AttitudeController2D",
    "AttitudeReference2D",
    "HorizontalPositionController2D",
    "HorizontalPositionReference2D",
    "PDAltitudeController2D",
    "PDAltitudeControllerParameters",
    "PDAttitudeController2D",
    "PDAttitudeControllerParameters",
    "PDHorizontalPositionController2D",
    "PDHorizontalPositionControllerParameters",
    "design_pd_altitude_gains",
    "design_pd_attitude_gains",
    "design_pd_horizontal_position_gains",
]
