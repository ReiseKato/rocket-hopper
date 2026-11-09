"""Newton-Euler dynamics for the first 2D Rocket Hopper model."""

from math import cos, sin

from fast_sim.models import (
    HopperActuation2D,
    HopperState2D,
    HopperStateDerivative2D,
    RigidBody2DParameters,
)


class Hopper2DDynamics:
    """Deterministic 2D rigid-body dynamics for the hopper.

    The model is intentionally stateless apart from its immutable physical
    parameters. It maps a mechanical state and physical actuation to the
    instantaneous state derivative.
    """

    def __init__(self, parameters: RigidBody2DParameters) -> None:
        self._parameters = parameters

    @property
    def parameters(self) -> RigidBody2DParameters:
        """Return the immutable physical parameters used by this model."""
        return self._parameters

    def derivative(
        self,
        state: HopperState2D,
        actuation: HopperActuation2D,
    ) -> HopperStateDerivative2D:
        """Compute the instantaneous 2D rigid-body state derivative.

        Args:
            state: Current mechanical state of the hopper.
            actuation: Actual physical thrust and gimbal angle.

        Returns:
            Instantaneous derivative of the mechanical state.
        """
        p = self._parameters
        thrust = actuation.thrust
        delta = actuation.gimbal_angle
        thrust_angle_world = state.theta + delta

        ax = thrust / p.mass * sin(thrust_angle_world)
        az = thrust / p.mass * cos(thrust_angle_world) - p.gravity
        angular_acceleration = (
            -p.engine_offset * thrust / p.inertia_y * sin(delta)
        )

        return HopperStateDerivative2D(
            px_dot=state.vx,
            pz_dot=state.vz,
            vx_dot=ax,
            vz_dot=az,
            theta_dot=state.omega,
            omega_dot=angular_acceleration,
        )
