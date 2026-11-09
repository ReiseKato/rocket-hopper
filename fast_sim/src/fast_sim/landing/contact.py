"""Minimal impact and single-contact dynamics for 2D landing."""

from dataclasses import dataclass
from enum import Enum
from math import cos, isfinite, sin

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing.touchdown import (
    TouchdownContact2D,
    TouchdownDetection2D,
)
from fast_sim.models import (
    HopperActuation2D,
    HopperState2D,
    RigidBody2DParameters,
)


class SingleContactFoot2D(Enum):
    """Landing-gear foot that is fixed to the ground."""

    LEFT = "left"
    RIGHT = "right"


@dataclass(frozen=True, slots=True)
class SingleContactConstraint2D:
    """Fixed point-contact constraint after the first impact.

    Attributes:
        foot: Landing-gear foot that remains fixed.
        contact_x: Fixed horizontal world position of the contact point [m].
        ground_height: Fixed vertical world position of the contact point [m].
    """

    foot: SingleContactFoot2D
    contact_x: float
    ground_height: float

    def __post_init__(self) -> None:
        if not isfinite(self.contact_x):
            raise ValueError("contact_x must be finite")

        if not isfinite(self.ground_height):
            raise ValueError("ground_height must be finite")


@dataclass(frozen=True, slots=True)
class SingleContactState2D:
    """Reduced one-degree-of-freedom state during single-foot contact.

    Attributes:
        theta: Vehicle attitude angle [rad].
        omega: Angular velocity about the fixed contact point [rad/s].
    """

    theta: float
    omega: float

    def __post_init__(self) -> None:
        if not isfinite(self.theta):
            raise ValueError("theta must be finite")

        if not isfinite(self.omega):
            raise ValueError("omega must be finite")


@dataclass(frozen=True, slots=True)
class SingleContactStateDerivative2D:
    """Derivative of the reduced single-contact state."""

    theta_dot: float
    omega_dot: float


@dataclass(frozen=True, slots=True)
class FirstContactImpactResult2D:
    """Result of the fully inelastic first-foot impact transition."""

    constraint: SingleContactConstraint2D
    single_contact_state: SingleContactState2D
    post_impact_state: HopperState2D

    contact_velocity_before_x: float
    contact_velocity_before_z: float

    angular_momentum_about_contact_before: float
    angular_momentum_about_contact_after: float

    kinetic_energy_before: float
    kinetic_energy_after: float
    dissipated_energy: float


class FirstContactImpactTransition2D:
    """Apply a fully inelastic no-slip first-foot impact.

    The first contacting foot is projected onto the horizontal ground plane
    and becomes a fixed point. Linear momentum is not conserved because the
    ground supplies an impulse. Angular momentum about the contact point is
    conserved during the instantaneous impact.
    """

    def __init__(
        self,
        parameters: RigidBody2DParameters,
        geometry: LandingGearGeometry2D,
    ) -> None:
        self._parameters = parameters
        self._geometry = geometry

    @property
    def parameters(self) -> RigidBody2DParameters:
        """Return the rigid-body parameters."""
        return self._parameters

    @property
    def geometry(self) -> LandingGearGeometry2D:
        """Return the landing-gear geometry."""
        return self._geometry

    def apply(
        self,
        state: HopperState2D,
        detection: TouchdownDetection2D,
    ) -> FirstContactImpactResult2D:
        """Apply the first-contact impact transition.

        Exactly one foot must be in contact. Direct simultaneous two-foot
        contact is already a completed geometric touchdown and therefore
        does not enter the single-contact phase.
        """
        foot = self._single_contact_foot(
            detection.contact
        )

        positions = self._geometry.foot_positions_world(
            state
        )

        if foot is SingleContactFoot2D.LEFT:
            contact_x = positions.left_x
        else:
            contact_x = positions.right_x

        constraint = SingleContactConstraint2D(
            foot=foot,
            contact_x=contact_x,
            ground_height=detection.ground_height,
        )

        r_x, r_z = self._cg_to_contact_vector_world(
            theta=state.theta,
            foot=foot,
        )

        contact_velocity_before_x = (
            state.vx
            + state.omega * r_z
        )

        contact_velocity_before_z = (
            state.vz
            - state.omega * r_x
        )

        p = self._parameters

        angular_momentum_before = (
            p.inertia_y * state.omega
            + p.mass
            * (
                r_x * state.vz
                - r_z * state.vx
            )
        )

        inertia_about_contact = (
            p.inertia_y
            + p.mass * (
                r_x**2
                + r_z**2
            )
        )

        omega_after = (
            angular_momentum_before
            / inertia_about_contact
        )

        single_contact_state = SingleContactState2D(
            theta=state.theta,
            omega=omega_after,
        )

        post_impact_state = self._hopper_state_from_constraint(
            single_contact_state,
            constraint,
        )

        kinetic_energy_before = (
            0.5 * p.mass
            * (
                state.vx**2
                + state.vz**2
            )
            + 0.5 * p.inertia_y
            * state.omega**2
        )

        kinetic_energy_after = (
            0.5
            * inertia_about_contact
            * omega_after**2
        )

        dissipated_energy = (
            kinetic_energy_before
            - kinetic_energy_after
        )

        angular_momentum_after = (
            inertia_about_contact
            * omega_after
        )

        return FirstContactImpactResult2D(
            constraint=constraint,
            single_contact_state=single_contact_state,
            post_impact_state=post_impact_state,
            contact_velocity_before_x=(
                contact_velocity_before_x
            ),
            contact_velocity_before_z=(
                contact_velocity_before_z
            ),
            angular_momentum_about_contact_before=(
                angular_momentum_before
            ),
            angular_momentum_about_contact_after=(
                angular_momentum_after
            ),
            kinetic_energy_before=(
                kinetic_energy_before
            ),
            kinetic_energy_after=(
                kinetic_energy_after
            ),
            dissipated_energy=(
                dissipated_energy
            ),
        )

    def _single_contact_foot(
        self,
        contact: TouchdownContact2D,
    ) -> SingleContactFoot2D:
        if contact is TouchdownContact2D.LEFT:
            return SingleContactFoot2D.LEFT

        if contact is TouchdownContact2D.RIGHT:
            return SingleContactFoot2D.RIGHT

        raise ValueError(
            "first-contact impact requires exactly "
            "one contacting foot"
        )

    def _cg_to_contact_vector_world(
        self,
        *,
        theta: float,
        foot: SingleContactFoot2D,
    ) -> tuple[float, float]:
        body_x = self._foot_body_x(foot)
        body_z = -self._geometry.cg_to_feet

        r_x = (
            body_x * cos(theta)
            + body_z * sin(theta)
        )

        r_z = (
            -body_x * sin(theta)
            + body_z * cos(theta)
        )

        return r_x, r_z

    def _foot_body_x(
        self,
        foot: SingleContactFoot2D,
    ) -> float:
        half_spacing = (
            self._geometry.half_foot_spacing
        )

        if foot is SingleContactFoot2D.LEFT:
            return -half_spacing

        return half_spacing

    def _hopper_state_from_constraint(
        self,
        state: SingleContactState2D,
        constraint: SingleContactConstraint2D,
    ) -> HopperState2D:
        body_x = self._foot_body_x(
            constraint.foot
        )
        body_z = -self._geometry.cg_to_feet

        r_x = (
            body_x * cos(state.theta)
            + body_z * sin(state.theta)
        )

        r_z = (
            -body_x * sin(state.theta)
            + body_z * cos(state.theta)
        )

        q_x = -r_x
        q_z = -r_z

        px = constraint.contact_x + q_x
        pz = constraint.ground_height + q_z

        vx = state.omega * q_z
        vz = -state.omega * q_x

        return HopperState2D(
            px=px,
            pz=pz,
            vx=vx,
            vz=vz,
            theta=state.theta,
            omega=state.omega,
        )


class SingleContactDynamics2D:
    """One-DOF rigid-body dynamics about a fixed landing foot.

    The first contacting foot is assumed to stick to a horizontal ground
    plane without slipping or lifting off. The only remaining degree of
    freedom is vehicle rotation about that point.
    """

    def __init__(
        self,
        parameters: RigidBody2DParameters,
        geometry: LandingGearGeometry2D,
    ) -> None:
        self._parameters = parameters
        self._geometry = geometry

    @property
    def parameters(self) -> RigidBody2DParameters:
        """Return the rigid-body parameters."""
        return self._parameters

    @property
    def geometry(self) -> LandingGearGeometry2D:
        """Return the landing-gear geometry."""
        return self._geometry

    def inertia_about_contact(
        self,
    ) -> float:
        """Return the rigid-body inertia about either symmetric foot."""
        p = self._parameters

        radius_squared = (
            self._geometry.half_foot_spacing**2
            + self._geometry.cg_to_feet**2
        )

        return (
            p.inertia_y
            + p.mass * radius_squared
        )

    def derivative(
        self,
        state: SingleContactState2D,
        constraint: SingleContactConstraint2D,
        actuation: HopperActuation2D,
    ) -> SingleContactStateDerivative2D:
        """Return the constrained angular state derivative."""
        q_x, q_z = self._contact_to_cg_vector_world(
            theta=state.theta,
            foot=constraint.foot,
        )

        p = self._parameters
        thrust = actuation.thrust
        delta = actuation.gimbal_angle

        thrust_angle_world = (
            state.theta
            + delta
        )

        thrust_x = (
            thrust * sin(thrust_angle_world)
        )

        thrust_z = (
            thrust * cos(thrust_angle_world)
        )

        gravity_torque = (
            p.mass
            * p.gravity
            * q_x
        )

        thrust_force_torque = (
            q_z * thrust_x
            - q_x * thrust_z
        )

        thrust_offset_torque = (
            -p.engine_offset
            * thrust
            * sin(delta)
        )

        total_torque = (
            gravity_torque
            + thrust_force_torque
            + thrust_offset_torque
        )

        angular_acceleration = (
            total_torque
            / self.inertia_about_contact()
        )

        return SingleContactStateDerivative2D(
            theta_dot=state.omega,
            omega_dot=angular_acceleration,
        )

    def to_hopper_state(
        self,
        state: SingleContactState2D,
        constraint: SingleContactConstraint2D,
    ) -> HopperState2D:
        """Reconstruct the full rigid-body state from the fixed contact."""
        q_x, q_z = self._contact_to_cg_vector_world(
            theta=state.theta,
            foot=constraint.foot,
        )

        px = (
            constraint.contact_x
            + q_x
        )

        pz = (
            constraint.ground_height
            + q_z
        )

        vx = (
            state.omega
            * q_z
        )

        vz = (
            -state.omega
            * q_x
        )

        return HopperState2D(
            px=px,
            pz=pz,
            vx=vx,
            vz=vz,
            theta=state.theta,
            omega=state.omega,
        )

    def _contact_to_cg_vector_world(
        self,
        *,
        theta: float,
        foot: SingleContactFoot2D,
    ) -> tuple[float, float]:
        body_x = self._foot_body_x(foot)
        body_z = -self._geometry.cg_to_feet

        cg_to_foot_x = (
            body_x * cos(theta)
            + body_z * sin(theta)
        )

        cg_to_foot_z = (
            -body_x * sin(theta)
            + body_z * cos(theta)
        )

        return (
            -cg_to_foot_x,
            -cg_to_foot_z,
        )

    def _foot_body_x(
        self,
        foot: SingleContactFoot2D,
    ) -> float:
        half_spacing = (
            self._geometry.half_foot_spacing
        )

        if foot is SingleContactFoot2D.LEFT:
            return -half_spacing

        return half_spacing


@dataclass(frozen=True, slots=True)
class SimultaneousContactImpactResult2D:
    """Result of a fully inelastic simultaneous two-foot impact.

    Both feet are assumed to stick to the horizontal ground at the same
    instant. The post-impact rigid body is stationary.

    The dissipated energy is the kinetic energy lost by the complete
    vehicle. This model does not assign that energy to individual legs.
    """

    post_impact_state: HopperState2D

    left_contact_velocity_before_x: float
    left_contact_velocity_before_z: float
    right_contact_velocity_before_x: float
    right_contact_velocity_before_z: float

    kinetic_energy_before: float
    kinetic_energy_after: float
    dissipated_energy: float

    foot_height_difference_before: float


class SimultaneousContactImpactTransition2D:
    """Apply a fully inelastic no-slip simultaneous two-foot impact.

    A `BOTH` detection is necessary but not sufficient. In a discrete
    simulation, both feet may already lie below the ground even though
    one foot would have contacted earlier. Therefore the two foot heights
    must also agree within `max_foot_height_difference`.

    For a genuine simultaneous impact, the post-impact generalized
    velocity is zero:

        vx+ = 0
        vz+ = 0
        omega+ = 0

    The complete pre-impact kinetic energy is dissipated by the idealized
    impact. No leg-wise energy split is introduced.
    """

    def __init__(
        self,
        parameters: RigidBody2DParameters,
        geometry: LandingGearGeometry2D,
        *,
        max_foot_height_difference: float = 1e-6,
    ) -> None:
        if not isfinite(max_foot_height_difference):
            raise ValueError(
                "max_foot_height_difference must be finite"
            )

        if max_foot_height_difference < 0.0:
            raise ValueError(
                "max_foot_height_difference must be >= 0"
            )

        self._parameters = parameters
        self._geometry = geometry
        self._max_foot_height_difference = (
            max_foot_height_difference
        )

    @property
    def parameters(self) -> RigidBody2DParameters:
        """Return the rigid-body parameters."""
        return self._parameters

    @property
    def geometry(self) -> LandingGearGeometry2D:
        """Return the landing-gear geometry."""
        return self._geometry

    @property
    def max_foot_height_difference(self) -> float:
        """Return the geometric simultaneity tolerance in meters."""
        return self._max_foot_height_difference

    def apply(
        self,
        state: HopperState2D,
        detection: TouchdownDetection2D,
    ) -> SimultaneousContactImpactResult2D:
        """Apply the simultaneous two-foot impact transition."""
        if detection.contact is not TouchdownContact2D.BOTH:
            raise ValueError(
                "simultaneous impact requires BOTH contact"
            )

        positions = self._geometry.foot_positions_world(
            state
        )

        foot_height_difference = abs(
            positions.left_z
            - positions.right_z
        )

        if (
            foot_height_difference
            > self._max_foot_height_difference
        ):
            raise ValueError(
                "BOTH detection is not geometrically simultaneous; "
                "resolve the earlier single-foot contact instead"
            )

        left_velocity_x, left_velocity_z = (
            self._foot_velocity_world(
                state,
                body_x=-self._geometry.half_foot_spacing,
            )
        )

        right_velocity_x, right_velocity_z = (
            self._foot_velocity_world(
                state,
                body_x=self._geometry.half_foot_spacing,
            )
        )

        p = self._parameters

        kinetic_energy_before = (
            0.5
            * p.mass
            * (
                state.vx**2
                + state.vz**2
            )
            + 0.5
            * p.inertia_y
            * state.omega**2
        )

        # With two separated sticking contacts, all planar rigid-body
        # generalized velocities are constrained to zero after impact.
        kinetic_energy_after = 0.0

        mean_foot_height = 0.5 * (
            positions.left_z
            + positions.right_z
        )

        vertical_projection = (
            detection.ground_height
            - mean_foot_height
        )

        post_impact_state = HopperState2D(
            px=state.px,
            pz=state.pz + vertical_projection,
            vx=0.0,
            vz=0.0,
            theta=state.theta,
            omega=0.0,
        )

        return SimultaneousContactImpactResult2D(
            post_impact_state=post_impact_state,
            left_contact_velocity_before_x=(
                left_velocity_x
            ),
            left_contact_velocity_before_z=(
                left_velocity_z
            ),
            right_contact_velocity_before_x=(
                right_velocity_x
            ),
            right_contact_velocity_before_z=(
                right_velocity_z
            ),
            kinetic_energy_before=(
                kinetic_energy_before
            ),
            kinetic_energy_after=(
                kinetic_energy_after
            ),
            dissipated_energy=(
                kinetic_energy_before
                - kinetic_energy_after
            ),
            foot_height_difference_before=(
                foot_height_difference
            ),
        )

    def _foot_velocity_world(
        self,
        state: HopperState2D,
        *,
        body_x: float,
    ) -> tuple[float, float]:
        body_z = -self._geometry.cg_to_feet

        r_x = (
            body_x * cos(state.theta)
            + body_z * sin(state.theta)
        )

        r_z = (
            -body_x * sin(state.theta)
            + body_z * cos(state.theta)
        )

        return (
            state.vx
            + state.omega * r_z,
            state.vz
            - state.omega * r_x,
        )
