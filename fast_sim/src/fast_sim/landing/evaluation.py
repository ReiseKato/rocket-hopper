"""Landing success criteria and touchdown evaluation for the 2D hopper."""

from dataclasses import dataclass
from enum import Enum
from math import cos, isfinite, sin

from fast_sim.geometry import LandingGearGeometry2D
from fast_sim.landing.contact import (
    FirstContactImpactResult2D,
    SimultaneousContactImpactResult2D,
    SingleContactFoot2D,
)
from fast_sim.landing.touchdown import (
    TouchdownContact2D,
    TouchdownDetection2D,
)
from fast_sim.models import (
    HopperState2D,
    RigidBody2DParameters,
)


class LandingFailureReason2D(Enum):
    """Reason why a geometrically completed landing failed."""

    HORIZONTAL_POSITION_ERROR = "horizontal_position_error"
    FIRST_CONTACT_NORMAL_SPEED = "first_contact_normal_speed"
    FIRST_CONTACT_TANGENTIAL_SPEED = "first_contact_tangential_speed"
    SECOND_CONTACT_NORMAL_SPEED = "second_contact_normal_speed"
    SECOND_CONTACT_TANGENTIAL_SPEED = "second_contact_tangential_speed"
    ATTITUDE_ANGLE = "attitude_angle"
    ANGULAR_SPEED = "angular_speed"
    SIMULTANEOUS_LEFT_NORMAL_SPEED = "simultaneous_left_normal_speed"
    SIMULTANEOUS_LEFT_TANGENTIAL_SPEED = "simultaneous_left_tangential_speed"
    SIMULTANEOUS_RIGHT_NORMAL_SPEED = "simultaneous_right_normal_speed"
    SIMULTANEOUS_RIGHT_TANGENTIAL_SPEED = "simultaneous_right_tangential_speed"


@dataclass(frozen=True, slots=True)
class LandingSuccessCriteria2D:
    """Configurable success limits for a completed 2D landing.

    Units:
        max_horizontal_position_error: m
        max_first_contact_normal_speed: m/s
        max_first_contact_tangential_speed: m/s
        max_second_contact_normal_speed: m/s
        max_second_contact_tangential_speed: m/s
        max_attitude_angle: rad
        max_angular_speed: rad/s
    """

    max_horizontal_position_error: float
    max_first_contact_normal_speed: float
    max_first_contact_tangential_speed: float
    max_second_contact_normal_speed: float
    max_second_contact_tangential_speed: float
    max_attitude_angle: float
    max_angular_speed: float
    max_simultaneous_contact_normal_speed: float | None = None
    max_simultaneous_contact_tangential_speed: float | None = None

    def __post_init__(self) -> None:
        values = {
            "max_horizontal_position_error": (
                self.max_horizontal_position_error
            ),
            "max_first_contact_normal_speed": (
                self.max_first_contact_normal_speed
            ),
            "max_first_contact_tangential_speed": (
                self.max_first_contact_tangential_speed
            ),
            "max_second_contact_normal_speed": (
                self.max_second_contact_normal_speed
            ),
            "max_second_contact_tangential_speed": (
                self.max_second_contact_tangential_speed
            ),
            "max_attitude_angle": self.max_attitude_angle,
            "max_angular_speed": self.max_angular_speed,
        }

        for name, value in values.items():
            if not isfinite(value):
                raise ValueError(
                    f"{name} must be finite"
                )

            if value < 0.0:
                raise ValueError(
                    f"{name} must be >= 0"
                )

        optional_values = {
            "max_simultaneous_contact_normal_speed": (
                self.max_simultaneous_contact_normal_speed
            ),
            "max_simultaneous_contact_tangential_speed": (
                self.max_simultaneous_contact_tangential_speed
            ),
        }

        for name, value in optional_values.items():
            if value is None:
                continue

            if not isfinite(value):
                raise ValueError(
                    f"{name} must be finite"
                )

            if value < 0.0:
                raise ValueError(
                    f"{name} must be >= 0"
                )

    @property
    def effective_simultaneous_normal_speed_limit(self) -> float:
        """Return the normal-speed limit for simultaneous contact."""
        if self.max_simultaneous_contact_normal_speed is not None:
            return self.max_simultaneous_contact_normal_speed

        return min(
            self.max_first_contact_normal_speed,
            self.max_second_contact_normal_speed,
        )

    @property
    def effective_simultaneous_tangential_speed_limit(self) -> float:
        """Return the tangential-speed limit for simultaneous contact."""
        if self.max_simultaneous_contact_tangential_speed is not None:
            return self.max_simultaneous_contact_tangential_speed

        return min(
            self.max_first_contact_tangential_speed,
            self.max_second_contact_tangential_speed,
        )


@dataclass(frozen=True, slots=True)
class LandingContactMetrics2D:
    """Kinematic and energetic metrics for one sequential foot contact.

    The energy values describe vehicle kinetic energy immediately before
    and after the idealized impact transition. They are model quantities,
    not structural energy absorbed by the physical landing leg.
    """

    foot: SingleContactFoot2D
    time: float

    normal_speed: float
    tangential_speed: float

    vehicle_theta: float
    vehicle_omega: float

    cg_vx: float
    cg_vz: float

    kinetic_energy_before: float
    kinetic_energy_after: float
    dissipated_energy: float

    def __post_init__(self) -> None:
        finite_values = {
            "time": self.time,
            "normal_speed": self.normal_speed,
            "tangential_speed": self.tangential_speed,
            "vehicle_theta": self.vehicle_theta,
            "vehicle_omega": self.vehicle_omega,
            "cg_vx": self.cg_vx,
            "cg_vz": self.cg_vz,
            "kinetic_energy_before": self.kinetic_energy_before,
            "kinetic_energy_after": self.kinetic_energy_after,
            "dissipated_energy": self.dissipated_energy,
        }

        for name, value in finite_values.items():
            if not isfinite(value):
                raise ValueError(
                    f"{name} must be finite"
                )

        non_negative_values = {
            "normal_speed": self.normal_speed,
            "tangential_speed": self.tangential_speed,
            "kinetic_energy_before": self.kinetic_energy_before,
            "kinetic_energy_after": self.kinetic_energy_after,
            "dissipated_energy": self.dissipated_energy,
        }

        for name, value in non_negative_values.items():
            if value < 0.0:
                raise ValueError(
                    f"{name} must be >= 0"
                )


@dataclass(frozen=True, slots=True)
class LandingEvaluation2D:
    """Final evaluation of a geometrically completed sequential landing."""

    completed: bool
    successful: bool

    first_contact: LandingContactMetrics2D
    second_contact: LandingContactMetrics2D

    horizontal_position_error: float
    time_between_contacts: float
    total_dissipated_energy: float

    failed_criteria: tuple[LandingFailureReason2D, ...]

    def __post_init__(self) -> None:
        if not self.completed:
            raise ValueError(
                "LandingEvaluation2D represents a completed landing"
            )

        values = {
            "horizontal_position_error": (
                self.horizontal_position_error
            ),
            "time_between_contacts": (
                self.time_between_contacts
            ),
            "total_dissipated_energy": (
                self.total_dissipated_energy
            ),
        }

        for name, value in values.items():
            if not isfinite(value):
                raise ValueError(
                    f"{name} must be finite"
                )

        if self.time_between_contacts < 0.0:
            raise ValueError(
                "time_between_contacts must be >= 0"
            )

        if self.total_dissipated_energy < 0.0:
            raise ValueError(
                "total_dissipated_energy must be >= 0"
            )

        expected_success = not self.failed_criteria

        if self.successful is not expected_success:
            raise ValueError(
                "successful must match failed_criteria"
            )


@dataclass(frozen=True, slots=True)
class SimultaneousLandingContactMetrics2D:
    """Metrics for one simultaneous two-foot landing impact."""

    time: float

    left_normal_speed: float
    left_tangential_speed: float
    right_normal_speed: float
    right_tangential_speed: float

    vehicle_theta: float
    vehicle_omega: float

    cg_vx: float
    cg_vz: float

    kinetic_energy_before: float
    kinetic_energy_after: float
    dissipated_energy: float

    def __post_init__(self) -> None:
        values = {
            "time": self.time,
            "left_normal_speed": self.left_normal_speed,
            "left_tangential_speed": self.left_tangential_speed,
            "right_normal_speed": self.right_normal_speed,
            "right_tangential_speed": self.right_tangential_speed,
            "vehicle_theta": self.vehicle_theta,
            "vehicle_omega": self.vehicle_omega,
            "cg_vx": self.cg_vx,
            "cg_vz": self.cg_vz,
            "kinetic_energy_before": self.kinetic_energy_before,
            "kinetic_energy_after": self.kinetic_energy_after,
            "dissipated_energy": self.dissipated_energy,
        }

        for name, value in values.items():
            if not isfinite(value):
                raise ValueError(
                    f"{name} must be finite"
                )

        non_negative = {
            "left_normal_speed": self.left_normal_speed,
            "left_tangential_speed": self.left_tangential_speed,
            "right_normal_speed": self.right_normal_speed,
            "right_tangential_speed": self.right_tangential_speed,
            "kinetic_energy_before": self.kinetic_energy_before,
            "kinetic_energy_after": self.kinetic_energy_after,
            "dissipated_energy": self.dissipated_energy,
        }

        for name, value in non_negative.items():
            if value < 0.0:
                raise ValueError(
                    f"{name} must be >= 0"
                )


@dataclass(frozen=True, slots=True)
class SimultaneousLandingEvaluation2D:
    """Evaluation of a completed simultaneous two-foot landing."""

    completed: bool
    successful: bool

    contact: SimultaneousLandingContactMetrics2D

    horizontal_position_error: float
    total_dissipated_energy: float

    failed_criteria: tuple[LandingFailureReason2D, ...]

    def __post_init__(self) -> None:
        if not self.completed:
            raise ValueError(
                "simultaneous landing evaluation must be completed"
            )

        if not isfinite(self.horizontal_position_error):
            raise ValueError(
                "horizontal_position_error must be finite"
            )

        if (
            not isfinite(self.total_dissipated_energy)
            or self.total_dissipated_energy < 0.0
        ):
            raise ValueError(
                "total_dissipated_energy must be finite and >= 0"
            )

        if self.successful is not (not self.failed_criteria):
            raise ValueError(
                "successful must match failed_criteria"
            )


class LandingEvaluator2D:
    """Evaluate sequential first- and second-foot landing contacts.

    The evaluator is stateless. It does not advance the simulation and
    does not modify rigid-body or contact states.
    """

    def __init__(
        self,
        parameters: RigidBody2DParameters,
        geometry: LandingGearGeometry2D,
        criteria: LandingSuccessCriteria2D,
    ) -> None:
        self._parameters = parameters
        self._geometry = geometry
        self._criteria = criteria

    @property
    def parameters(self) -> RigidBody2DParameters:
        """Return the rigid-body parameters."""
        return self._parameters

    @property
    def geometry(self) -> LandingGearGeometry2D:
        """Return the landing-gear geometry."""
        return self._geometry

    @property
    def criteria(self) -> LandingSuccessCriteria2D:
        """Return the configured success criteria."""
        return self._criteria

    def evaluate_first_contact(
        self,
        *,
        time: float,
        pre_impact_state: HopperState2D,
        impact_result: FirstContactImpactResult2D,
    ) -> LandingContactMetrics2D:
        """Create metrics for the first sequential foot impact."""
        self._validate_time(time)

        normal_speed = max(
            0.0,
            -impact_result.contact_velocity_before_z,
        )

        tangential_speed = abs(
            impact_result.contact_velocity_before_x
        )

        return LandingContactMetrics2D(
            foot=impact_result.constraint.foot,
            time=time,
            normal_speed=normal_speed,
            tangential_speed=tangential_speed,
            vehicle_theta=pre_impact_state.theta,
            vehicle_omega=pre_impact_state.omega,
            cg_vx=pre_impact_state.vx,
            cg_vz=pre_impact_state.vz,
            kinetic_energy_before=max(
                0.0,
                impact_result.kinetic_energy_before,
            ),
            kinetic_energy_after=max(
                0.0,
                impact_result.kinetic_energy_after,
            ),
            dissipated_energy=max(
                0.0,
                impact_result.dissipated_energy,
            ),
        )

    def evaluate_second_contact(
        self,
        *,
        time: float,
        pre_impact_state: HopperState2D,
        detection: TouchdownDetection2D,
        first_contact: LandingContactMetrics2D,
        target_x: float,
    ) -> LandingEvaluation2D:
        """Evaluate the second foot contact and the completed landing."""
        self._validate_time(time)

        if not isfinite(target_x):
            raise ValueError(
                "target_x must be finite"
            )

        if time < first_contact.time:
            raise ValueError(
                "second contact time must be >= first contact time"
            )

        if detection.contact is not TouchdownContact2D.BOTH:
            raise ValueError(
                "second-contact evaluation requires BOTH contact"
            )

        second_foot = self._opposite_foot(
            first_contact.foot
        )

        foot_velocity_x, foot_velocity_z = (
            self._foot_velocity_world(
                pre_impact_state,
                second_foot,
            )
        )

        normal_speed = max(
            0.0,
            -foot_velocity_z,
        )

        tangential_speed = abs(
            foot_velocity_x
        )

        kinetic_energy_before = (
            self._kinetic_energy(
                pre_impact_state
            )
        )

        second_contact = LandingContactMetrics2D(
            foot=second_foot,
            time=time,
            normal_speed=normal_speed,
            tangential_speed=tangential_speed,
            vehicle_theta=pre_impact_state.theta,
            vehicle_omega=pre_impact_state.omega,
            cg_vx=pre_impact_state.vx,
            cg_vz=pre_impact_state.vz,
            kinetic_energy_before=kinetic_energy_before,
            kinetic_energy_after=0.0,
            dissipated_energy=kinetic_energy_before,
        )

        horizontal_position_error = (
            pre_impact_state.px
            - target_x
        )

        failed_criteria = self._failed_criteria(
            first_contact=first_contact,
            second_contact=second_contact,
            horizontal_position_error=(
                horizontal_position_error
            ),
        )

        total_dissipated_energy = (
            first_contact.dissipated_energy
            + second_contact.dissipated_energy
        )

        return LandingEvaluation2D(
            completed=True,
            successful=not failed_criteria,
            first_contact=first_contact,
            second_contact=second_contact,
            horizontal_position_error=(
                horizontal_position_error
            ),
            time_between_contacts=(
                time - first_contact.time
            ),
            total_dissipated_energy=(
                total_dissipated_energy
            ),
            failed_criteria=failed_criteria,
        )

    def evaluate_simultaneous_contact(
        self,
        *,
        time: float,
        pre_impact_state: HopperState2D,
        impact_result: SimultaneousContactImpactResult2D,
        target_x: float,
    ) -> SimultaneousLandingEvaluation2D:
        """Evaluate a genuine simultaneous two-foot landing impact."""
        self._validate_time(time)

        if not isfinite(target_x):
            raise ValueError(
                "target_x must be finite"
            )

        contact = SimultaneousLandingContactMetrics2D(
            time=time,
            left_normal_speed=max(
                0.0,
                -impact_result.left_contact_velocity_before_z,
            ),
            left_tangential_speed=abs(
                impact_result.left_contact_velocity_before_x
            ),
            right_normal_speed=max(
                0.0,
                -impact_result.right_contact_velocity_before_z,
            ),
            right_tangential_speed=abs(
                impact_result.right_contact_velocity_before_x
            ),
            vehicle_theta=pre_impact_state.theta,
            vehicle_omega=pre_impact_state.omega,
            cg_vx=pre_impact_state.vx,
            cg_vz=pre_impact_state.vz,
            kinetic_energy_before=max(
                0.0,
                impact_result.kinetic_energy_before,
            ),
            kinetic_energy_after=max(
                0.0,
                impact_result.kinetic_energy_after,
            ),
            dissipated_energy=max(
                0.0,
                impact_result.dissipated_energy,
            ),
        )

        horizontal_position_error = (
            pre_impact_state.px
            - target_x
        )

        failures: list[LandingFailureReason2D] = []
        criteria = self._criteria

        if (
            abs(horizontal_position_error)
            > criteria.max_horizontal_position_error
        ):
            failures.append(
                LandingFailureReason2D.HORIZONTAL_POSITION_ERROR
            )

        normal_limit = (
            criteria.effective_simultaneous_normal_speed_limit
        )

        tangential_limit = (
            criteria.effective_simultaneous_tangential_speed_limit
        )

        if contact.left_normal_speed > normal_limit:
            failures.append(
                LandingFailureReason2D.SIMULTANEOUS_LEFT_NORMAL_SPEED
            )

        if contact.left_tangential_speed > tangential_limit:
            failures.append(
                LandingFailureReason2D.SIMULTANEOUS_LEFT_TANGENTIAL_SPEED
            )

        if contact.right_normal_speed > normal_limit:
            failures.append(
                LandingFailureReason2D.SIMULTANEOUS_RIGHT_NORMAL_SPEED
            )

        if contact.right_tangential_speed > tangential_limit:
            failures.append(
                LandingFailureReason2D.SIMULTANEOUS_RIGHT_TANGENTIAL_SPEED
            )

        if (
            abs(contact.vehicle_theta)
            > criteria.max_attitude_angle
        ):
            failures.append(
                LandingFailureReason2D.ATTITUDE_ANGLE
            )

        if (
            abs(contact.vehicle_omega)
            > criteria.max_angular_speed
        ):
            failures.append(
                LandingFailureReason2D.ANGULAR_SPEED
            )

        failed_criteria = tuple(failures)

        return SimultaneousLandingEvaluation2D(
            completed=True,
            successful=not failed_criteria,
            contact=contact,
            horizontal_position_error=horizontal_position_error,
            total_dissipated_energy=(
                contact.dissipated_energy
            ),
            failed_criteria=failed_criteria,
        )

    def _failed_criteria(
        self,
        *,
        first_contact: LandingContactMetrics2D,
        second_contact: LandingContactMetrics2D,
        horizontal_position_error: float,
    ) -> tuple[LandingFailureReason2D, ...]:
        criteria = self._criteria
        failures: list[LandingFailureReason2D] = []

        if (
            abs(horizontal_position_error)
            > criteria.max_horizontal_position_error
        ):
            failures.append(
                LandingFailureReason2D.HORIZONTAL_POSITION_ERROR
            )

        if (
            first_contact.normal_speed
            > criteria.max_first_contact_normal_speed
        ):
            failures.append(
                LandingFailureReason2D.FIRST_CONTACT_NORMAL_SPEED
            )

        if (
            first_contact.tangential_speed
            > criteria.max_first_contact_tangential_speed
        ):
            failures.append(
                LandingFailureReason2D.FIRST_CONTACT_TANGENTIAL_SPEED
            )

        if (
            second_contact.normal_speed
            > criteria.max_second_contact_normal_speed
        ):
            failures.append(
                LandingFailureReason2D.SECOND_CONTACT_NORMAL_SPEED
            )

        if (
            second_contact.tangential_speed
            > criteria.max_second_contact_tangential_speed
        ):
            failures.append(
                LandingFailureReason2D.SECOND_CONTACT_TANGENTIAL_SPEED
            )

        if (
            abs(second_contact.vehicle_theta)
            > criteria.max_attitude_angle
        ):
            failures.append(
                LandingFailureReason2D.ATTITUDE_ANGLE
            )

        if (
            abs(second_contact.vehicle_omega)
            > criteria.max_angular_speed
        ):
            failures.append(
                LandingFailureReason2D.ANGULAR_SPEED
            )

        return tuple(failures)

    def _kinetic_energy(
        self,
        state: HopperState2D,
    ) -> float:
        p = self._parameters

        return (
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

    def _foot_velocity_world(
        self,
        state: HopperState2D,
        foot: SingleContactFoot2D,
    ) -> tuple[float, float]:
        body_x = self._foot_body_x(foot)
        body_z = -self._geometry.cg_to_feet

        r_x = (
            body_x * cos(state.theta)
            + body_z * sin(state.theta)
        )

        r_z = (
            -body_x * sin(state.theta)
            + body_z * cos(state.theta)
        )

        velocity_x = (
            state.vx
            + state.omega * r_z
        )

        velocity_z = (
            state.vz
            - state.omega * r_x
        )

        return velocity_x, velocity_z

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

    @staticmethod
    def _opposite_foot(
        foot: SingleContactFoot2D,
    ) -> SingleContactFoot2D:
        if foot is SingleContactFoot2D.LEFT:
            return SingleContactFoot2D.RIGHT

        return SingleContactFoot2D.LEFT

    @staticmethod
    def _validate_time(time: float) -> None:
        if not isfinite(time):
            raise ValueError(
                "time must be finite"
            )
