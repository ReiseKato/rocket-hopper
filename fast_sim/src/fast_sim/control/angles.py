"""Angle utilities used by controllers."""

from math import isfinite, pi


def wrap_angle_pi(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi)."""
    if not isfinite(angle):
        raise ValueError("angle must be finite")

    return (angle + pi) % (2.0 * pi) - pi
