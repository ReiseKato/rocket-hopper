import math

from fast_sim.dynamics import Hopper2DDynamics
from fast_sim.integration import EulerIntegrator
from fast_sim.models import (
    HopperActuation2D,
    HopperState2D,
    RigidBody2DParameters,
)
from fast_sim.simulation import HopperSimulator


parameters = RigidBody2DParameters(
    mass=2.0,
    inertia_y=0.08,
    engine_offset=0.25,
)

dynamics = Hopper2DDynamics(parameters)

state = HopperState2D(
    px=0.0,
    pz=2.0,
    vx=0.0,
    vz=0.0,
    theta=0.0,
    omega=0.0,
)

simulator = HopperSimulator(
    dynamics=dynamics,
    integrator=EulerIntegrator(),
    initial_state=state,
)

actuation = HopperActuation2D(
    thrust=parameters.mass * parameters.gravity,
    gimbal_angle=math.radians(5.0),
)

for _ in range(5):
    state = simulator.step(
        actuation=actuation,
        dt=0.01,
    )

    print(
        f"t={simulator.time:.2f} "
        f"px={state.px:.8f} "
        f"pz={state.pz:.8f} "
        f"vx={state.vx:.8f} "
        f"vz={state.vz:.8f} "
        f"theta={math.degrees(state.theta):.4f} deg "
        f"omega={math.degrees(state.omega):.4f} deg/s"
    )