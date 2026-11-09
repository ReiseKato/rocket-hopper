# Rocket Hopper Fast Simulator

This directory contains the fast, mathematically transparent simulation backend of the Rocket Hopper project.

The current implementation is intentionally limited to the deterministic 2D rigid-body state derivative. It does not yet include an integrator, controller, actuator dynamics, logging, or visualization.

From the repository root, install development dependencies with:

```bash
python -m pip install -e "./fast_sim[dev,docs]"
```

Run the tests with:

```bash
python -m pytest fast_sim/tests
```

See `docs/fast_sim/` for the physical model and architecture documentation.
